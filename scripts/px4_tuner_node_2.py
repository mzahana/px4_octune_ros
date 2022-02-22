#!/usr/bin/env python

import time
import rospy
from mavros_msgs.msg import AttitudeTarget, ActuatorControl, PositionTarget, State, ExtendedState
from mavros_msgs.srv import MessageInterval, MessageIntervalRequest, MessageIntervalResponse
from mavros_msgs.srv import ParamGet, ParamGetRequest, ParamGetResponse, ParamSet, ParamSetRequest, ParamSetResponse
from mavros_msgs.srv import CommandBool, SetMode
from rospy.core import logerr
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
from std_srvs.srv import Empty, Trigger, TriggerRequest
from math import ceil
from octune.optimization import BackProbOptimizer
from px4_octune_ros.srv import MaxVel, MaxVelRequest
import pandas as pd
import matplotlib.pyplot as plt
import tf
import numpy as np

import utils
from process_data import ProcessData

class PX4Tuner:
    def __init__(self):
        # Print debug messages
        self._debug = rospy.get_param('~debug', False)

        # System excitation time, seconds
        self._excitation_t = rospy.get_param('~excitation_t', 2.0)

        # Sampling/upsampling time, seconds
        self._sampling_dt = rospy.get_param('~sampling_dt', 0.005)

        # Maximum array length, maybe dependent on excitiation time and sampling time?
        self._max_arr_L=int(ceil(self._excitation_t/self._sampling_dt))

        # Flag to record/store data
        self._record_data=False

        # Optimization object
        # TODO Instantiate optimizer object for each controller (e.g. pich/roll rate, pitch/roll angles, xyz velocities, xyz positions)
        self._optimizer=BackProbOptimizer()
        self._optimizer._debug=False
        self._optimizer._alpha=0.002
        self._optimizer._use_optimal_alpha = True # optimize learnign rate to gurantee convergence
        self._optimizer._use_adam = False # Use gradient descent
        # Max optimization iterations
        self._opt_max_iter=rospy.get_param('~opt_max_iter', 300)
        # Max optimization time (seconds)
        self._opt_max_time=rospy.get_param('~opt_max_time', 60.0)

        # Flag to stop tuning process
        self._start_tuning=False

        # Tuning indicator
        self._is_tuning_running=False

        # States of the tuning state machine
        self.IDLE_STATE = True
        self.GET_INIT_GAINS_STATE = False
        self.GET_DATA_STATE = False
        self.OPTIMIZATION_STATE = False


        # ----------------------------------- Publishers -------------------------- #
        # Local position setpoint (mavros)
        self._pos_sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # ----------------------------------- Subscribers -------------------------- #
        # Commanded attitude and attitude rates
        rospy.Subscriber("mavros/setpoint_raw/target_attitude",AttitudeTarget, self.cmdAttCb)
        # Commanded angular rates
        rospy.Subscriber("mavros/target_actuator_control",ActuatorControl, self.ratesPIDOutputCb)
        # Commanded position/velocity/acceleration
        rospy.Subscriber("mavros/setpoint_raw/target_local", PositionTarget, self.cmdPosCb)

        # Feedback, angular velocity and linear acceleration
        rospy.Subscriber("mavros/imu/data_raw", Imu, self.rawImuCb)
        # Feedback, angular velocity and attitude
        rospy.Subscriber("mavros/imu/data", Imu, self.imuCb)
        # Feedback, local pose
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.poseCb)
        # Feedback, local velocity
        rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.velCb)
        # Mavros state (to get current flight mode and armed state)
        rospy.Subscriber("mavros/state", State, self.mavrosStateCb)
        # Mavros extended state (to check if drone is in air or landed)
        rospy.Subscriber("mavros/extended_state", ExtendedState, self.mavrosExtStatecb)

        # ----------------------------------- Timers -------------------------- #
    
        # Timer for setpoint calback
        # Can be shutdown by: self._setpoint_timer.shutdown()
        self._setpoint_timer = rospy.Timer(rospy.Duration(0.1), self.applySinPositionSetpoint)

        # ----------------------------------- Services -------------------------- #
        # Service for testing data pre-process
        rospy.Service('px4_octune/process_data', Empty, self.prepDataSrv)
        # Service for testing data pre-process
        rospy.Service('px4_octune/start_tune', Empty, self.startTuningSrv)
        # Service for testing data pre-process
        rospy.Service('px4_octune/stop_tune', Empty, self.stopTuningSrv)

        # Request higher data streams
        ret = self.increaseStreamRates()
        if not ret:
            rospy.logerr("Could not request higher stream rate. Shutting down px4_tuner_node ...")
            exit()

    # ---------------------------- Callbacks ----------------------------#

    def mavrosExtStatecb(self, msg):
        if msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self._isInAir = True
        else:
            self._isInAir = False
    
    def mavrosStateCb(self, msg):
        self._isArmed = msg.armed
        if msg.mode == 'OFFBOARD':
            self._isOffboard=True
        else:
            self._isOffboard=False

    def startTuningSrv(self, req):
        if self._is_tuning_running:
            rospy.logwarn("Tuning is already in progress")
        else:
            self._start_tuning = True
            rospy.loginfo("Tuning is started")

        return []

    def stopTuningSrv(self, req):
        self._start_tuning=False
        self.resetStates()
        self._is_tuning_running = False
        rospy.loginfo("Tuning is stopped")

        return []

    def cmdAttCb(self, msg):
        """Callback of commanded attitude, and attitude rates
        """
        if msg is None:
            return

        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000

        # Commanded attitude
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
        roll_x=euler[2]
        pitch_y=euler[1]
        yaw_z=euler[0]

        if self._record_data:
            self._cmd_att_dict = self.insertData(dict=self._cmd_att_dict, t=t_micro, x=roll_x, y=pitch_y, z=yaw_z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "_cmd_att_dict length is: %s",len(self._cmd_att_dict['time']))

        # Commanded angular rates
        x = msg.body_rate.x # commanded roll rate
        y = msg.body_rate.y # commanded pitch rate
        z = msg.body_rate.z # commanded yaw rate

        if self._record_data:
            self._cmd_att_rate_dict = self.insertData(dict=self._cmd_att_rate_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "cmd_att_rate_dict length is: %s",len(self._cmd_att_rate_dict['time']))


    def ratesPIDOutputCb(self, msg):
        """The output (controller output) of the angular rates PID controller
        """
        if msg is None:
            return

        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000

        x=msg.controls[0] # roll index
        y=msg.controls[1] # pitch index
        z=msg.controls[2] # yaw index

        if self._record_data:
            self._ang_rate_cnt_output_dict = self.insertData(dict=self._ang_rate_cnt_output_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "_ang_rate_cnt_output_dict length is: %s",len(self._ang_rate_cnt_output_dict['time']))


    def rawImuCb(self, msg):
        """Raw IMU values (feedback); gyros, accelerometers"""
        if msg is None:
            return
        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000
        x=msg.angular_velocity.x
        y=msg.angular_velocity.y
        z=msg.angular_velocity.z

        if self._record_data:
            self._att_rate_dict = self.insertData(dict=self._att_rate_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "att_rate_dict length is: %s",len(self._att_rate_dict['time']))

    def imuCb(self, msg):
        """Processed IMU (feedback); attitude """
        if msg is None:
            return
        # TODO
        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000

        # Construct a quaternion tuple
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
        roll_x=euler[2]
        pitch_y=euler[1]
        yaw_z=euler[0]

        if self._record_data:
            self._att_dict = self.insertData(dict=self._att_dict, t=t_micro, x=roll_x, y=pitch_y, z=yaw_z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "att_dict length is: %s",len(self._att_dict['time']))
    
    def poseCb(self, msg):
        """Pose (feedback) callback
        """
        if msg is None:
            return
        self._current_drone_pos = Point()
        self._current_drone_pos.x = msg.pose.position.x
        self._current_drone_pos.y = msg.pose.position.y
        self._current_drone_pos.z = msg.pose.position.z

    def velCb(self, msg):
        """Velocity (feedback) callback
        """
        if msg is None:
            return
        # TODO

    # ------------------------------------- Functions -------------------------------------#

    def resetStates(self):
        self.IDLE_STATE = False
        self.GET_INIT_GAINS_STATE = False
        self.GET_DATA_STATE = False
        self.OPTIMIZATION_STATE = False

    def execIdleState(self):
        rospy.loginfo_throttle(1, "Executing the IDLE STATE \n")

        if self._start_tuning:
            self._start_tuning = False
            self._is_tuning_running = True
            self.resetStates()
            self.GET_INIT_GAINS_STATE = True

    def execGetGainsState(self):
        # TODO: Implement
        # Gains of which controller ??
        pass

    def execGetDataState(self):
        # TODO Implement
        pass

    def execOptimizationState(Self):
        # TODO Implement
        pass

    def increaseStreamRates(self):
        """Requests from PX4 an increase in stream rates of commanded/feedback signals

        Returns
        --
        @return True if all stream requests succedded, False otherwise 
        """

        # MAVLINK MSG IDs
        ACTUATOR_CONTROL_TARGET=140
        ATTITUDE_TARGET=83
        POSITION_TARGET_LOCAL_NED=85
        HIGHRES_IMU=105
        ATTITUDE_QUATERNION=31
        LOCAL_POSITION_NED=32
        msg_id_list=[  ACTUATOR_CONTROL_TARGET,
                    ATTITUDE_TARGET,
                    POSITION_TARGET_LOCAL_NED,
                    HIGHRES_IMU,
                    ATTITUDE_QUATERNION,
                    LOCAL_POSITION_NED
                ]
        # TODO Implement rosservice call of /mavros/set_message_interval
        try:
            rospy.wait_for_service('mavros/set_message_interval', timeout=2.0)
        except rospy.ServiceException as e:
            rospy.logerr("Waiting for mavros/set_message_interval timedout!")
            is_req_sent=False
            return is_req_sent

        is_req_sent = True
        for id in msg_id_list:
            try:
                msg_stream_req = MessageIntervalRequest()
                msg_stream_req.message_id = id
                msg_stream_req.message_rate = 100.0 # Hz
                streamReqSrv = rospy.ServiceProxy('mavros/set_message_interval', MessageInterval)
                msg_stream_resp = streamReqSrv(msg_stream_req)
                is_req_sent = is_req_sent and  msg_stream_resp.success
                    
            except rospy.ServiceException as e:
                rospy.logerr('Failed to call mavros/set_message_interval for msg id= %s, error: %s', id,e)
                is_req_sent = is_req_sent and False

        if is_req_sent:
            rospy.loginfo("Requesting higher data stream: SUCCESS")
        
        return is_req_sent

    def getRatePIDGains(self, axis=None):
        """Requests from PX4 the gain values of the axis rate PID controller

        Parameters
        --
        @param axis Case sensitve 'ROLL', or 'PITCH', or 'YAW'

        Returns
        --
        @return axis_rate_kp float
        @return axis_rate_ki float
        @return axis_rate_kd float
        """
        axis_rate_kp = None
        axis_rate_ki = None
        axis_rate_kd = None

        if axis is None:
            rospy.logerr("Axis is not set;  axis=%s", axis)
            return axis_rate_kp, axis_rate_ki, axis_rate_kd

        if axis=='ROLL' or axis=='PITCH' or axis=='YAW':
            param_str_p='MC_'+axis+'RATE_P'
            param_str_i='MC_'+axis+'RATE_I'
            param_str_d='MC_'+axis+'RATE_D'
        else:
            rospy.logerr("axis value is not recognized: axis=%s", axis)
            return axis_rate_kp, axis_rate_ki, axis_rate_kd

        # request params
        # P gain
        try:
            rospy.wait_for_service('mavros/param/get', timeout=1.0)
            paramGetSrv=rospy.ServiceProxy('mavros/param/get', ParamGet)
            resp=paramGetSrv(param_id=param_str_p)
            if resp.success:
                axis_rate_kp=resp.value.real
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/get failed for %s: %s", param_str_p, e)

        # I gain
        try:
            rospy.wait_for_service('mavros/param/get', timeout=1.0)
            paramGetSrv=rospy.ServiceProxy('mavros/param/get', ParamGet)
            resp=paramGetSrv(param_id=param_str_i)
            if resp.success:
                axis_rate_ki=resp.value.real
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/get failed for %s: %s",param_str_i, e)

        # D gain
        try:
            rospy.wait_for_service('mavros/param/get', timeout=1.0)
            paramGetSrv=rospy.ServiceProxy('mavros/param/get', ParamGet)
            resp=paramGetSrv(param_id=param_str_d)
            if resp.success:
                axis_rate_kd=resp.value.real
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/get failed for %s: %s",param_str_d, e)

        if (self._debug):
            rospy.loginfo("PID gains %s=%s, %s=%s, %s=%s", param_str_p, axis_rate_kp, param_str_i, axis_rate_ki, param_str_d, axis_rate_kd)

        return axis_rate_kp, axis_rate_ki, axis_rate_kd

    def setRatePIDGains(self, axis=None, kP=None, kI=None, kD=None):
        """Sets PID gains of angular rate controllers of PX4

        Parameters
        --
        @param axis Case sensitve 'ROLL', or 'PITCH', or 'YAW'
        @param kP float P gain
        @param kI float I gain
        @param kD float D gain

        Returns
        --
        @return Bool True: gains are successfully set. False: otherwise.
        """
        if axis is None:
            rospy.logerr("Axis is not set; axis=%s", axis)
            return False

        if kP is None or kI is None or kD is None:
            rospy.logerr("One of the gains is None")
            return False

        if axis=='ROLL' or axis=='PITCH' or axis=='YAW':
            param_str_p='MC_'+axis+'RATE_P'
            param_str_i='MC_'+axis+'RATE_I'
            param_str_d='MC_'+axis+'RATE_D'
        else:
            rospy.logerr("axis value is not recognized: axis=%s", axis)
            return False

        req=ParamSetRequest()
        
        # P gain
        try:
            rospy.wait_for_service('mavros/param/set', timeout=1.0)
            paramSetSrv = rospy.ServiceProxy('mavros/param/set', ParamSet)
            req.param_id=param_str_p
            req.value.real=kP
            resp=paramSetSrv(req)
            if not resp.success:
                rospy.logerr("Setting %s is not successful", param_str_p)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/set failed: %s", e)
            return False

        # I gain
        try:
            rospy.wait_for_service('mavros/param/set', timeout=1.0)
            paramSetSrv = rospy.ServiceProxy('mavros/param/set', ParamSet)
            req.param_id=param_str_i
            req.value.real=kI
            resp=paramSetSrv(req)
            if not resp.success:
                rospy.logerr("Setting %s is not successful", param_str_i)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/set failed: %s", e)
            return False

        # D gain
        try:
            rospy.wait_for_service('mavros/param/set', timeout=1.0)
            paramSetSrv = rospy.ServiceProxy('mavros/param/set', ParamSet)
            req.param_id=param_str_d
            req.value.real=kD
            resp=paramSetSrv(req)
            if not resp.success:
                rospy.logerr("Setting %s is not successful", param_str_d)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/set failed: %s", e)
            return False

        # If reached here, means all good!
        return True


    # --------------------------------------------------------------------------- #
    #                               Tuning methods
    # --------------------------------------------------------------------------- #
    def tuneRatePID(self):
        """Tunes the angular rates PID loops
        """
        
        # Initial data
        # Get current PID gains
        kp,ki,kd=self.getRatePIDGains(axis='ROLL')
        start_kp = kp
        start_ki = ki
        start_kd = kd
        if kp is None:
            rospy.logerr("[tuneRatePID] Could not get ROLLRATE PID gains. Exiting tuning process")
            return

        # # Set current local position as a position setpoint to go to after tuning
        # try:
        #     rospy.wait_for_service('offboard_controller/use_current_pos', timeout=2.0)
        #     homeSrv=rospy.ServiceProxy('offboard_controller/use_current_pos', Trigger)
        #     homeSrv()
        # except rospy.ServiceException as e:
        #     rospy.logerr("service offboard_controller/use_current_pos call failed: %s. offboard_controller/use_current_pos Mode could not be set.", e)
        #     return 

        # aplly step input & record data
        self.resetDict()
        self.startRecordingData()
        # good = self.applyVelStepInput(step=0.5, duration=1.0)
        #good = self.applyPositionStepInput(step_xy=5.0, step_z=0.0, t=5.0)
        good = self.startSinPositionSetpoint()
        if (not good):
            rospy.logerr("[tuneRatePID] Error in applying step input. Exiting tuning process.")
            self._is_tuning_running=False
            self.stopRecordingData()
            return

        # Wait for some time for enough data to be collected    
        while(not self.gotEnoughRateData(t=self._excitation_t)):
            pass
        self.stopRecordingData()

        # Get initial signals
        cmd_roll_rate, roll_rate, pid_roll=self.prepRollRate()
        init_roll_rate = roll_rate # Used for plotting. Will be compared with final_roll_rate (after tuning)
        init_roll_cmd = cmd_roll_rate
        if cmd_roll_rate is None:
            rospy.logerr("Data has NaN value(s). Exiting tuning process")
            self._is_tuning_running=False
            return

        # Pass signals to the optimizer
        self._optimizer.setSignals(r=np.array(cmd_roll_rate),u=np.array(pid_roll),y=np.array(roll_rate))
        # Construct the PID numerator coefficients of the PID discrete transfer function
        # num=self.getPIDCoeffFromGains(kp=kp, ki=ki, kd=kd, dt=1.0)
        num=self.getPIDCoeffFromGains(kp=kp, ki=ki, kd=kd, dt=self._sampling_dt)
        self._optimizer.setContCoeffs(den_coeff=[1,0,-1], num_coeff=num)
        iter=1
        start_time = time.time()

        # The following lists are for plotting
        performance=[] # to store optimization objective values
        s_max_list = []
        kp_list=[]
        kp_list.append(kp)
        ki_list=[]
        ki_list.append(ki)
        kd_list=[]
        kd_list.append(kd)

        rospy.loginfo("[tuneRatePID] Startin the tuning loop")
        #------------------ Main tuning loop ---------------------#
        while (iter < self._opt_max_iter and (time.time() - start_time) < self._opt_max_time):
            self._is_tuning_running=True

            # Exit if a Stop is requested
            if (self._stop_tuning):
                self._stop_tuning=False
                self._is_tuning_running=False
                self.stopSinPositionSetpoint()
                self.stopRecordingData()
                break
            # Execute an optimization iteration
            good =self._optimizer.update(iter=iter)
            if(good):
                print("-------------- Did gradient update --------------- \n")
                # Get new conrtroller coeffs
                den,num=self._optimizer.getNewContCoeff()
                # kp,ki,kd=self.getPIDGainsFromCoeff(num=num, dt=1.0) # Always keep dt=1.0 !!
                kp,ki,kd=self.getPIDGainsFromCoeff(num=num, dt=self._sampling_dt) 
                # TODO Is the following limiting method safe?
                kp,ki,kd=self.limitPIDGains(P=kp, I=ki, D=kd, kp_min=0.05, kp_max=0.5, ki_min=0.01, ki_max=0.4, kd_min=0.001, kd_max=0.005)
                if kp is None:
                    rospy.logerr("PID coeffs is None. Exiting tuning process")
                    self._is_tuning_running=False
                    self.stopSinPositionSetpoint()
                    self.stopRecordingData()
                    break

                # NOTE Just for debugging
                rospy.loginfo("New PID gains P=%s, I=%s, D=%s", kp, ki, kd)

                # Store for plotting
                performance.append(self._optimizer.getObjective())
                kp_list.append(kp)
                ki_list.append(ki)
                kd_list.append(kd)

                # Update PX4 controller
                good=self.setRatePIDGains(axis='ROLL', kP=kp, kD=kd, kI=ki)
                if (not good):
                    rospy.logerr("Could not set PID values. Exiting tuning process")
                    self._is_tuning_running=False
                    break

                # TODO Check if new gains are OK!
                
                # Apply step input and get new signals
                self.resetDict()
                self.startRecordingData()
                # TODO Consider sending  position setpoints using mavros setpoint plugin
                # good = self.applyVelStepInput(step=0.5, duration=1.0)
                #good = self.applyPositionStepInput(step_xy=5.0, step_z=0.0, t=5.0)
                # if (not good):
                #     rospy.logerr("[tuneRatePID] Error in applying step input.Exiting tuning process.")
                #     self._is_tuning_running=False
                #     break
                while(not self.gotEnoughRateData(t=self._excitation_t)):
                    if(self._debug):
                        rospy.loginfo_throttle(1, "[tuneRatePID] Waiting for enough data to be collected")
                    pass
                self.stopRecordingData()
                # Get signals
                cmd_roll_rate, roll_rate, pid_roll=self.prepRollRate()
                # TODO get pitch rate data 
                

                # Update optimizer
                # r=reference signal, u=pid output, y=output(feedback)
                self._optimizer.setSignals(r=np.array(cmd_roll_rate),u=np.array(pid_roll),y=np.array(roll_rate))
                # num=self.getPIDCoeffFromGains(kp=kp, ki=ki, kd=kd, dt=1.0)
                num=self.getPIDCoeffFromGains(kp=kp, ki=ki, kd=kd, dt=self._sampling_dt)
                # The denominator coefficients of a discrete PID transfer function is always = [1, -1, 0]
                self._optimizer.setContCoeffs(den_coeff=[1,0,-1], num_coeff=num)

                s_max, w_max, f_max = self._optimizer.maxSensitivity(dt=self._sampling_dt)
                s_max_list.append(s_max)

                # Update iteration number
                iter +=1
            else:
                rospy.logerr("\n [tuneRatePID] Could not perform update step\n")
                self._is_tuning_running=False
                self.stopSinPositionSetpoint()
                self.stopRecordingData()
                break

        self.stopSinPositionSetpoint()
        self.stopRecordingData()
        rospy.loginfo(" [tuneRatePID] Done with atitude rate tuning llop")

        # plotting

        plt.plot(init_roll_cmd, 'r', label='Initial Desired reference')
        plt.plot(init_roll_rate, 'k', label='Initial response')
        plt.ylabel('rad/s')
        plt.legend()
        plt.show()
        
        final_roll_cmd = cmd_roll_rate
        final_roll_rate = roll_rate
        plt.plot(final_roll_cmd, 'k', label='Final reference')
        plt.plot(final_roll_rate, 'g', label='Tuned response')
        plt.ylabel('rad/s')
        plt.legend()
        plt.show()

        plt.plot(s_max_list, label='Maximum sensitivity')
        plt.xlabel('iteration number')
        plt.ylabel('s_max')
        plt.legend()
        plt.show()

        plt.plot(performance)
        plt.xlabel('iteration number')
        plt.ylabel('performance')
        plt.show()

        plt.plot(kp_list, label='Kp')
        plt.plot(ki_list, label='Ki')
        plt.plot(kd_list, label='Kd')
        plt.xlabel('iteration number')
        plt.ylabel('Gain value')
        plt.legend()
        plt.show()

        return

    def tuneVelPIDs(self):
        """Tunes linear velocity PID loops
        """
        # TODO 
        pass

    def mainLoop(self):
        """
        This is the main tuning ROS loop, It implements the tuning state machine.
        """
        loop_rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            # TODO: swithc between the tuning state machine
            loop_rate.sleep()
        
        
def main():
    rospy.init_node("px4_tuner_node", anonymous=True)
    rospy.loginfo("Starting px4_tuner_node...\n")
    obj = PX4Tuner()
    obj._debug=True
    obj.mainLoop()


if __name__ == "__main__":
    main()
