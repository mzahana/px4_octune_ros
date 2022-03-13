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

from px4_octune_ros.msg import TuningState

import utils
from process_data import ProcessData

class PX4Tuner:
    def __init__(self):
        # Print debug messages
        self._debug = rospy.get_param('~debug', False)

        # Desired duration of collected data samples, seconds
        self._data_len_sec = rospy.get_param('~data_len_sec', 1.0)

        # Sampling/upsampling time, seconds
        self._sampling_dt = rospy.get_param('~sampling_dt', 0.01)

        # Maximum array length, maybe dependent on excitiation time and sampling time?
        self._max_arr_L=int(ceil(self._data_len_sec/self._sampling_dt))

        # Flag to record/store data
        self._record_data=False

        # Date object
        self._data = ProcessData()
        self._data._sampling_dt = self._sampling_dt
        self._data._data_len_sec = self._data_len_sec
        self._data._raw_data_freq = 50.0
        self._data_buffering_timeout = self._data_len_sec + 0.1
        self._data_buffering_start_t = time.time()

        # PID gains
        self._roll_rate_pid = utils.PIDGains()
        self._pitch_rate_pid = utils.PIDGains()
        self._vel_xy_pid = utils.PIDGains()
        self._vel_z_pid = utils.PIDGains()

        # Optimization objects
        self._roll_rate_optimizer=BackProbOptimizer()
        self._roll_rate_optimizer._debug=False
        self._roll_rate_optimizer._alpha=0.002
        self._roll_rate_optimizer.updateAlphaList()
        self._roll_rate_optimizer._use_optimal_alpha = True # optimize learning rate to gurantee convergence
        self._roll_rate_optimizer._use_adam = False # Use gradient descent
        # Useful for plotting
        self._roll_rate_init_data = {'r':[], 'u':[], 'y':[]}
        self._roll_rate_final_data = {'r':[], 'u':[], 'y':[]}
        # ROS msg
        self._roll_rate_msg = TuningState()

        self._pitch_rate_optimizer=BackProbOptimizer()
        self._pitch_rate_optimizer._debug=False
        self._pitch_rate_optimizer._alpha=0.002
        self._pitch_rate_optimizer.updateAlphaList()
        self._pitch_rate_optimizer._use_optimal_alpha = True # optimize learning rate to gurantee convergence
        self._pitch_rate_optimizer._use_adam = False # Use gradient descent
        # Useful for plotting
        self._pitch_rate_init_data = {'r':[], 'u':[], 'y':[]}
        self._pitch_rate_final_data = {'r':[], 'u':[], 'y':[]}
        # ROS msg
        self._pitch_rate_msg = TuningState()

        self._vel_xy_optimizer=BackProbOptimizer()
        self._vel_xy_optimizer._debug=False
        self._vel_xy_optimizer._alpha=0.002
        self._vel_xy_optimizer._use_optimal_alpha = True # optimize learning rate to gurantee convergence
        self._vel_xy_optimizer._use_adam = False # Use gradient descent

        self._vel_z_optimizer=BackProbOptimizer()
        self._vel_z_optimizer._debug=False
        self._vel_z_optimizer._alpha=0.002
        self._vel_z_optimizer._use_optimal_alpha = True # optimize learning rate to gurantee convergence
        self._vel_z_optimizer._use_adam = False # Use gradient descent

        # TODO need optimizers for angular and linear positions (Just P controllers )

        # Max optimization iterations
        self._opt_max_iter=rospy.get_param('~opt_max_iter', 1000)
        # Max optimization time (seconds)
        self._opt_max_time=rospy.get_param('~opt_max_time', 60.0)
        self._opt_max_failures = rospy.get_param("~opt_max_failures", 10)
        self._current_opt_iteration = 0
        self._opt_failure_counter = 0
        self._opt_start_t = time.time()

        # Flag to stop tuning process
        self._start_tuning=False
        # Tuning indicator
        self._is_tuning_running=False
        # Flag to start/stop inserting data into data buffers
        self._insert_data = True

        # States of the tuning state machine
        self.IDLE_STATE = False
        self.GET_INIT_GAINS_STATE = False
        self.GET_DATA_STATE = False
        self.OPTIMIZATION_STATE = False


        # ----------------------------------- Publishers -------------------------- #
        self._roll_rate_pub = rospy.Publisher("roll_rate/tuning_state", TuningState, queue_size=10)
        self._pitch_rate_pub = rospy.Publisher("pitch_rate/tuning_state", TuningState, queue_size=10)

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

        # ----------------------------------- Timers -------------------------- #
    

        # ----------------------------------- Services -------------------------- #
        # Service for testing data pre-process
        rospy.Service('px4_octune/start_tune', Empty, self.startTuningSrv)
        # Service for testing data pre-process
        rospy.Service('px4_octune/stop_tune', Empty, self.stopTuningSrv)
        # Plot service
        rospy.Service('px4_octune/plot', Empty, self.plotSrv)

        # Request higher data streams
        ret = self.increaseStreamRates()
        if not ret:
            rospy.logerr("Could not request higher stream rate. Shutting down px4_tuner_node ...")
            exit()

    # ---------------------------- Callbacks ----------------------------#
    def startTuningSrv(self, req):
        self.startTuning()

        return []

    def stopTuningSrv(self, req):
        self.stopTuning()

        return []

    def plotSrv(self, req):
        self.plotData()

        return []

    def cmdPosCb(self, msg):
        """Commanded position callback
        """
        if msg is None:
            return
        # TODO

    def cmdAttCb(self, msg):
        """Callback of commanded attitude, and attitude rates
        """
        if msg is None:
            return

        if not self._insert_data:
            return

        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000

        # Commanded attitude
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
        euler = tf.transformations.euler_from_quaternion(q)
        roll_x=euler[2]
        pitch_y=euler[1]
        yaw_z=euler[0]

        self._data._cmd_att_dict = self._data.insertData(dict=self._data._cmd_att_dict, t=t_micro, x=roll_x, y=pitch_y, z=yaw_z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "_cmd_att_dict length is: %s",len(self._cmd_att_dict['time']))

        # Commanded angular rates
        x = msg.body_rate.x # commanded roll rate
        y = msg.body_rate.y # commanded pitch rate
        z = msg.body_rate.z # commanded yaw rate

        
        self._data._cmd_att_rate_dict = self._data.insertData(dict=self._data._cmd_att_rate_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "cmd_att_rate_dict length is: %s",len(self._cmd_att_rate_dict['time']))


    def ratesPIDOutputCb(self, msg):
        """The output (controller output) of the angular rates PID controller
        """
        if msg is None:
            return

        if not self._insert_data:
            return

        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000

        x=msg.controls[0] # roll index
        y=msg.controls[1] # pitch index
        z=msg.controls[2] # yaw index

        self._data._ang_rate_cnt_output_dict = self._data.insertData(dict=self._data._ang_rate_cnt_output_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "_ang_rate_cnt_output_dict length is: %s",len(self._ang_rate_cnt_output_dict['time']))


    def rawImuCb(self, msg):
        """Raw IMU values (feedback); gyros, accelerometers"""
        if msg is None:
            return

        if not self._insert_data:
            return
        
        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000
        x=msg.angular_velocity.x
        y=msg.angular_velocity.y
        z=msg.angular_velocity.z

        self._data._att_rate_dict = self._data.insertData(dict=self._data._att_rate_dict, t=t_micro, x=x, y=y, z=z)
            # if self._debug:
            #     rospy.loginfo_throttle(1, "att_rate_dict length is: %s",len(self._att_rate_dict['time']))

    def imuCb(self, msg):
        """Processed IMU (feedback); attitude """
        if msg is None:
            return

        if not self._insert_data:
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
        # euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
        euler = tf.transformations.euler_from_quaternion(q)
        roll_x=euler[0]
        pitch_y=euler[1]
        yaw_z=euler[2]

        self._data._att_dict = self._data.insertData(dict=self._data._att_dict, t=t_micro, x=roll_x, y=pitch_y, z=yaw_z)
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

    def startTuning(self):
        """Starts the tuning process"""
        if self._is_tuning_running:
            rospy.logwarn("Tuning is already in progress")
        else:
            self.resetStates()
            self.IDLE_STATE = True
            self._start_tuning = True
            self._insert_data = True
            rospy.loginfo("Tuning is started")

    def stopTuning(self):
        """Stops the tuning process"""
        self.resetStates()
        self._start_tuning=False
        self._is_tuning_running = False
        self._opt_failure_counter=0
        self._current_opt_iteration = 0
        self._data.resetDict() # clear data buffers
        self.IDLE_STATE=True
    
        rospy.loginfo("Tuning is stopped")

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

    def publishRollRateMsg(self, ros_time):
        """Updates the self._roll_rate_msg and publishes it

        Parameters
        --
        @param ros_time ROS time stamp rospy.Time
        """
        self._roll_rate_msg.header.stamp= ros_time
        self._roll_rate_msg.iteration_number = self._current_opt_iteration
        self._roll_rate_msg.learning_rate = self._roll_rate_optimizer._alpha
        self._roll_rate_msg.objective_value = self._roll_rate_optimizer.getObjective()
        self._roll_rate_msg.eig_value = self._roll_rate_optimizer._smallest_eig_val
        self._roll_rate_msg.kp = self._roll_rate_pid.kp
        self._roll_rate_msg.ki = self._roll_rate_pid.ki
        self._roll_rate_msg.kd = self._roll_rate_pid.kd
        self._roll_rate_pub.publish(self._roll_rate_msg)

    def publishPitchRateMsg(self, ros_time):
        """Updates the self._pitch_rate_msg and publishes it

        Parameters
        --
        @param ros_time ROS time stamp rospy.Time
        """
        self._pitch_rate_msg.header.stamp= ros_time
        self._pitch_rate_msg.iteration_number = self._current_opt_iteration
        self._pitch_rate_msg.learning_rate = self._pitch_rate_optimizer._alpha
        self._pitch_rate_msg.objective_value = self._pitch_rate_optimizer.getObjective()
        self._pitch_rate_msg.eig_value = self._pitch_rate_optimizer._smallest_eig_val
        self._pitch_rate_msg.kp = self._pitch_rate_pid.kp
        self._pitch_rate_msg.ki = self._pitch_rate_pid.ki
        self._pitch_rate_msg.kd = self._pitch_rate_pid.kd
        self._pitch_rate_pub.publish(self._pitch_rate_msg)

    def plotData(self):
        """
        This should not be used in real-time. This is is only for debugging, and it's called by a ROS service.
        It's supposed to plot everything ! Initial and final data, learning rates, PID gains, you name it!
        """
        if self._is_tuning_running:
            rospy.logerr("Tuning is still running. Stop tuning first.")
            return

        try:
            # Roll rate: initial response
            plt.title("Roll rate: Initial response")
            plt.plot(self._roll_rate_init_data['r'], 'r', label='Initial desired reference')
            plt.plot(self._roll_rate_init_data['y'], 'k', label='Initial response')
            plt.ylabel('rad/s')
            plt.ylim(-3,3)
            plt.legend()
            plt.show()

            plt.title("Roll rate: Final response")
            plt.plot(self._roll_rate_final_data['r'], 'r', label='Final desired reference')
            plt.plot(self._roll_rate_final_data['y'], 'k', label='Final response')
            plt.ylabel('rad/s')
            plt.ylim(-3,3)
            plt.legend()
            plt.show()

            # Roll rate:  Learning rate vs. optimization iterations
            plt.title("Roll rate: Learning rate vs. iteration")
            plt.plot(self._roll_rate_optimizer._alpha_list, 'k', label='Learning rate')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()

            # Roll rate:  Performance vs. optimization iterations
            plt.title("Roll rate: Performance vs. iteration")
            plt.plot(self._roll_rate_optimizer._performance_list, 'k', label='Performance')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()

            # Roll rate:  PID gains vs. iteration
            plt.title("Roll rate: PID gains vs. iteration")
            plt.plot(self._roll_rate_pid._kp_list, 'k', label='kp')
            plt.plot(self._roll_rate_pid._ki_list, 'r', label='ki')
            plt.plot(self._roll_rate_pid._kd_list, 'b', label='kd')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()

            # ----------------------------------- #
            # Pitch rate: initial response
            plt.title("Pitch rate: Initial response")
            plt.plot(self._pitch_rate_init_data['r'], 'r', label='Initial desired reference')
            plt.plot(self._pitch_rate_init_data['y'], 'k', label='Initial response')
            plt.ylabel('rad/s')
            plt.ylim(-3,3)
            plt.legend()
            plt.show()

            plt.title("Pitch rate: Final response")
            plt.plot(self._pitch_rate_final_data['r'], 'r', label='Final desired reference')
            plt.plot(self._pitch_rate_final_data['y'], 'k', label='Final response')
            plt.ylabel('rad/s')
            plt.ylim(-3,3)
            plt.legend()
            plt.show()

            # Pitch rate:  Learning rate vs. optimization iterations
            plt.title("Pitch rate: Learning rate vs. iteration")
            plt.plot(self._pitch_rate_optimizer._alpha_list, 'k', label='Learning rate')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()

            # Pitch rate:  Performance vs. optimization iterations
            plt.title("Pitch rate: Performance vs. iteration")
            plt.plot(self._pitch_rate_optimizer._performance_list, 'k', label='Performance')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()

            # Pitch rate:  PID gains vs. iteration
            plt.title("Pitch rate: PID gains vs. iteration")
            plt.plot(self._pitch_rate_pid._kp_list, 'k', label='kp')
            plt.plot(self._pitch_rate_pid._ki_list, 'r', label='ki')
            plt.plot(self._pitch_rate_pid._kd_list, 'b', label='kd')
            plt.xlabel('Iteration')
            plt.legend()
            plt.show()
        except:
            rospy.logerr("Error in plotData")

    #############################################################
    #                       State Machine                       #
    #############################################################

    def resetStates(self):
        """Reset all states to FALSE"""
        self.IDLE_STATE = False
        self.GET_INIT_GAINS_STATE = False
        self.GET_DATA_STATE = False
        self.OPTIMIZATION_STATE = False

    def execIdleState(self):
        """Implementation of IDLE_STATE"""
        rospy.loginfo_throttle(1, "Executing the IDLE STATE \n")

        if self._start_tuning:
            self._start_tuning = False
            self._is_tuning_running = True
            
            self._roll_rate_optimizer.resetLists()
            self._roll_rate_pid.resetLists()
            self._pitch_rate_optimizer.resetLists()
            self._pitch_rate_pid.resetLists()

            self.resetStates()
            self.GET_INIT_GAINS_STATE = True

    def execGetGainsState(self):
        """Implementation of GET_INIT_GAINS_STATE"""   
        rospy.loginfo_throttle(1, "Executing the GET_INIT_GAINS_STATE \n")

        # roll rate
        kp,ki,kd=self.getRatePIDGains(axis='ROLL')
        if kp is None:
            rospy.logerr("Could not get ROLLRATE PID gains. Exiting tuning process")
            self.stopTuning()
            return
        self._roll_rate_pid.kp = kp
        self._roll_rate_pid.ki = ki
        self._roll_rate_pid.kd = kd

        # pitch rate
        kp,ki,kd=self.getRatePIDGains(axis='PITCH')
        if kp is None:
            rospy.logerr("Could not get PITCHRATE PID gains. Exiting tuning process")
            self.stopTuning()
            return
        self._pitch_rate_pid.kp = kp
        self._pitch_rate_pid.ki = ki
        self._pitch_rate_pid.kd = kd

        self.resetStates()
        self.GET_DATA_STATE = True
        self._data_buffering_start_t= time.time()


    def execGetDataState(self):
        """Implementation of GET_DATA_STATE"""
        rospy.loginfo_throttle(1, "Executing the GET_DATA_STATE \n")

        if not self._insert_data:
            self._insert_data = True

        now = time.time()
        if ( (now - self._data_buffering_start_t) > self._data_buffering_timeout):
            rospy.logerr("Data buffering timedout. Aborting tuning process")
            self.stopTuning()
            return

        if (self._data.gotEnoughRateData(t=self._data_len_sec)):
            rospy.logdebug("[execGetDataState] Got enough angular rate raw data")

            self._insert_data = False

            # Process data
            data=self._data.prepRollRate()
            if data is None:
                rospy.logerr("Processed roll rate data has NaN value(s). Exiting tuning process")
                self.stopTuning()
                return

            data=self._data.prepPitchRate()
            if data is None:
                rospy.logerr("Processed pitch rate data has NaN value(s). Exiting tuning process")
                self.stopTuning()
                return

            self._insert_data = True

            self.resetStates()
            self.OPTIMIZATION_STATE=True
            if self._current_opt_iteration == 0:
                self._opt_start_t = time.time()
        #else: stay in this state to collect more data

    def execOptimizationState(self):
        """Implementation of OPTIMIZATION_STATE"""
        rospy.loginfo_throttle(1, "Executing the OPTIMIZATION_STATE \n")

        if (self._opt_failure_counter > self._opt_max_failures):
            rospy.logerr("Max optimization failures is reached. Aborting tuning process.\n")
            self.stopTuning()
            return

        if (self._current_opt_iteration > self._opt_max_iter):
            rospy.logwarn("Max optimization iterations {} is reached".format(self._opt_max_iter))
            self.stopTuning()
            return

        dt = time.time() - self._opt_start_t
        if (dt > self._opt_max_time):
            rospy.logerr("Max optimization time {} is reached".format(dt))
            self.stopTuning()
            return

        # Update current itertion number
        self._current_opt_iteration +=1
        rospy.loginfo_throttle(1, "Optimization iteration %s", self._current_opt_iteration)

        # update roll gains
        self._roll_rate_optimizer.setSignals(r=np.array(self._data._prep_roll_rate_cmd),u=np.array(self._data._prep_roll_cnt_output),y=np.array(self._data._prep_roll_rate))
        # num=utils.getPIDCoeffFromGains(kp=self._roll_rate_pid.kp, ki=self._roll_rate_pid.ki, kd=self._roll_rate_pid.kd, dt=self._sampling_dt)
        num=utils.getPIDCoeffFromGains(kp=self._roll_rate_pid.kp, ki=self._roll_rate_pid.ki, kd=self._roll_rate_pid.kd, dt=1.0)
        self._roll_rate_optimizer.setContCoeffs(den_coeff=[1,0,-1], num_coeff=num)
        good =self._roll_rate_optimizer.update(iter=self._current_opt_iteration)
        if not good:
            rospy.logerr("Roll rate optimization was not successful in iteration {}.".format(self._current_opt_iteration))
            self._opt_failure_counter +=1
        else: # Good
            # Get new conrtroller coeffs
            den,num=self._roll_rate_optimizer.getNewContCoeff()
            # kp,ki,kd=utils.getPIDGainsFromCoeff(num=num, dt=self._sampling_dt) 
            kp,ki,kd=utils.getPIDGainsFromCoeff(num=num, dt=1.0) 
            # TODO Is the following limiting method safe?
            K=utils.limitPIDGains(P=kp, I=ki, D=kd, kp_min=0.05, kp_max=0.5, ki_min=0.01, ki_max=0.4, kd_min=0.001, kd_max=0.005)
            if K is None:
                rospy.logerr("Roll rate PID coeffs is None. Skipping roll rate gain update")
                self._opt_failure_counter +=1
            else:
                good=self.setRatePIDGains(axis='ROLL', kP=K[0], kI=K[1], kD=K[2])
                if not good:
                    rospy.logerr("Error in sending new roll rate pid gains to PX4")
                    self._opt_failure_counter +=1
                else:
                    self._roll_rate_pid.kp = K[0]
                    self._roll_rate_pid.ki = K[1]
                    self._roll_rate_pid.kd = K[2]
                    self._roll_rate_pid.updateLists()

        # update pitch gains
        self._pitch_rate_optimizer.setSignals(r=np.array(self._data._prep_pitch_rate_cmd),u=np.array(self._data._prep_pitch_cnt_output),y=np.array(self._data._prep_pitch_rate))
        # num=utils.getPIDCoeffFromGains(kp=self._pitch_rate_pid.kp, ki=self._pitch_rate_pid.ki, kd=self._pitch_rate_pid.kd, dt=self._sampling_dt)
        num=utils.getPIDCoeffFromGains(kp=self._pitch_rate_pid.kp, ki=self._pitch_rate_pid.ki, kd=self._pitch_rate_pid.kd, dt=1)
        self._pitch_rate_optimizer.setContCoeffs(den_coeff=[1,0,-1], num_coeff=num)
        good =self._pitch_rate_optimizer.update(iter=self._current_opt_iteration)
        if not good:
            rospy.logerr("Pitch rate optimization was not successful in iteration {}.".format(self._current_opt_iteration))
            self._opt_failure_counter +=1
        else: # Good
            # Get new conrtroller coeffs
            den,num=self._pitch_rate_optimizer.getNewContCoeff()
            # kp,ki,kd=utils.getPIDGainsFromCoeff(num=num, dt=self._sampling_dt) 
            kp,ki,kd=utils.getPIDGainsFromCoeff(num=num, dt=1) 
            # TODO Is the following limiting method safe?
            K=utils.limitPIDGains(P=kp, I=ki, D=kd, kp_min=0.05, kp_max=0.6, ki_min=0.01, ki_max=0.4, kd_min=0.001, kd_max=0.005)
            if K is None:
                rospy.logerr("Pitch rate PID coeffs is None. Skipping pitch rate gain update")
                self._opt_failure_counter +=1
            else:
                good=self.setRatePIDGains(axis='PITCH', kP=K[0], kI=K[1], kD=K[2])
                if not good:
                    rospy.logerr("Error in sending new pitch rate pid gains to PX4")
                    self._opt_failure_counter +=1
                else:
                    self._pitch_rate_pid.kp = K[0]
                    self._pitch_rate_pid.ki = K[1]
                    self._pitch_rate_pid.kd = K[2]
                    self._pitch_rate_pid.updateLists()
                    # rospy.loginfo_throttle(1, "Pitch rate PID gains: P=%s I=%s D=%s", self._pitch_rate_pid.kp, self._pitch_rate_pid.ki, self._pitch_rate_pid.kd)

        # All good, update lists, go and collect data again to prepare for a new optimization iteration

        if self._current_opt_iteration == 1:
            self._roll_rate_init_data['r'] = self._data._prep_roll_rate_cmd
            self._roll_rate_init_data['y'] = self._data._prep_roll_rate
            self._roll_rate_init_data['u'] = self._data._prep_roll_cnt_output

            self._pitch_rate_init_data['r'] = self._data._prep_pitch_rate_cmd
            self._pitch_rate_init_data['y'] = self._data._prep_pitch_rate
            self._pitch_rate_init_data['u'] = self._data._prep_pitch_cnt_output
        
        self._roll_rate_final_data['r'] = self._data._prep_roll_rate_cmd
        self._roll_rate_final_data['y'] = self._data._prep_roll_rate
        self._roll_rate_final_data['u'] = self._data._prep_roll_cnt_output

        self._pitch_rate_final_data['r'] = self._data._prep_pitch_rate_cmd
        self._pitch_rate_final_data['y'] = self._data._prep_pitch_rate
        self._pitch_rate_final_data['u'] = self._data._prep_pitch_cnt_output        

        self._roll_rate_optimizer.updateAlphaList()
        self._roll_rate_optimizer.updatePerformanceList()
        self._roll_rate_optimizer.updateEigValList()

        self._pitch_rate_optimizer.updateAlphaList()
        self._pitch_rate_optimizer.updatePerformanceList()
        self._pitch_rate_optimizer.updateEigValList()

        # publish ROS msgs
        now = rospy.Time.now()
        self.publishRollRateMsg(now)
        self.publishPitchRateMsg(now)

        # clear data buffers, to prepare for receiving fresh data
        self._data.resetDict()
        self._data_buffering_start_t = time.time()

        self.resetStates()
        self.GET_DATA_STATE = True


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
        while(not self.gotEnoughRateData(t=self._data_len_sec)):
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
                while(not self.gotEnoughRateData(t=self._data_len_sec)):
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
            # TODO: switch between the tuning state machine's state
            if self.IDLE_STATE:
                self.execIdleState()
            elif self.GET_INIT_GAINS_STATE:
                self.execGetGainsState()
            elif self.GET_DATA_STATE:
                self.execGetDataState()
            elif self.OPTIMIZATION_STATE:
                self.execOptimizationState()
            else:
                rospy.logwarn_throttle(1, "\n Unsupported state \n")
            loop_rate.sleep()
        
        
def main():
    rospy.init_node("px4_tuner_node", anonymous=True)
    rospy.loginfo("Starting px4_tuner_node...\n")
    obj = PX4Tuner()
    obj._debug=True
    obj.IDLE_STATE=True
    obj.mainLoop()


if __name__ == "__main__":
    main()
