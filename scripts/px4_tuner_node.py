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

class FCUModes:
    def __init__(self):
	    pass    

    def setArm(self):
        try:
            rospy.wait_for_service('mavros/cmd/arming', timeout=2.0)
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: {}".format(e))
            return False

        return True

    def setDisarm(self):
        try:
            rospy.wait_for_service('mavros/cmd/arming', timeout=2.0)
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: {}".format(e))
            return False

        return True

    def setStabilizedMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode',timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Stabilized Mode could not be set.".format(e))
            return False

        return True

    def setOffboardMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Offboard Mode could not be set.".format(e))
            return False

        return True

    def setAltitudeMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.".format(e))
            return False

        return True

    def setPositionMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Position Mode could not be set.".format(e))
            return False

        return True

    def setHoldMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LOITER')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Auto Hold Mode could not be set.".format(e))
            return False

        return True

    def setAutoLandMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', timeout=2.0)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Autoland Mode could not be set.".format(e))
            return False

        return True

class PX4Tuner:
    def __init__(self):
        # Print debug messages
        self._debug = rospy.get_param('~debug', False)

        self._fcu_modes = FCUModes()

        # Armed state
        self. _isArmed = False
        # OFFBOARD
        self._isOffboard = False
        # Landed state
        self._isInAir = False

        # Angular rate controller output (actuator_controls msg in PX4); PID output
        self._ang_rate_cnt_output_dict= {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded attitude rates, dictionary, has four arrays, timesatmp and commanded attitude rates
        self._cmd_att_rate_dict=        {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded attitude rates, dictionary, has four arrays, timesatmp and commanded attitude
        self._cmd_att_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded velocity, dictionary, has four arrays, timesatmp and commanded velocity
        self._cmd_vel_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded position, dictionary, has four arrays, timesatmp and commanded position
        self._cmd_pos_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}

        # Feedback, attitude rates, dictionary, four arrays, time & data
        self._att_rate_dict=            {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, attitude,  dictionary, four arrays, time & data
        self._att_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, local velocity,  dictionary, four arrays, time & data
        self._vel_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, local position,  dictionary, four arrays, time & data
        self._pos_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}

        # System excitation time, seconds
        self._excitation_t = rospy.get_param('~excitation_t', 2.0)

        # Sampling/upsampling time, seconds
        self._sampling_dt = rospy.get_param('~sampling_dt', 0.005)

        # Tuning altitude
        self._tuning_alt=rospy.get_param('~tuning_alt', 5.0)

        # Current drone local position, Point()
        self._current_drone_pos=None

        # Maximum array length, maybe dependent on excitiation time and sampling time?
        self._max_arr_L=int(ceil(self._excitation_t/self._sampling_dt))

        # Flag to record/store data
        self._record_data=False

        # Setpoint msg
        self._setpoint_msg=Point()
        # Velocity setpoint msg, only XY, the Z component is altitude
        self._vel_sp_msg=Vector3()

        # Amplitude of setpoint sin wave, meter[s]
        self._sp_sin_amplitude = 0.5
        # Frequency of the sin wave, Hz
        self._sp_sin_freq = 1.0
        # True: send position (sin wave) setpoints to PX4
        self._send_sin_sp = False
        self._sp_sin_start_time = time.time()
        # Tuning starting position
        self._tuning_starting_pos = Point()

        # Optimization object
        # TODO Instantiate optimizer object for each controller (e.g. pich/roll rate, pitch/roll angles, xyz velocities, xyz positions)
        self._optimizer=BackProbOptimizer()
        self._optimizer._debug=False
        self._optimizer._alpha=0.002
        self._optimizer._use_optimal_alpha = True # optimize learnign rate
        self._optimizer._use_adam = False # Use gradient descent
        # Max optimization iterations
        self._opt_max_iter=rospy.get_param('~opt_max_iter', 300)
        # Max optimization time (seconds)
        self._opt_max_time=rospy.get_param('~opt_max_time', 60.0)

        # Flag to stop tuning process
        self._stop_tuning=False
        self._is_tuning_running=False


        # ----------------------------------- Publishers -------------------------- #
        # Setpoint publisher (mavros_offboard_controller.py)
        self._setpoint_pub = rospy.Publisher('offboard_controller/setpoint/local_pos', Point, queue_size=10)
        # Velocity setpoint publisher
        self._vel_sp_pub = rospy.Publisher('offboard_controller/setpoint/body_xy_vel', Vector3, queue_size=10)
        self._local_vel_sp_pub = rospy.Publisher('offboard_controller/setpoint/local_xy_vel', Vector3, queue_size=10)
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
        self.tuneRatePID()

        return []

    def stopTuningSrv(self, req):
        self._stop_tuning=True
        self.stopSinPositionSetpoint()
        self.stopRecordingData()    

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
        """Angular rates PID output
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

    def cmdPosCb(self, msg):
        """Commanded position callback
        """
        if msg is None:
            return
        # TODO

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
    def resetDict(self):
        """Clears all data dictionaries
        """
        self._ang_rate_cnt_output_dict= {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_att_rate_dict=        {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_att_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_vel_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_pos_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._att_rate_dict=            {'time':[], 'x':[], 'y':[], 'z':[]}
        self._att_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        self._vel_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        self._pos_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}

        return

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

    def insertData(self, dict=None, t=None, x=None, y=None, z=None):
        """Inserts a data point into a dictionary dict

        Parameters
        --
        @param dict Input dictionary with two arrays/lists, time and data
        @param t Timestamp in micro seconds
        @param x Data point in x axis
        @param y Data point in y axis
        @param z Data point in z axis

        Returns
        --
        @return out_dict Populated dictionary after data point insertion
        """
        if t is None or x is None or y is None or z is None or dict is None:
            rospy.logerr("[insertData] One of the input arguments is None")
            return
        # TODO Make sure that the input dict is with the right data structure, time & data arrays

        out_dict=dict
        out_dict['time'].append(t)
        out_dict['x'].append(x)
        out_dict['y'].append(y)
        out_dict['z'].append(z)
        if len(out_dict['time']) > self._max_arr_L:
            out_dict['time'].pop(0)
        if len(out_dict['x']) > self._max_arr_L:
            out_dict['x'].pop(0)
        if len(out_dict['y']) > self._max_arr_L:
            out_dict['y'].pop(0)
        if len(out_dict['z']) > self._max_arr_L:
            out_dict['z'].pop(0)

        return out_dict

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
            rospy.wait_for_service('mavros/param/get')
            paramGetSrv=rospy.ServiceProxy('mavros/param/get', ParamGet)
            resp=paramGetSrv(param_id=param_str_p)
            if resp.success:
                axis_rate_kp=resp.value.real
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/get failed for %s: %s", param_str_p, e)

        # I gain
        try:
            rospy.wait_for_service('mavros/param/get')
            paramGetSrv=rospy.ServiceProxy('mavros/param/get', ParamGet)
            resp=paramGetSrv(param_id=param_str_i)
            if resp.success:
                axis_rate_ki=resp.value.real
        except rospy.ServiceException as e:
            rospy.logerr("Calling mavros/param/get failed for %s: %s",param_str_i, e)

        # D gain
        try:
            rospy.wait_for_service('mavros/param/get')
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
            rospy.wait_for_service('mavros/param/set')
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
            rospy.wait_for_service('mavros/param/set')
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
            rospy.wait_for_service('mavros/param/set')
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
            
    def prepDataSrv(self, req):
        """Data pre-processing service (**THIS IS JUST FOR TESTING**) 
        """
        if(self._debug):
            rospy.loginfo("Stopping data recording")
        self.stopRecordingData()

        if(self._debug):
            rospy.loginfo("Resetting data dictionaries")
        self.resetDict()

        if(self._debug):
            rospy.loginfo("Starting data recording")
        self.startRecordingData()

        if (self._debug):
            rospy.loginfo("Applying step input")
        self.applyVelStepInput(step=2.0, duration=1.0)

        if(self._debug):
            rospy.loginfo("Stopping data recording")
        self.stopRecordingData()

        if(self._debug):
            rospy.loginfo("Processing Roll Rate data")
        roll_rate_cmd, roll_rate, roll_cnt = self.prepRollRate()

        if (roll_rate_cmd is not None and roll_rate is not None):
            rospy.loginfo("Roll rate data is processed")
        else:
            rospy.logwarn("Roll rate data processing failed")

        if(self._debug):
            rospy.loginfo("Processing Pitch Rate data")
        pitch_rate_cmd, pitch_rate, pitch_cnt = self.prepPitchRate()

        if (pitch_rate_cmd is not None and pitch_rate is not None):
            rospy.loginfo("Pitch rate data is processed")
        else:
            rospy.logwarn("Pitch rate data processing failed")

        if(self._debug):
            rospy.loginfo("Processing Roll data")
        roll_cmd, roll = self.prepRoll()

        if (roll_cmd is not None and roll is not None):
            rospy.loginfo("Roll data is processed")
        else:
            rospy.logwarn("Roll data processing failed")

        
        if self._debug:
            plt.show()

        return []

    def applySinPositionSetpoint(self, event):
        """Sends a sin wave of position setpoints at amplitude step w.r.t starting position, and frequency freq

        Uses class members
        --
        self._sp_sin_amplitude Amplitude of the sine wave in meters w.r.t to the starting position
        self._sp_sin_freq Sin wave frequency in Hz
        self._sp_sin_start_time

        @return Bool False if there is any error
        """
        if not self._send_sin_sp:
            return

        if self._debug:
            rospy.loginfo_throttle(1, "[applySinPositionSetpoint] Running applySinPositionSetpoint")

         # Sanity checks
        if (not self._isArmed):
            rospy.logerr_throttle(1, "[PX4_OCTUNE::applyPositionStepInput] Drone is not armed. Skipping position step input")
            return False

        if (not self._isInAir):
            rospy.logerr_throttle(1, "[PX4_OCTUNE::applyPositionStepInput] Drone is not in the air. Skipping position step input")
            return False

        # Position setpoint ROS msg
        sp_msg = PoseStamped()
        sp_msg.header.stamp = rospy.Time.now()
        t = time.time() - self._sp_sin_start_time
        sp_msg.pose.position.x = self._tuning_starting_pos.x + self._sp_sin_amplitude * np.sin(2.0 * np.pi * self._sp_sin_freq * t)
        sp_msg.pose.position.y = self._tuning_starting_pos.y + self._sp_sin_amplitude * np.sin(2.0 * np.pi * self._sp_sin_freq * t)
        sp_msg.pose.position.z = self._tuning_starting_pos.z    # Keep current height for now (Assuming it's in air!)

        self._pos_sp_pub.publish(sp_msg)

    def startSinPositionSetpoint(self):
        # Sanity check
        if self._current_drone_pos is None:
            rospy.logerr("[startSinPositionSetpoint] Can not start setpoint streaming. Current drone position is None")
            return False

        if not self._send_sin_sp:
            self._tuning_starting_pos = self._current_drone_pos
            self._sp_sin_start_time = time.time()
            # Set OFFBOARD mode
            if not self._isOffboard:
                good = self._fcu_modes.setOffboardMode()
                if not good:
                    rospy.logerr("[PX4_OCTUNE::applyPositionStepInput] Could not set offboard mode. Skipping position step input ")
                    return False

            # Activate 
            self._send_sin_sp = True
            
            return True
        else:
            rospy.logerr("[startSinpositionSetpoint] Position setpoints (sin wave) are already being sent")
            return False

    def stopSinPositionSetpoint(self):
        if not self._send_sin_sp:
            rospy.logwarn("[stopSinPositionSetpoint] Setpoint streaming is already stopped")
            return

        self._send_sin_sp = False
        # Go back to the starting position
        sp_msg = PoseStamped()
        sp_msg.pose.position.x = self._tuning_starting_pos.x
        sp_msg.pose.position.y = self._tuning_starting_pos.y
        sp_msg.pose.position.z = self._tuning_starting_pos.z

        # Just publish setpoint for sometime to make sure it's back home
        # Otherwise,  it wall go out of OFFBOARD mode
        rate = rospy.Rate(10)
        k=0
        while k < 20:
            sp_msg.header.stamp = rospy.Time.now()
            self._pos_sp_pub.publish(sp_msg)
            rate.sleep()
            k+=1

    def applyPositionStepInput(self, t=1.0, step_xy=1.0, step_z=0.0):
        """
        Sends local position step inputs to excite the system.
        Requires

        Parameters:
        --
        @param t setpoint publishing total time in seconds
        @param step_xy Position increment to the current XY local position, meters
        @param step_z Position increment to the current Z local position, meters

        Returns
        --
        @return Bool False if there is any error
        """

        # Sanity checks
        if (not self._isArmed):
            rospy.logerr("[PX4_OCTUNE::applyPositionStepInput] Drone is not armed. Skipping position step input")
            return False

        if (not self._isInAir):
            rospy.logerr("[PX4_OCTUNE::applyPositionStepInput] Drone is not in the air. Skipping position step input")
            return False

        # while loop to send position setpoints in offboard mode for t seconds
        # After t seconds set Hold flight mode
        
        # Save the current drone position
        starting_pos = self._current_drone_pos


        # Setpoint msg
        sp_msg = PoseStamped()

        rate = rospy.Rate(50)

        if not self._isOffboard:
            rospy.loginfo("Setting Offboard mode")
            # Need to publish some setpoints before activating OFFBOARD mode
            k=0
            sp_msg.pose.position = starting_pos
            while k < 10:
                k += 1
                sp_msg.header.stamp = rospy.Time.now()
                self._pos_sp_pub.publish(sp_msg)
                rate.sleep()

            # set OFFBOARD mode
            ret = self._fcu_modes.setOffboardMode()
            if not ret:
                rospy.logerr("[PX4_OCTUNE::applyPositionStepInput] Could not set offboard mode. Skipping position step input ")
                return False
                
        sp_msg.pose.position.x = starting_pos.x + step_xy
        sp_msg.pose.position.y = starting_pos.y + step_xy
        sp_msg.pose.position.z = starting_pos.z + step_z
        # Start time, sec
        start_t = time.time()
        while ( (time.time() - start_t) < t ):
            sp_msg.header.stamp = rospy.Time.now()
            self._pos_sp_pub.publish(sp_msg)
            rate.sleep()

        # Send drone back to initial position
        # TODO Not an accurate way!!!!
        sp_msg.pose.position = starting_pos
        k=0
        while k < 10:
            k += 1
            sp_msg.header.stamp = rospy.Time.now()
            self._pos_sp_pub.publish(sp_msg)
            rate.sleep()

        # Set Hold position mode
        if (not self._fcu_modes.setHoldMode() ):
            rospy.logerr("[PX4_OCTUNE::applyPositionStepInput] Could not set Hold mode. Skipping position step input ")
            return False

        return True


    def applyVelStepInput(self, step=2.0, duration=1.0):
        """Sends local velocity step inputs to excite the system.
        Requires mavros_offboard_controller.py node

        Parameters
        --
        @param step max velocity in m/s
        @param duration total step time in seconds

        Returns
        --
        @return Bool False if there is any error
        """

        # velocity setpoint in x-axis
        if (self._debug):
            rospy.loginfo("Sending velocity setpoint: %s m/s in local xy plane", step)
        self._vel_sp_msg.x=step # m/s
        self._vel_sp_msg.y=step
        self._vel_sp_msg.z = self._current_drone_pos.z #self._tuning_alt # this is altitude in meter(s)
        self._local_vel_sp_pub.publish(self._vel_sp_msg)

        # Activate offboard mode to engage offboard controller 
        try:
            rospy.wait_for_service('offboard_controller/arm_and_offboard', timeout=2.0)
            offbSrv = rospy.ServiceProxy('offboard_controller/arm_and_offboard', Empty)
            offbSrv()
        except rospy.ServiceException as e:
            rospy.logerr("service offboard_controller/arm_and_offboard call failed: %s. arm_and_offboard Mode could not be set.", e)

        if(self._debug):
            rospy.loginfo("Waiting for %s seconds", duration/2)
        rospy.sleep(rospy.Duration(secs=duration/2))

        #  xy velocity setpoint
        if (self._debug):
            rospy.loginfo("Sending velocity setpoint: %s m/s in local xy plane", step)
        self._vel_sp_msg.x=-1.0*step # m/s
        self._vel_sp_msg.y=-1.0*step
        self._vel_sp_msg.z = self._current_drone_pos.z #self._tuning_alt # this is altitude in meter(s)
        self._local_vel_sp_pub.publish(self._vel_sp_msg)

        if(self._debug):
            rospy.loginfo("Waiting for %s seconds", duration/2)
        rospy.sleep(rospy.Duration(secs=duration/2))

        # Go to last recordet position set point
        try:
            # This will bring the drone to the last recroded local position setpoints
            rospy.wait_for_service('offboard_controller/set_local_tracking', timeout=2.0)
            holdSrv = rospy.ServiceProxy('offboard_controller/set_local_tracking', Trigger)
            resp=holdSrv()
        except rospy.ServiceException as e:
            rospy.logerr("service offboard_controller/set_local_tracking call failed: %s. offboard_controller/set_local_tracking Mode could not be set.", e)

        return True

    def startRecordingData(self):
        """Sets _record_data=True so that the callbacks starts storing data in the dictionaries
        """
        self._record_data=True

    def stopRecordingData(self):
        """Sets _record_data=False so that the callbacks stop storing data in the dictionaries
        """
        self._record_data=False

    def gotEnoughRateData(self, t=1.0):
        # 50 is the expected number of samples/sec
        cmd_att_rate = len(self._cmd_att_rate_dict['time']) > int(ceil(t*50))
        att_rate = len(self._att_rate_dict['time']) > int(ceil(t*50))
        att_cnt = len(self._ang_rate_cnt_output_dict['time']) > int(ceil(t*50))
        if cmd_att_rate and att_rate and att_cnt:
            return True
        else:
            return False

    def prepData(self, cmd_df=None, fb_df=None, cnt_df=None, cmd_data_label=None, data_label=None, cnt_label=None):
        """Merges data (target and actual), resample, and align their timestamps.

        Parameters
        --
        @param cmd_df Pandas Dataframe of commanded data
        @param fb_df Pandas Dataframe of feedback data
        @param cmd_data_label column label of the commanded data
        @param data_label column label of the feedback data
        @param cnt_label column label of the controller output data, if available

        Returns
        --
        @return prep_cmd_data Prepared commanded data as a Python list
        @return prep_fb_data Prepared feedback data as a Python list
        """
        prep_cmd_data=None
        prep_data=None
        prep_cnt_data=None
        if cmd_df is None:
            rospy.logerr("Commanded dataframe is None")
            return prep_cmd_data,prep_data,prep_cnt_data
        if fb_df is None:
            rospy.logerr("Feedback dataframe is None")
            return prep_cmd_data,prep_data,prep_cnt_data
        if cmd_data_label is None:
            rospy.logerr("Commanded data label is None")
            return prep_cmd_data,prep_data,prep_cnt_data
        if data_label is None:
            rospy.logerr("Feedback data label is None")
            return prep_cmd_data,prep_data,prep_cnt_data
        if cnt_df is None:
            rospy.logerr("Control data is None")
            return prep_cmd_data,prep_data,prep_cnt_data
        if cnt_label is None:
            rospy.logerr("Control data label is None")
            return prep_cmd_data,prep_data,prep_cnt_data

        if self._debug:
            rospy.loginfo("[prepData] Length of %s=%s, length of %s=%s, length of %s=%s", cmd_data_label,len(cmd_df),data_label, len(fb_df), cnt_label,len(cnt_df))

        # Merge
        df_merged =cmd_df.merge(fb_df, how='outer', left_index=True, right_index=True)
        df_merged = df_merged.merge(cnt_df, how='outer', left_index=True, right_index=True)

        # Resample
        dt_str=str(self._sampling_dt*1000.0)+'L' # L is for milli-second in Pandas
        df_resampled = df_merged.resample(dt_str).mean().interpolate()

        # Find common time range
        min_idx=cmd_df.index[0]
        max_idx=cmd_df.index[-1]
        if ( min_idx < fb_df.index[0]):
            min_idx=fb_df.index[0]
        if min_idx < cnt_df.index[0]:
            min_idx=cnt_df.index[0]
        

        if (max_idx > fb_df.index[-1]):
            max_idx=fb_df.index[-1]
        if max_idx > cnt_df.index[-1] :
            max_idx=cnt_df.index[-1]

        df_common=df_resampled[min_idx:max_idx]

        if (self._debug):
            rospy.loginfo("Total time for processed data = %s seocond(s)\n", (max_idx-min_idx).total_seconds())
            # df_common.plot()
            #plt.show()

        # Finally extract processed data as lists
        prep_cmd_data =  df_common[cmd_data_label].tolist()
        prep_data = df_common[data_label].tolist()
        prep_cnt_data = df_common[cnt_label].tolist()
        
            
        return prep_cmd_data, prep_data, prep_cnt_data


    def prepRatesPIDOutput(self):
        """ Prepares PID output data

        Returns
        --
        @return prep_pid_roll, prep_pid_pitch, prep_pid_yaw PID output data (lists)
        """
        prep_rate_cmd=None
        prep_pid_roll=None
        prep_pid_pitch=None
        prep_pid_yaw=None

        # Sanity check
        if len(self._cmd_att_rate_dict['time']) < 1:
            rospy.logerr("[prepRatesPIDOutput] No commanded attitude rate data to process")
            return prep_pid_roll, prep_pid_pitch, prep_pid_yaw

        if len(self._ang_rate_cnt_output_dict['time']) < 1:
            rospy.logerr("[prepRatesPIDOutput] No actuator (PID output) data to process")
            return prep_pid_roll, prep_pid_pitch, prep_pid_yaw

        #roll
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_roll_rate = pd.DataFrame(self._cmd_att_rate_dict['x'], index=cmd_idx, columns =['cmd_roll_rate'])
        pid_roll_idx=pd.to_timedelta(self._ang_rate_cnt_output_dict['time'], unit='us') # 'us' = micro seconds
        pid_roll = pd.DataFrame(self._ang_rate_cnt_output_dict['x'], index=pid_roll_idx, columns =['pid_roll'])
        prep_roll_rate_cmd, prep_pid_roll=self.prepData(cmd_df=cmd_roll_rate, fb_df=pid_roll, cmd_data_label='cmd_roll_rate', data_label='pid_roll')

        #pitch
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_pitch_rate = pd.DataFrame(self._cmd_att_rate_dict['y'], index=cmd_idx, columns =['cmd_pitch_rate'])
        pid_pitch_idx=pd.to_timedelta(self._ang_rate_cnt_output_dict['time'], unit='us') # 'us' = micro seconds
        pid_pitch = pd.DataFrame(self._ang_rate_cnt_output_dict['y'], index=pid_pitch_idx, columns =['pid_pitch'])
        prep_pitch_rate_cmd, prep_pid_pitch=self.prepData(cmd_df=cmd_pitch_rate, fb_df=pid_pitch, cmd_data_label='cmd_pitch_rate', data_label='pid_pitch')

        #yaw
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_yaw_rate = pd.DataFrame(self._cmd_att_rate_dict['z'], index=cmd_idx, columns =['cmd_yaw_rate'])
        pid_yaw_idx=pd.to_timedelta(self._ang_rate_cnt_output_dict['time'], unit='us') # 'us' = micro seconds
        pid_yaw = pd.DataFrame(self._ang_rate_cnt_output_dict['y'], index=pid_yaw_idx, columns =['pid_yaw'])
        prep_yaw_rate_cmd, prep_pid_yaw=self.prepData(cmd_df=cmd_yaw_rate, fb_df=pid_yaw, cmd_data_label='cmd_yaw_rate', data_label='pid_yaw')

        return prep_pid_roll, prep_pid_pitch, prep_pid_yaw
        

    def prepRollRate(self):
        """Merges roll rate data (target and actual) and the corresponding controller output signal, resample, and align their timestamps.
        Uses self._cmd_att_rate_dict and self._att_rate_dict

        Returns
        --
        @return prep_roll_rate_cmd Numpy array
        @return prep_roll_rate Numpy array
        """
        prep_roll_rate_cmd=None
        prep_roll_rate=None
        prep_cnt_output=None
        # Sanity check
        if len(self._cmd_att_rate_dict['time']) < 1:
            rospy.logerr("[prepRollRate] No commanded attitude rate data to process")
            return prep_roll_rate_cmd, prep_roll_rate

        if len(self._att_rate_dict['time']) < 1:
            rospy.logerr("[prepRollRate] No feedback attitude rate data to process")
            return prep_roll_rate_cmd, prep_roll_rate
        
        

        # setup the time index list
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_roll_rate = pd.DataFrame(self._cmd_att_rate_dict['x'], index=cmd_idx, columns =['cmd_roll_rate'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_rate_dict['time'], unit='us') # 'us' = micro seconds
        fb_roll_rate = pd.DataFrame(self._att_rate_dict['x'], index=fb_idx, columns =['roll_rate'])
        # controller output
        cnt_idx=pd.to_timedelta(self._ang_rate_cnt_output_dict['time'], unit='us') # 'us' = micro seconds
        cnt_roll_rate = pd.DataFrame(self._ang_rate_cnt_output_dict['x'], index=cnt_idx, columns =['roll_cnt'])

        prep_roll_rate_cmd, prep_roll_rate, prep_cnt_output=self.prepData(cmd_df=cmd_roll_rate, fb_df=fb_roll_rate,cnt_df=cnt_roll_rate, cmd_data_label='cmd_roll_rate', data_label='roll_rate', cnt_label='roll_cnt')

        return prep_roll_rate_cmd, prep_roll_rate, prep_cnt_output

    def prepPitchRate(self):
        """Merges Pitch rate data (target and actual), resample, and align their timestamps.
        Uses self._cmd_att_rate_dict and self._att_rate_dict

        Returns
        --
        @return prep_pitch_rate_cmd Numpy array
        @return prep_pitch_rate Numpy array
        """
        prep_pitch_rate_cmd=None
        prep_pitch_rate=None
        # Sanity check
        if len(self._cmd_att_rate_dict['time']) < 1:
            rospy.logerr("[prepPitchRate] No commanded attitude rate data to process")
            return prep_pitch_rate_cmd, prep_pitch_rate

        if len(self._att_rate_dict['time']) < 1:
            rospy.logerr("[prepPitchRate] No feedback attitude rate data to process")
            return prep_pitch_rate_cmd, prep_pitch_rate
        
        # TODO implement data pre-processing
        # setup the time index list
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_pitch_rate = pd.DataFrame(self._cmd_att_rate_dict['y'], index=cmd_idx, columns =['cmd_pitch_rate'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_rate_dict['time'], unit='us') # 'us' = micro seconds
        fb_pitch_rate = pd.DataFrame(self._att_rate_dict['y'], index=fb_idx, columns =['pitch_rate'])

        prep_pitch_rate_cmd, prep_pitch_rate=self.prepData(cmd_df=cmd_pitch_rate, fb_df=fb_pitch_rate, cmd_data_label='cmd_pitch_rate', data_label='pitch_rate')

        return prep_pitch_rate_cmd, prep_pitch_rate

    def prepYawRate(self):
        """Merges Yaw rate data (target and actual), resample, and align their timestamps.
        Uses self._cmd_att_rate_dict and self._att_rate_dict

        Returns
        --
        @return prep_yaw_rate_cmd Numpy array
        @return prep_yaw_rate Numpy array
        """
        prep_yaw_rate_cmd=None
        prep_yaw_rate=None
        # Sanity check
        if len(self._cmd_att_rate_dict['time']) < 1:
            rospy.logerr("[prepYawRate] No commanded attitude rate data to process")
            return prep_yaw_rate_cmd, prep_yaw_rate

        if len(self._att_rate_dict['time']) < 1:
            rospy.logerr("[prepYawRate] No feedback attitude rate data to process")
            return prep_yaw_rate_cmd, prep_yaw_rate
        
        # TODO implement data pre-processing

        return prep_yaw_rate_cmd, prep_yaw_rate

    def prepRoll(self):
        """Merges Roll  data (target and actual), resample, and align their timestamps.
        Uses self._cmd_att_dict and self._att_dict

        Returns
        --
        @return prep_roll_cmd Numpy array
        @return prep_roll Numpy array
        """
        prep_roll_cmd=None
        prep_roll=None
        # Sanity check
        if len(self._cmd_att_dict['time']) < 1:
            rospy.logerr("[prepRoll] No commanded attitude data to process")
            return prep_roll_cmd, prep_roll

        if len(self._att_dict['time']) < 1:
            rospy.logerr("[prepRoll] No feedback attitude data to process")
            return prep_roll_cmd, prep_roll
        
        # TODO implement data pre-processing
        # setup the time index list, and roll Dataframe
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_dict['time'], unit='us') # 'us' = micro seconds
        cmd_roll = pd.DataFrame(self._cmd_att_dict['x'], index=cmd_idx, columns =['cmd_roll'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_dict['time'], unit='us') # 'us' = micro seconds
        fb_roll = pd.DataFrame(self._att_dict['x'], index=fb_idx, columns =['roll'])

        prep_roll_cmd, prep_roll=self.prepData(cmd_df=cmd_roll, fb_df=fb_roll, cmd_data_label='cmd_roll', data_label='roll')

        return prep_roll_cmd, prep_roll

    def prepPitch(self):
        """Merges Pitch  data (target and actual), resample, and align their timestamps.
        Uses self._cmd_att_dict and self._att_dict

        Returns
        --
        @return prep_pitch_cmd Numpy array
        @return prep_pitch Numpy array
        """
        prep_pitch_cmd=None
        prep_pitch=None
        # Sanity check
        if len(self._cmd_att_dict['time']) < 1:
            rospy.logerr("[prepPitch] No commanded attitude data to process")
            return prep_pitch_cmd, prep_pitch

        if len(self._att_dict['time']) < 1:
            rospy.logerr("[prepPitch] No feedback attitude data to process")
            return prep_pitch_cmd, prep_pitch
        
        # TODO implement data pre-processing
        # setup the time index list, and roll Dataframe
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_dict['time'], unit='us') # 'us' = micro seconds
        cmd_pitch = pd.DataFrame(self._cmd_att_dict['y'], index=cmd_idx, columns =['cmd_pitch'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_dict['time'], unit='us') # 'us' = micro seconds
        fb_pitch = pd.DataFrame(self._att_dict['y'], index=fb_idx, columns =['pitch'])

        prep_pitch_cmd, prep_pitch=self.prepData(cmd_df=cmd_pitch, fb_df=fb_pitch, cmd_data_label='cmd_pitch', data_label='pitch')

        return prep_pitch_cmd, prep_pitch

    def getPIDGainsFromCoeff(self,num=None, dt=0.001):
        """Computes Kp,Ki,Kd of a discrete PID controller from its transfer function's numerator coeff
        Numerator is of the form n0*Z^2+n1*Z+n2

        Parameters
        --
        @param num Numerator coeff =[n0,n1,n2]
        @param dt sampling time in seconds

        Returns
        --
        @return kp,ki,kd PID gains
        """
        kp=None
        ki=None
        kd=None
        if(num is None):
            rospy.logerr("[getPIDGainsFromCoeff] numerator coeffs array is None")
            return kp, ki, kd
        if( dt<=0):
            rospy.logerr("[getPIDGainsFromCoeff] sampling time should be >0")
            return kp, ki, kd

        n0=num[0]
        n1=num[1]
        n2=num[2]

        # kd=dt*n2
        # ki=(n0+n1+n2)/dt
        # kp=(n0+n1-n2)/2.

        # Ref: last equation (13) in https://www.scilab.org/discrete-time-pid-controller-implementation
        kp = (n0-n2)/2
        ki = (n0+n1+n2)/(2.*dt)
        kd = dt*(n0 - n1 + n2)/8.

        return kp,ki,kd

    def getPIDCoeffFromGains(self, kp=None, ki=None, kd=None, dt=0.001):
        """Computes transfer function's numerator of a discrete PID from its gains.
        The denominator is constant Z^2+Z+0
        The numnerator is of the form n0*Z^2+n1*Z+n2

        Parameters
        --
        @param kp Proportional gain
        @param ki Integral gain
        @param kd Derivtive gain
        @param dt sampling time in seconds
        
        Returns
        --
        @return num=[n0,n1,n2] list of PID numerator coeffs
        """
        if kp is None or ki is None or kd is None:
            rospy.logerr("[getPIDCoeffFromGains] One of the coeffs is None kp=%s, ki=%s, kd=%s", kp, ki, kd)
            return None

        if kp <0.0 or ki < 0.0 or kd<0.0:
            rospy.logerr("[getPIDCoeffFromGains] One of the coeffs is negative, kp=%s, ki=%s, kd=%s", kp, ki, kd)
            return None
        if( dt<=0):
            rospy.logerr("[getPIDCoeffFromGains] sampling time should be >0")
            return None

        # Numerator coeff
        # n0=kp+ki*dt/2.+kd/dt
        # n1=-kp+ki*dt/2.-2.*kd/dt
        # n2=kd/dt

        # Ref: last equation (13) in https://www.scilab.org/discrete-time-pid-controller-implementation
        n0 = (2*dt*kp + ki*dt**2 + 4*kd) /2./dt
        n1 = (2*ki*dt**2 - 8*kd) / 2./dt
        n2 = (ki*dt**2 - 2*dt*kp + 4*kd) /2./dt

        num=[n0,n1,n2]

        return num

    def limitPIDGains(self, P=None, I=None, D=None, kp_min=0.0, kp_max=0.5, ki_min=0.0, ki_max=0.4, kd_min=0.0, kd_max=0.005):

        kP=None
        kI=None
        kD=None
        if P is None or I is None or D is None:
            rospy.logerr("One of the coeffs is None")
            return kP, kI, kD
        kP=P
        kI=I
        kD=D
        
        if P < kp_min:
            kP=kp_min
        if P > kp_max:
            kP=kp_max
        if I<ki_min:
            kI=ki_min
        if I>ki_max:
            kI=ki_max
        if D<kd_min:
            kD=kd_min
        if D>kd_max:
            kD=kd_max

        return kP, kI, kD
        


    def alignData(self):
        t1=np.arange(10)+100.0
        t2=np.arange(5)+101.0
        d1=pd.to_timedelta(t1, unit='s')
        d2=pd.to_timedelta(t2, unit='s')
        print("--------------------")
        print(type(d1))

    # --------------------------------------------------------------------------- #
    #                               Tuning methods
    # --------------------------------------------------------------------------- #
    def tuneRatePID(self):
        """Tunes the angular rates PID loops
        """
        # TODO should make sure that drone is in the air before starting the tuning process

        # Set max velocities
        # try:
        #     rospy.wait_for_service('offboard_controller/max_vel')
        #     velSrv=rospy.ServiceProxy('offboard_controller/max_vel', MaxVel)
        #     req=MaxVelRequest()
        #     req.up_vel=3.0
        #     req.down_vel=2.0
        #     req.xy_vel=5.0
        #     resp=velSrv(req)
        #     if not resp.success:
        #         rospy.logerr("Could not set max velocities. Exitign tuning process")
        #         return
        #     else:
        #         rospy.loginfo("Calling offboard_controller/max_vel succeeded")
        # except rospy.ServiceException as e:
        #     rospy.logerr("Failed to call board_controller/max_vel: %s", e)
        #     return
        
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
        
        
def main():
    rospy.init_node("px4_tuner_node", anonymous=True)
    rospy.loginfo("Starting px4_tuner_node...\n")
    obj = PX4Tuner()
    # obj._record_data=True
    obj._debug=True

    rospy.spin()

if __name__ == "__main__":
    main()
