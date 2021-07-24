#!/usr/bin/env python

from time import time
import rospy
from mavros_msgs.msg import AttitudeTarget, ActuatorControl, PositionTarget
from mavros_msgs.srv import MessageInterval, MessageIntervalRequest, MessageIntervalResponse
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
from std_srvs.srv import Empty, Trigger, TriggerRequest
from math import ceil
from octune.optimization import BackProbOptimizer
import pandas as pd
import matplotlib.pyplot as plt
import tf
import numpy as np

class PX4Tuner:
    def __init__(self):
        # Print debug messages
        self._debug = rospy.get_param('~debug', False)

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
        self._excitation_t = rospy.get_param('~excitation_t', 0.5)

        # Sampling/upsampling time, seconds
        self._sampling_dt = rospy.get_param('~sampling_dt', 0.001)

        # Tuning altitude
        self._tuning_alt=rospy.get_param('~tuning_alt', 5.0)

        # Current drone local position
        self._current_drone_pos=Point()

        # Maximum array length, maybe dependent on excitiation time and sampling time?
        self._max_arr_L=int(ceil(self._excitation_t/self._sampling_dt))

        # Flag to record/store data
        self._record_data=False

        # Setpoint msg
        self._setpoint_msg=Point()
        # Velocity setpoint msg, only XY, the Z component is altitude
        self._vel_sp_msg=Vector3()

        # Setpoint publisher
        self._setpoint_pub = rospy.Publisher('offboard_controller/setpoint/local_pos', Point, queue_size=10)

        # Velocity setpoint publisher
        self._vel_sp_pub = rospy.Publisher('offboard_controller/setpoint/body_xy_vel', Vector3, queue_size=10)
        self._local_vel_sp_pub = rospy.Publisher('offboard_controller/setpoint/local_xy_vel', Vector3, queue_size=10)

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

        # Service for testing data pre-process
        rospy.Service('px4_octune/process_data', Empty, self.prepDataSrv)

        # Request higher data streams
        ret = self.increaseStreamRates()
        if not ret:
            rospy.logerr("Could not request higher stream rate. Shutting down px4_tuner_node ...")
            exit()

     # ---------------------------- Callbacks ----------------------------#

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
            if self._debug:
                rospy.loginfo_throttle(1, "_cmd_att_dict length is: %s",len(self._cmd_att_dict['time']))

        # Commanded angular rates
        x = msg.body_rate.x # commanded roll rate
        y = msg.body_rate.y # commanded pitch rate
        z = msg.body_rate.z # commanded yaw rate

        if self._record_data:
            self._cmd_att_rate_dict = self.insertData(dict=self._cmd_att_rate_dict, t=t_micro, x=x, y=y, z=z)
            if self._debug:
                rospy.loginfo_throttle(1, "cmd_att_rate_dict length is: %s",len(self._cmd_att_rate_dict['time']))


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
            if self._debug:
                rospy.loginfo_throttle(1, "_ang_rate_cnt_output_dict length is: %s",len(self._ang_rate_cnt_output_dict['time']))

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
            if self._debug:
                rospy.loginfo_throttle(1, "att_rate_dict length is: %s",len(self._att_rate_dict['time']))

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
            if self._debug:
                rospy.loginfo_throttle(1, "att_dict length is: %s",len(self._att_dict['time']))
    
    def poseCb(self, msg):
        """Pose (feedback) callback
        """
        if msg is None:
            return
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
        except rospy.ServiceException, e:
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
                    
            except rospy.ServiceException, e:
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
            
    def prepDataSrv(self, req):
        """Data pre-processing service (for testing) 
        """
        if(self._debug):
            rospy.loginfo("Stopping data recording")
        self.stopsRecordingData()

        if(self._debug):
            rospy.loginfo("Resetting data dictionaries")
        self.resetDict()

        if(self._debug):
            rospy.loginfo("Starting data recording")
        self.startRecordingData()

        # Set home point (to go to after tuning)
        try:
            rospy.wait_for_service('offboard_controller/use_current_pos', timeout=2.0)
            homeSrv=rospy.ServiceProxy('offboard_controller/use_current_pos', Trigger)
            homeSrv()
        except rospy.ServiceException, e:
            print("service offboard_controller/use_current_pos call failed: {}. offboard_controller/use_current_pos Mode could not be set.".format(e))

        # 5m/s velocity setpoint in x-axis
        if (self._debug):
            rospy.loginfo("Sending velocity setpoint: 5 m/s in body x-axis")
        self._vel_sp_msg.x=-5.0 # m/s
        self._vel_sp_msg.y=-5.0
        self._vel_sp_msg.z = self._current_drone_pos.z #self._tuning_alt # this is altitude in meter(s)
        self._local_vel_sp_pub.publish(self._vel_sp_msg)

        # Activate offboard mode
        try:
            rospy.wait_for_service('offboard_controller/arm_and_offboard', timeout=2.0)
            offbSrv = rospy.ServiceProxy('offboard_controller/arm_and_offboard', Empty)
            offbSrv()
        except rospy.ServiceException, e:
            print("service offboard_controller/arm_and_offboard call failed: {}. arm_and_offboard Mode could not be set.".format(e))

        if(self._debug):
            rospy.loginfo("Waiting for %s seconds", self._excitation_t/2)
        rospy.sleep(rospy.Duration(secs=self._excitation_t/2))

        # -5m/s velocity setpoint in x-axis
        if (self._debug):
            rospy.loginfo("Sending velocity setpoint: -5 m/s in body x-axis")
        self._vel_sp_msg.x=5.0 # m/s
        self._vel_sp_msg.y=5.0
        self._vel_sp_msg.z = self._current_drone_pos.z #self._tuning_alt # this is altitude in meter(s)
        self._local_vel_sp_pub.publish(self._vel_sp_msg)

        if(self._debug):
            rospy.loginfo("Waiting for %s seconds", self._excitation_t/2)
        rospy.sleep(rospy.Duration(secs=self._excitation_t/2))

        # Hold position
        try:
            rospy.wait_for_service('offboard_controller/set_local_tracking', timeout=2.0)
            holdSrv = rospy.ServiceProxy('offboard_controller/set_local_tracking', Trigger)
            resp=holdSrv()
        except rospy.ServiceException, e:
            print("service offboard_controller/set_local_tracking call failed: {}. offboard_controller/set_local_tracking Mode could not be set.".format(e))

        if(self._debug):
            rospy.loginfo("Stopping data recording")
        self.stopsRecordingData()

        if(self._debug):
            rospy.loginfo("Processing Roll Rate data")
        roll_rate_cmd, roll_rate = self.prepRollRate()

        if (roll_rate_cmd is not None and roll_rate is not None):
            rospy.loginfo("Roll rate data is processed")
        else:
            rospy.logwarn("Roll rate data processing failed")

        if(self._debug):
            rospy.loginfo("Processing Pitch Rate data")
        pitch_rate_cmd, pitch_rate = self.prepPitchRate()

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

        if (self._debug):
            rospy.loginfo("Angular rates PID output data")
        pid_roll, pid_pitch, pid_yaw=self.prepRatesPIDOutput()

        if (pid_roll is not None):
            rospy.loginfo("Angular rates PID output data is processed")
        else:
            rospy.logwarn("Angular rates PID output data is None")

        
        if self._debug:
            plt.show()

        return []

    def apllyStepInput(self):
        """Sends local velocity/position step inputs to excite the system.
        Requires mavros_offboard_controller.py node
        """
        # TODO implement step inputs; requires mavros_offboard_controller.py node

    def startRecordingData(self):
        """Sets _record_data=True so that the callbacks starts storing data in the dictionaries
        """
        self._record_data=True

    def stopsRecordingData(self):
        """Sets _record_data=False so that the callbacks stop storing data in the dictionaries
        """
        self._record_data=False

    def preProcessData(self):
        """Organizes data, aligns their timesamps, and upsamples them, in preparation for optimization
        """
        # TODO Implement data pre-processing
        pass

    def tuneRatePID(self):
        """Tunes the angular rates PID loops
        """
        # TODO 
        # reset data dictionaries
        # get current PID gains
        # apply step inputs
        # record data
        # optimize
        # Check new PID gains
        # if gains are in acceptable range, send new gains to FCU
        # repeat until performance is acceptable (what is the metric?)
        pass

    def tuneVelPIDs(self):
        """Tunes linear velocity PID loops
        """
        # TODO 
        pass

    def prepData(self, cmd_df=None, fb_df=None, cmd_data_label=None, data_label=None):
        """Merges data (target and actual), resample, and align their timestamps.

        Parameters
        --
        @param cmd_df Pandas Dataframe of commanded data
        @param fb_df Pandas Dataframe of feedback data

        Returns
        --
        @return prep_cmd_data Prepared commanded data as a Python list
        @return prep_fb_data Prepared feedback data as a Python list
        """
        prep_cmd_data=None
        prep_data=None
        if cmd_df is None:
            rospy.logerr("Commanded dataframe is None")
            return prep_cmd_data,prep_data
        if fb_df is None:
            rospy.logerr("Feedback dataframe is None")
            return prep_cmd_data,prep_data
        if cmd_data_label is None:
            rospy.logerr("Commanded data label is None")
            return prep_cmd_data,prep_data
        if data_label is None:
            rospy.logerr("Feedback data label is None")
            return prep_cmd_data,prep_data

        # Merge
        df_merged =cmd_df.merge(fb_df, how='outer', left_index=True, right_index=True)

        # Resample
        dt_str=str(self._sampling_dt*1000.0)+'L' # L is for milli-second in Pandas
        df_resampled = df_merged.resample(dt_str).mean().interpolate()

        # Find common time range
        min_idx=None
        max_idx=None
        if (cmd_df.index[0] > fb_df.index[0]):
            min_idx=cmd_df.index[0]
        else:
            min_idx=fb_df.index[0]

        if (cmd_df.index[-1] < fb_df.index[-1]):
            max_idx=cmd_df.index[-1]
        else:
            max_idx=fb_df.index[-1]

        df_common=df_resampled[min_idx:max_idx]

        if (self._debug):
            rospy.loginfo("Total time for processed data = %s seocond(s)\n", (max_idx-min_idx).total_seconds())
            df_common.plot()
            #plt.show()

        # Finally extract processed data as lists
        prep_cmd_data =  df_common[cmd_data_label].tolist()
        prep_data = df_common[data_label].tolist()

        return prep_cmd_data, prep_data


    def prepRatesPIDOutput(self):
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
        """Merges roll rate data (target and actual), resample, and align their timestamps.
        Uses self._cmd_att_rate_dict and self._att_rate_dict

        Returns
        --
        @return prep_roll_rate_cmd Numpy array
        @return prep_roll_rate Numpy array
        """
        prep_roll_rate_cmd=None
        prep_roll_rate=None
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

        prep_roll_rate_cmd, prep_roll_rate=self.prepData(cmd_df=cmd_roll_rate, fb_df=fb_roll_rate, cmd_data_label='cmd_roll_rate', data_label='roll_rate')

        return prep_roll_rate_cmd, prep_roll_rate

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

    def alignData(self):
        t1=np.arange(10)+100.0
        t2=np.arange(5)+101.0
        d1=pd.to_timedelta(t1, unit='s')
        d2=pd.to_timedelta(t2, unit='s')
        print("--------------------")
        print(type(d1))
        
        
def main():
    rospy.init_node("px4_tuner_node", anonymous=True)
    rospy.loginfo("Starting px4_tuner_node...\n")
    obj = PX4Tuner()
    # obj._record_data=True
    obj._debug=True

    rospy.spin()

if __name__ == "__main__":
    main()
