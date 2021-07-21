#!/usr/bin/env python

import rospy
from mavros_msgs.msg import AttitudeTarget, ActuatorControl, PositionTarget
from mavros_msgs.srv import MessageInterval, MessageIntervalRequest, MessageIntervalResponse
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from math import ceil
from octune.optimization import BackProbOptimizer
import pandas as pd

class PX4Tuner:
    def __init__(self):
        # Print debug messages
        self._debug = rospy.get_param('~debug', False)

        # Commanded attitude rates, dictionary, has two arrays, timesatmp and commanded attitude rates
        self._cmd_att_rate_dict=        {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded attitude rates, dictionary, has two arrays, timesatmp and commanded attitude
        self._cmd_att_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded velocity, dictionary, has two arrays, timesatmp and commanded velocity
        self._cmd_vel_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        # Commanded position, dictionary, has two arrays, timesatmp and commanded position
        self._cmd_pos_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}

        # Feedback, attitude rates, dictionary, two arrays, time & data
        self._att_rate_dict=           {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, attitude,  dictionary, two arrays, time & data
        self._att_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, local velocity,  dictionary, two arrays, time & data
        self._vel_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        # Feedback, local position,  dictionary, two arrays, time & data
        self._pos_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}

        # System excitation time, seconds
        self._excitation_t = rospy.get_param('~excitation_t', 2.0)

        # Sampling/upsampling time, seconds
        self._sampling_dt = rospy.get_param('~sampling_dt', 0.01)

        # Maximum array length, maybe dependent on excitiation time and sampling time?
        self._max_arr_L=int(ceil(self._excitation_t/self._sampling_dt))

        # Flag to record/store data
        self._record_data=False



        # Commanded attitude
        rospy.Subscriber("mavros/setpoint_raw/target_attitude",AttitudeTarget, self.cmdAttCb)
        # Commanded angular rates
        rospy.Subscriber("mavros/target_actuator_control",ActuatorControl, self.cmdRatesCb)
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

        # Request higher data streams
        ret = self.increaseStreamRates()
        if not ret:
            rospy.logerr("Could not request higher stream rate. Exiting...")
            exit()

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
        rospy.wait_for_service('mavros/set_message_interval')
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

    def resetDict(self):
        """Clears all data dictionaries
        """
        self._cmd_att_rate_dict=        {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_att_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_vel_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._cmd_pos_dict=             {'time':[], 'x':[], 'y':[], 'z':[]}
        self._att_rate_dict=            {'time':[], 'x':[], 'y':[], 'z':[]}
        self._att_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        self._vel_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}
        self._pos_dict=                 {'time':[], 'x':[], 'y':[], 'z':[]}

    
    def cmdAttCb(self, msg):
        if msg is None:
            return
        
        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000
            
        

    def cmdRatesCb(self, msg):
        if msg is None:
            return

        t = rospy.Time(secs=msg.header.stamp.secs, nsecs=msg.header.stamp.nsecs)
        t_micro = t.to_nsec()/1000
        x = msg.controls[0] # commanded roll rate
        y = msg.controls[1] # commanded pitch rate
        z = msg.controls[2] # commanded yaw rate

        if self._record_data:
            self._cmd_att_rate_dict = self.insertData(dict=self._cmd_att_rate_dict, t=t_micro, x=x, y=y, z=z)
            if self._debug:
                rospy.loginfo_throttle(1, "cmd_att_rate_dict length is: %s",len(self._cmd_att_rate_dict['time']))

    def cmdPosCb(self, msg):
        if msg is None:
            return
        # TODO

    def rawImuCb(self, msg):
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
        if msg is None:
            return
        # TODO
    
    def poseCb(self, msg):
        if msg is None:
            return
        # TODO

    def velCb(self, msg):
        if msg is None:
            return
        # TODO

    def apllyStepInput(self):
        """Sends local velocity step inputs to excite the system
        """
        # TODO implement step inputs; required mavros offboard controller

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
        
        # TODO Verify the following implementation in simulation

        # setup the time index list
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_roll_rate = pd.DataFrame(self._cmd_att_rate_dict['x'], index=cmd_idx, columns =['cmd_roll_rate'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_rate_dict['time'], unit='us') # 'us' = micro seconds
        fb_roll_rate = pd.DataFrame(self._att_rate_dict['x'], index=fb_idx, columns =['roll_rate'])

        # Merge
        df_merged =cmd_roll_rate.merge(fb_roll_rate, how='outer', left_index=True, right_index=True)

        # Resample
        dt_str=str(self._sampling_dt*1000.0)+'L' # L is for milli-second in Pnadas
        df_resampled = df_merged.resample(dt_str).mean().interpolate()

        # Find common time range
        min_idx=None
        max_idx=None
        if (cmd_roll_rate.index[0] > fb_roll_rate.index[0]):
            min_idx=cmd_roll_rate.index[0]
        else:
            min_idx=fb_roll_rate.index[0]

        if (cmd_roll_rate.index[-1] < fb_roll_rate.index[-1]):
            max_idx=cmd_roll_rate.index[-1]
        else:
            max_idx=fb_roll_rate.index[-1]

        df_common=df_resampled[min_idx:max_idx]

        # Finally extract processed data as lists
        prep_roll_rate_cmd = df_common['cmd_roll_rate'].tolist()
        prep_roll_rate = df_common['roll_rate'].tolist()

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
    obj._record_data=True
    obj._debug=True

    rospy.spin()

if __name__ == "__main__":
    main()
