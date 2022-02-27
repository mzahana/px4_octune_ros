from math import ceil
import pandas as pd
import matplotlib.pyplot as plt
import logging
logging.basicConfig(level=logging.DEBUG)

class ProcessData:
    def __init__(self):
        
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

        # Processed Roll rate data
        self._prep_roll_rate_cmd=None
        self._prep_roll_rate=None
        self._prep_roll_cnt_output=None

        # Processed Pitch rate data
        self._prep_pitch_rate_cmd=None
        self._prep_pitch_rate=None
        self._prep_pitch_cnt_output=None

        # Sampling/upsampling time, seconds
        self._sampling_dt = 0.005
        
        # Expected raw data frequency samples/sec
        self._raw_data_freq = 50.0 

        # Max data length in seconds
        self._data_len_sec = 2.0

        # Flag to start/stop filling data arrays
        self._record_data=False

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
            logging.error("[insertData] One of the input arguments is None")
            return
        # TODO Make sure that the input dict is with the right data structure, time & data arrays

        max_arr_L = int(self._data_len_sec / self._sampling_dt)

        out_dict=dict
        out_dict['time'].append(t)
        out_dict['x'].append(x)
        out_dict['y'].append(y)
        out_dict['z'].append(z)
        if len(out_dict['time']) > max_arr_L:
            out_dict['time'].pop(0)
        if len(out_dict['x']) > max_arr_L:
            out_dict['x'].pop(0)
        if len(out_dict['y']) > max_arr_L:
            out_dict['y'].pop(0)
        if len(out_dict['z']) > max_arr_L:
            out_dict['z'].pop(0)

        return out_dict

    def gotEnoughRateData(self, t=1.0):
        # 50 is the expected number of samples/sec
        cmd_att_rate = len(self._cmd_att_rate_dict['time']) > int(ceil(t*self._raw_data_freq))
        att_rate = len(self._att_rate_dict['time']) > int(ceil(t*self._raw_data_freq))
        att_cnt = len(self._ang_rate_cnt_output_dict['time']) > int(ceil(t*self._raw_data_freq))
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
            logging.error("Commanded dataframe is None")
            return None
        if fb_df is None:
            logging.error("Feedback dataframe is None")
            return None
        if cmd_data_label is None:
            logging.error("Commanded data label is None")
            return None
        if data_label is None:
            logging.error("Feedback data label is None")
            return None
        if cnt_df is None:
            logging.error("Control data is None")
            return None
        if cnt_label is None:
            logging.error("Control data label is None")
            return None

        
        logging.debug("[prepData] Length of {}={}, length of {}={}, length of {}={}".format( cmd_data_label,len(cmd_df),data_label, len(fb_df), cnt_label,len(cnt_df)))

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
            logging.info("Total time for processed data = {} seocond(s)\n".format( (max_idx-min_idx).total_seconds()))
            # df_common.plot()
            #plt.show()

        # Finally extract processed data as lists
        prep_cmd_data =  df_common[cmd_data_label].tolist()
        prep_data = df_common[data_label].tolist()
        prep_cnt_data = df_common[cnt_label].tolist()        
            
        return (prep_cmd_data, prep_data, prep_cnt_data)

    def prepRollRate(self):
        """Merges roll rate data (target and actual) and the corresponding controller output signal, resample, and align their timestamps.
        Uses self._cmd_att_rate_dict and self._att_rate_dict

        Returns
        --
        @return prep_roll_rate_cmd Numpy array
        @return prep_roll_rate Numpy array
        """
        
        # Sanity check
        if len(self._cmd_att_rate_dict['time']) < 1:
            logging.error("[prepRollRate] No commanded attitude rate data to process")
            return None

        if len(self._att_rate_dict['time']) < 1:
            logging.error("[prepRollRate] No feedback attitude rate data to process")
            return None
        
        

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

        self._prep_roll_rate_cmd, self._prep_roll_rate, self._prep_roll_cnt_output=self.prepData(cmd_df=cmd_roll_rate, fb_df=fb_roll_rate,cnt_df=cnt_roll_rate, cmd_data_label='cmd_roll_rate', data_label='roll_rate', cnt_label='roll_cnt')
        processed_data = (self._prep_roll_rate_cmd, self._prep_roll_rate, self._prep_roll_cnt_output)

        return processed_data

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
            logging.error("[prepPitchRate] No commanded attitude rate data to process")
            return None

        if len(self._att_rate_dict['time']) < 1:
            logging.error("[prepPitchRate] No feedback attitude rate data to process")
            return None
        
        # TODO implement data pre-processing
        # setup the time index list
        #command
        cmd_idx=pd.to_timedelta(self._cmd_att_rate_dict['time'], unit='us') # 'us' = micro seconds
        cmd_pitch_rate = pd.DataFrame(self._cmd_att_rate_dict['y'], index=cmd_idx, columns =['cmd_pitch_rate'])
        # feedback
        fb_idx=pd.to_timedelta(self._att_rate_dict['time'], unit='us') # 'us' = micro seconds
        fb_pitch_rate = pd.DataFrame(self._att_rate_dict['y'], index=fb_idx, columns =['pitch_rate'])
        # controller output
        cnt_idx=pd.to_timedelta(self._ang_rate_cnt_output_dict['time'], unit='us') # 'us' = micro seconds
        cnt_pitch_rate = pd.DataFrame(self._ang_rate_cnt_output_dict['y'], index=cnt_idx, columns =['pitch_cnt'])

        self._prep_pitch_rate_cmd, self._prep_pitch_rate, self._prep_pitch_cnt_output=self.prepData(cmd_df=cmd_pitch_rate, fb_df=fb_pitch_rate,cnt_df=cnt_pitch_rate, cmd_data_label='cmd_pitch_rate', data_label='pitch_rate', cnt_label='pitch_cnt')
        processed_data = (self._prep_pitch_rate_cmd, self._prep_pitch_rate, self._prep_pitch_cnt_output)

        return processed_data

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
            logging.error("[prepYawRate] No commanded attitude rate data to process")
            return prep_yaw_rate_cmd, prep_yaw_rate

        if len(self._att_rate_dict['time']) < 1:
            logging.error("[prepYawRate] No feedback attitude rate data to process")
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
            logging.error("[prepRoll] No commanded attitude data to process")
            return prep_roll_cmd, prep_roll

        if len(self._att_dict['time']) < 1:
            logging.error("[prepRoll] No feedback attitude data to process")
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
            logging.error("[prepPitch] No commanded attitude data to process")
            return prep_pitch_cmd, prep_pitch

        if len(self._att_dict['time']) < 1:
            logging.error("[prepPitch] No feedback attitude data to process")
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

    def startRecordingData(self):
        """Sets _record_data=True so that the callbacks starts storing data in the dictionaries
        """
        self._record_data=True

    def stopRecordingData(self):
        """Sets _record_data=False so that the callbacks stop storing data in the dictionaries
        """
        self._record_data=False
