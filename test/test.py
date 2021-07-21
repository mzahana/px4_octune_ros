#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def alignData():
    # Time
    t1=np.arange(start=0.01, stop=4.0, step=0.1)
    t2=np.arange(start=1.0, stop=4.5, step=0.05)
    # Data
    data1=np.sin(t1)
    data2=np.cos(t2)

    # s: seconds
    idx1=pd.to_timedelta(t1, unit='s')
    idx2=pd.to_timedelta(t2, unit='s')

    df1 = pd.DataFrame(data1, index =idx1, columns =['Data1'])
    df2 = pd.DataFrame(data2, index =idx2, columns =['Data2'])

    # df_merged = df1.append(df2, sort=True)
    df_merged =df1.merge(df2, how='outer', left_index=True, right_index=True)
    #df_merged = pd.concat([df1, df2]).sort_index()
    #df_merged = pd.merge_asof(df1,df2,left_index=True, right_index=True, direction='forward')

    # Find the common index range between the two dataframes


    df_resampled = df_merged.resample('1L').mean().interpolate()
    min_idx=None
    max_idx=None
    if (df1.index[0] > df2.index[0]):
        min_idx=df1.index[0]
    else:
        min_idx=df2.index[0]

    if (df1.index[-1] < df2.index[-1]):
        max_idx=df1.index[-1]
    else:
        max_idx=df2.index[-1]

    df_common=df_resampled[min_idx:max_idx]

    print("----------Before resampling ----------")
    # print(df1)
    print("Size of df1={}".format(len(df1)))
    # print(df2)
    print("Size of df2={}".format(len(df2)))
    # print('----- Merged data -----')
    #print(df_merged)
    print("Size of df_merged={}".format(len(df_merged)))

    print('--------- After resmapling -----------')
    #print(df_resampled)
    print("Size of df_resampled={}".format(len(df_resampled)))

    df_has_nan=df_resampled.isnull().values.any()
    if (df_has_nan):
        print('df_resampeld has NaN values')
    else:
        print('df_resampeld does not have NaN values')


    print("Size of df_common= {}".format(len(df_common)))
    if (df_common.isnull().values.any()):
        print('df_common has NaN values')
    else:
        print('df_common does not have NaN values')

    #print(df_resampled[min_idx:max_idx])

    # Plot
    df1.plot(title='Original Data1')
    df2.plot(title='Original Data2')
    df_common.plot(title='Merged & Resampled & common-range extracted data')
    plt.show()
    
    

def main():
    alignData()
if __name__=="__main__":
    main()