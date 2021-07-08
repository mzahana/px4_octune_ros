#!/usr/bin/env python

import numpy as np
import pandas as pd


def alignData():
    # Time
    t1=np.arange(10)+100.0
    t2=np.arange(5)+101.0
    # Data
    data1=np.sin(t1)
    data2=np.sin(t2)

    idx1=pd.to_timedelta(t1, unit='s')
    idx2=pd.to_timedelta(t2, unit='s')

    df1 = pd.DataFrame(data1, index =idx1, columns =['Data1'])
    df2 = pd.DataFrame(data2, index =idx2, columns =['Data2'])

    print("--------------------")
    print(df1)

def main():
    alignData()
if __name__=="__main__":
    main()