# -*- coding: utf-8 -*-
"""
code to write usd logged crazyflie data to csv in format used for the snn pid
"""
import cfusdlog
import matplotlib.pyplot as plt
import re
import argparse
import pandas as pd
import numpy as np
from scipy import stats

parser = argparse.ArgumentParser()
parser.add_argument("--filename", type=str, default="tools/usdlog/log03")
args = parser.parse_args()

# decode binary log data
logData = cfusdlog.decode(args.filename)

#only focus on regular logging
logData = logData['fixedFrequency']

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1
plotRows = 1

# let's see which keys exists in current data set
keys = []
for k, v in logData.items():
    keys.append(k)

# first simply store in dataframe
data = pd.DataFrame()
for key in keys:
    data[key] = logData[key]

# determine start and end points and clip data
plt.subplot(1, 1, 1)
plt.plot(logData['pid_rate.pitch_outP'], '-', label='p')
plt.xlabel('index')
plt.ylabel('rate pid')
plt.legend(loc=9, ncol=4, borderaxespad=0.)
plt.show()
inStr = input("start/end-time (start,end): ")
start, end = map(int, inStr.split(','))
data = data[start:end]

# filter derivatives
roll_outD_ma = data["pid_rate.roll_outD"].rolling(4).median()
# roll_outD_outliers = (np.abs(stats.zscore(data["pid_rate.roll_outD"])) > 2.5)
roll_outD_outliers = (np.abs(data["pid_rate.roll_outD"] - roll_outD_ma) > 2000) 

ax1=plt.subplot(2, 1, 1); 
plt.plot(data["pid_rate.roll_outD"] - roll_outD_ma); 
plt.subplot(2, 1, 2, sharex=ax1); 
plt.plot(data["pid_rate.roll_outD"]); 
plt.plot(data["pid_rate.roll_outD"][roll_outD_outliers == True], 'bo'); 
data["pid_rate.roll_outD"][roll_outD_outliers] = roll_outD_ma[roll_outD_outliers]
pitch_outD_ma = data["pid_rate.pitch_outD"].rolling(4).median()
plt.plot(data["pid_rate.roll_outD"]); 
plt.show()
# pitch_outD_outliers = (np.abs(stats.zscore(data["pid_rate.pitch_outD"])) > 2.5)
pitch_outD_outliers = (np.abs(data["pid_rate.pitch_outD"] - pitch_outD_ma) > 2000) 
data["pid_rate.pitch_outD"][pitch_outD_outliers] = pitch_outD_ma[pitch_outD_outliers]
# yaw_outD_ma = data["pid_rate.yaw_outD"].rolling(4).median()
# yaw_outD_outliers = (np.abs(stats.zscore(data["pid_rate.yaw_outD"])) > 2.5)
# yaw_outD_outliers = (np.abs(data["pid_rate.yaw_outD"] - yaw_outD_ma) > 2000) 
# data["pid_rate.yaw_outD"][yaw_outD_outliers] = yaw_outD_ma[yaw_outD_outliers]

# write to .csv
data.to_csv(f'{args.filename}.csv', index=False)