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
parser.add_argument("--filename", type=str, default="tools/usdlog/log09")
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
plt.plot(logData['acc.z'], '-', label='acc z')
plt.xlabel('index')
plt.ylabel('thrust')
plt.legend(loc=9, ncol=4, borderaxespad=0.)
plt.show()

inStr = input("start/end-time (start,end): ")
try:
    start, end = map(int, inStr.split(','))
    data = data[start:end]
except ValueError:
    print("Invalid input, logging entire dataset")
    pass
data.to_csv(f'{args.filename}.csv', index=False)