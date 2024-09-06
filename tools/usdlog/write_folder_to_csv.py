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
import os

parser = argparse.ArgumentParser()
parser.add_argument("--folder_name", type=str, default="tools/usdlog/05_09/att_dist/crimson_dew")
args = parser.parse_args()

# get all files in folder
for fold in os.listdir(args.folder_name):

    # decode binary log data
    if fold.endswith('.csv'):
        continue
    logData = cfusdlog.decode(f'{args.folder_name}/{fold}')

    #only focus on regular logging
    logData = logData['fixedFrequency']

    # let's see which keys exists in current data set
    keys = []
    for k, v in logData.items():
        keys.append(k)

    # first simply store in dataframe
    data = pd.DataFrame()
    for key in keys:
        data[key] = logData[key]
    data.to_csv(f'{args.folder_name}/{fold}.csv', index=False)
