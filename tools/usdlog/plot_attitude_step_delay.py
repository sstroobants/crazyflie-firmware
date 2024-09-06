import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import scipy.signal
import os


folders = ["tools/usdlog/27_08/att_dist/crimson_dew", "tools/usdlog/27_08/att_dist/lilac_wood", "tools/usdlog/27_08/att_dist/pid", "tools/usdlog/27_08/att_dist/desert_snow"]
fig = plt.figure(figsize=[3,3])
colors = ["blue", "red", "green", "orange"]
color_idx = 0
for folder in folders:
    avg_pearson_list = []
    for dataset in os.listdir(folder):
        # only use .csv files
        if not dataset.endswith(".csv"):
            continue
        data = pd.read_csv(f"{folder}/{dataset}")
        pitch_list = []
        roll_list = []
        avg_pearson = []
        pearson_first = None
        for i in range(0, -13, -1):
            # calculate correlation coeff for different shifted outputs
            pitch = scipy.stats.pearsonr((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]), data["snn_control.torque_pitch"].shift(i).fillna(0))
            roll = scipy.stats.pearsonr((data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]), data["snn_control.torque_roll"].shift(i).fillna(0))
            avg_prs = (pitch[0] + roll[0]) / 2
            pitch_list.append(pitch[0])
            roll_list.append(roll[0])

            avg_pearson.append(np.array(avg_prs))
            # print(avg_prs)
        print(f"[{folder}/{dataset}] Avg max: {np.argmax(avg_pearson)}, pitch max: {np.argmax(pitch_list)}, roll max: {np.argmax(roll_list)}")
        avg_pearson_list.append(avg_pearson)




    # fig = plt.figure(figsize=[3,3])
    for dataset, name in zip(avg_pearson_list, [x for x in os.listdir(folder) if x.endswith(".csv")]):
        dataset = dataset #- np.max(dataset)
        # plt_color = "blue" if folder == folders[0] else "red"
        plt.plot(dataset, label=f"{folder.split("/")[-1]} {name}", color=colors[color_idx])
    color_idx += 1
plt.ylabel("pearson correlation")
plt.xlabel("shift [timesteps]")
# plt.legend()
plt.grid()
plt.tight_layout()
plt.show()