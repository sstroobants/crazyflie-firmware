import pandas as pd
import matplotlib.pyplot as plt
import os


folders = ["tools/usdlog/05_09/att_dist/lilac_wood", "tools/usdlog/05_09/att_dist/crimson_dew", "tools/usdlog/05_09/att_dist/desert_snow", "tools/usdlog/05_09/att_dist/pid"]
colors = ["blue", "red", "green", "orange"]
names = ["trained with delay and augmentation", "trained without delay", "trained without augmentation", "pid"]
color_idx = 0
legend_loc = "upper right"
title_fontsize = 10
fig, axes = plt.subplots(nrows=4, ncols=1, sharex=True, figsize=[10, 10])
axs_id = 0
t_roll_command = None
for folder in folders:
    avg_response_roll = []
    avg_response_pitch = []
    for dataset in os.listdir(folder):
        # only use .csv files
        if not dataset.endswith(".csv"):
            continue
        data = pd.read_csv(f"{folder}/{dataset}")

        this_t_roll_command = data["controller.roll"].idxmin()
        if t_roll_command is None:
            # find timestep of first roll command below 0
            t_roll_command = data["controller.roll"].idxmin()
        
        # shift entire dataset to have the first roll command at the same time
        data.index = data.index - this_t_roll_command
        data.index = data.index + t_roll_command

        # axs_id = 0
        data["stateEstimate.roll"].plot(ax=axes[axs_id], label="roll", color=colors[color_idx], alpha=0.2)
        data["controller.roll"].plot(ax=axes[axs_id], label="roll command", color="grey")
        # data["pid_rate.roll_output"].plot(ax=axes[axs_id], label="roll", color=colors[color_idx], alpha=0.2)
        # data["snn_control.torque_roll"].plot(ax=axes[axs_id], label="roll command", color="grey")
        axes[axs_id].axhline(0, 0, len(data["stateEstimate.roll"]), color='k')
        axes[axs_id].set_title(f"{names[color_idx]}", fontsize=title_fontsize)
        axes[axs_id].set_ylim([-18, 18])
        axes[axs_id].set_xlabel("timestep (0.002ms)")
  
        avg_response_roll.append(data["stateEstimate.roll"].values)
        avg_response_pitch.append(data["stateEstimate.pitch"].values)

        mse = ((data["stateEstimate.roll"] - data["controller.roll"])**2).mean()
        print(f"{folder.split('/')[-1]} {dataset}: {mse}")
    avg_response_roll = pd.DataFrame(avg_response_roll).mean()
    avg_response_pitch = pd.DataFrame(avg_response_pitch).mean()

    
    # plot average response
    # axs_id = 0
    # avg_response_roll.plot(ax=axes[axs_id], label=f"{folder.split('/')[-1]} roll", color=colors[color_idx])
    # axs_id += 1
    # avg_response_pitch.plot(ax=axes[axs_id], label=f"{folder.split('/')[-1]} pitch", color=colors[color_idx])
    axs_id += 1
    color_idx += 1
plt.tight_layout()
plt.show()