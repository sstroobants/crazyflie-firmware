import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats


avg_pearson_list = []
datasets = ["old_data", "new_data", "new_data_delay", "new_data_2x", "new_data_delay_2x", "new_data_delay_2x_not_trained", "full_net_old_data", "full_net_new_data_2x_delay"]
for dataset in datasets:
    data = pd.read_csv(f"tools/usdlog/{dataset}.csv")

    fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True, figsize=[6, 3])


    # set the title of the figure
    fig.suptitle(dataset)
    axs_id = 0

    (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).plot(ax=axes[axs_id], label="pid pitch")
    data["snn_control.torque_pitch"].shift(0).fillna(0).plot(ax=axes[axs_id], label="snn pitch")
    # set ylim
    axes[axs_id].set_xlim([1000, 4000])
    axes[axs_id].set_ylim([-18000, 18000])
    # set ylabel
    axes[axs_id].set_ylabel("pitch command")

    axes[axs_id].legend()

    axs_id += 1
    (data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]).plot(ax=axes[axs_id], label="pid roll")
    data["snn_control.torque_roll"].shift(0).fillna(0).plot(ax=axes[axs_id], label="snn roll")
    axes[axs_id].set_xlim([1000, 4000])
    axes[axs_id].set_ylim([-18000, 18000])
    axes[axs_id].legend()
    # set xlabel
    axes[axs_id].set_xlabel("timestep (0.002ms)")
    # set ylabel
    axes[axs_id].set_ylabel("roll command")

    plt.tight_layout()

    plt.savefig(f"tools/usdlog/{dataset}.pdf")

    # plt.show()

    pitch_list = []
    roll_list = []
    avg_pearson = []
    for i in range(0, -15, -1):
        # calculate correlation coeff for different shifted outputs
        # print(scipy.stats.pearsonr((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]), data["snn_control.torque_pitch"].shift(i).fillna(0)))
        # print(scipy.stats.pearsonr((data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]), data["snn_control.torque_roll"].shift(i).fillna(0)))
        # print("\n")
        pitch = scipy.stats.pearsonr((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]), data["snn_control.torque_pitch"].shift(i).fillna(0))
        roll = scipy.stats.pearsonr((data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]), data["snn_control.torque_roll"].shift(i).fillna(0))
        avg_prs = (pitch[0] + roll[0]) / 2
        pitch_list.append(pitch[0])
        roll_list.append(roll[0])
        avg_pearson.append(avg_prs)
    print(f"Avg max: {np.argmax(avg_pearson)}, pitch max: {np.argmax(pitch_list)}, roll max: {np.argmax(roll_list)}")
    avg_pearson_list.append(avg_pearson)

plt.figure(figsize=[4.3, 3.2])
for dataset, name in zip(avg_pearson_list, datasets):
    plt.plot(dataset, label=name)
# plt.plot(pitch_list, label="pitch")
# plt.plot(roll_list, label="roll")
plt.xlabel("shift [timesteps]")
plt.ylabel("pearson correlation")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig(f"tools/usdlog/pearson.pdf")
plt.show()