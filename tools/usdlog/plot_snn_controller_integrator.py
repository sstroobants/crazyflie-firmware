import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats


avg_pearson_list = []
dataset = "log12"
data = pd.read_csv(f"tools/usdlog/{dataset}.csv")

fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)

axs_id = 0
# data["stateEstimate.roll"].plot(ax=axes[axs_id], label="roll")
# data["controller.roll"].plot(ax=axes[axs_id], label="roll command")
# axes[axs_id].axhline(0, 0, len(data["stateEstimate.roll"]), color='k')
# axes[axs_id].legend()
# axes[axs_id].set_title("roll command")
# axes[axs_id].set_xlabel("timestep (0.002ms)")
# axs_id += 1

# data["stateEstimate.pitch"].plot(ax=axes[axs_id], label="pitch")
# data["controller.pitch"].plot(ax=axes[axs_id], label="pitch command")
# axes[axs_id].axhline(0, 0, len(data["stateEstimate.pitch"]), color='k')
# # axes[axs_id].set_ylim([-35000, 35000])
# axes[axs_id].legend()
# axes[axs_id].set_title("pitch command")
# axes[axs_id].set_xlabel("timestep (0.002ms)")
# axs_id += 1

# # (data["pid_attitude.pitch_output"] - data["pid_attitude.pitch_outI"]).plot(ax=axes[axs_id], label="pid")
# data["pid_attitude.pitch_output"].plot(ax=axes[axs_id], label="pid")
# # data["snn_control.pitch_input"].plot(ax=axes[axs_id], label="snn input")
# (-data["gyro.y"]).plot(ax=axes[axs_id], label="gyroy")
# axes[axs_id].axhline(0, 0, len(data["pid_attitude.pitch_output"]), color='k')
# axes[axs_id].legend()
# axes[axs_id].set_ylim([-100, 100])
# axes[axs_id].set_title("Pitch rate")
# axs_id += 1

# (data["pid_attitude.roll_output"] - data["pid_attitude.roll_outI"]).plot(ax=axes[axs_id], label="pid")
data["pid_attitude.roll_output"].plot(ax=axes[axs_id], label="pid")
# data["snn_control.roll_input"].plot(ax=axes[axs_id], label="snn input")
data["gyro.x"].plot(ax=axes[axs_id], label="gyrox")
axes[axs_id].axhline(0, 0, len(data["pid_attitude.roll_output"]), color='k')
axes[axs_id].legend()
axes[axs_id].set_ylim([-100, 100])
axes[axs_id].set_title("Roll rate")
axes[axs_id].set_xlabel("timestep (0.002ms)")
axs_id += 1

# axes[axs_id].set_title("Roll torque command")
# # (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).plot(ax=axes[axs_id], label="pid pitch")
# (data["pid_rate.pitch_output"]).plot(ax=axes[axs_id], label="pid pitch")
# # data["snn_control.torque_pitch"].shift(0).fillna(0).plot(ax=axes[axs_id], label="snn pitch")
# (data["pid_rate.pitch_outI"] - data["pid_rate.pitch_outI"][0]).plot(ax=axes[axs_id], label="pid pitch")
# calc_integ = np.zeros_like(data["pid_rate.roll_outI"])
# for i in range(1, len(data["pid_rate.roll_outI"])):
#     calc_integ[i] = calc_integ[i-1] + ((data["controller.pitch"][i] - data["stateEstimate.pitch"][i]) * 6 + data["gyro.y"][i]) * 0.5
#     if calc_integ[i] > 6800:
#         calc_integ[i] = 6800
#     elif calc_integ[i] < -10000:
#         calc_integ[i] = -10000
# axes[axs_id].plot(calc_integ, label="integ pitch calculated")
# # data["snn_control.pitch_integ"].plot(ax=axes[axs_id], label="snn integ")
# # data["pid_rate.roll_output"].plot(ax=axes[axs_id], label="pid roll")
# axes[axs_id].set_ylim([-20000, 20000])
# axes[axs_id].legend()
# axs_id += 1

axes[axs_id].set_title("Pitch torque command")
(data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]).plot(ax=axes[axs_id], label="pid roll")
(data["pid_rate.roll_output"]).plot(ax=axes[axs_id], label="pid roll")
(data["gyro.x"].diff() * 3000).shift(-25).plot(ax=axes[axs_id], label="gyrox diff")
# plot horizontal line at 0
axes[axs_id].axhline(0, 0, len(data["pid_rate.roll_output"]), color='k')
# data["snn_control.torque_roll"].shift(0).fillna(0).plot(ax=axes[axs_id], label="snn roll")
# data["pid_rate.pitch_outI"].plot(ax=axes[axs_id], label="pid pitch")
(data["pid_rate.roll_outI"] - data["pid_rate.roll_outI"][0]).plot(ax=axes[axs_id], label="integ roll")
calc_integ = np.zeros_like(data["pid_rate.roll_outI"])
for i in range(1, len(data["pid_rate.roll_outI"])):
    calc_integ[i] = calc_integ[i-1] + ((data["controller.roll"][i] - data["stateEstimate.roll"][i]) * 6 - data["gyro.x"][i]) * 0.5
    if calc_integ[i] > 12230:
        calc_integ[i] = 12230
    elif calc_integ[i] < -4330:
        calc_integ[i] = -4330
# (np.cumsum((data["controller.roll"] - data["stateEstimate.roll"]) * 6 - data["gyro.x"]) * 0.5).plot(ax=axes[axs_id], label="integ roll calculated")
axes[axs_id].plot(calc_integ, label="integ roll calculated")
# data["snn_control.roll_integ"].plot(ax=axes[axs_id], label="snn integ")
axes[axs_id].set_ylim([-20000, 20000])
axes[axs_id].legend()

plt.show()

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

plt.figure(figsize=[4, 3])
for dataset, name in zip(avg_pearson_list, ["old_data", "new_data", "new_data_delay", "new_data_2x"]):
    plt.plot(dataset, label=name)
# plt.plot(pitch_list, label="pitch")
# plt.plot(roll_list, label="roll")
plt.xlabel("shift [timesteps]")
plt.ylabel("pearson correlation")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()