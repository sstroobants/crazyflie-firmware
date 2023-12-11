import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("tools/usdlog/log00.csv")

fig, axes = plt.subplots(nrows=6, ncols=1, sharex=True)

axs_id = 0
data["stateEstimate.roll"].plot(ax=axes[axs_id], label="roll")
data["controller.roll"].plot(ax=axes[axs_id], label="roll command")
axes[axs_id].axhline(0, 0, len(data["stateEstimate.roll"]), color='k')
axes[axs_id].legend()
axes[axs_id].set_title("roll command")
axes[axs_id].set_xlabel("timestep (0.002ms)")
axs_id += 1

data["stateEstimate.pitch"].plot(ax=axes[axs_id], label="pitch")
data["controller.pitch"].plot(ax=axes[axs_id], label="pitch command")
axes[axs_id].axhline(0, 0, len(data["stateEstimate.pitch"]), color='k')
# axes[axs_id].set_ylim([-35000, 35000])
axes[axs_id].legend()
axes[axs_id].set_title("pitch command")
axes[axs_id].set_xlabel("timestep (0.002ms)")
axs_id += 1

(data["pid_attitude.pitch_output"] - data["pid_attitude.pitch_outI"]).plot(ax=axes[axs_id], label="pid")
(-data["gyro.y"]).plot(ax=axes[axs_id], label="gyroy")
axes[axs_id].axhline(0, 0, len(data["pid_attitude.pitch_output"]), color='k')
axes[axs_id].legend()
axes[axs_id].set_title("Pitch rate")
axs_id += 1

(data["pid_attitude.roll_output"] - data["pid_attitude.roll_outI"]).plot(ax=axes[axs_id], label="pid")
data["gyro.x"].plot(ax=axes[axs_id], label="gyrox")
axes[axs_id].axhline(0, 0, len(data["pid_attitude.roll_output"]), color='k')
axes[axs_id].legend()
axes[axs_id].set_title("Roll rate")
axes[axs_id].set_xlabel("timestep (0.002ms)")

axs_id += 1
(data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).plot(ax=axes[axs_id], label="pid pitch")
data["snn_control.torque_pitch"].plot(ax=axes[axs_id], label="snn pitch")
# data["pid_rate.pitch_output"].plot(ax=axes[axs_id], label="pid pitch")
# data["pid_rate.roll_output"].plot(ax=axes[axs_id], label="pid roll")

axs_id += 1
(data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]).plot(ax=axes[axs_id], label="pid roll")
data["snn_control.torque_roll"].plot(ax=axes[axs_id], label="snn roll")
# data["pid_rate.pitch_outI"].plot(ax=axes[axs_id], label="pid pitch")
# data["pid_rate.roll_outI"].plot(ax=axes[axs_id], label="pid roll")

plt.show()