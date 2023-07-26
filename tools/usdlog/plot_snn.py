import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("tools/usdlog/log03.csv")

fig, axes = plt.subplots(nrows=5, ncols=1, sharex=True)

axs_id = 0
data["snn_pid.pitchrate_p"].plot(ax=axes[axs_id])
data["pid_rate.pitch_outP"].plot(ax=axes[axs_id])
axes[axs_id].axhline(0, 0, len(data["snn_pid.pitchrate_p"]))
axes[axs_id].set_ylim([-20000, 20000])
axs_id += 1
data["snn_pid.pitchrate_i"].plot(ax=axes[axs_id])
data["pid_rate.pitch_outI"].plot(ax=axes[axs_id])
axes[axs_id].axhline(0, 0, len(data["snn_pid.pitchrate_i"]))
axes[axs_id].set_ylim([-20000, 20000])
axs_id += 1
data["snn_pid.pitchrate_d"].plot(ax=axes[axs_id])
data["pid_rate.pitch_outD"].plot(ax=axes[axs_id])
axes[axs_id].axhline(0, 0, len(data["snn_pid.pitchrate_d"]))
axes[axs_id].set_ylim([-20000, 20000])
axs_id += 1
data["gyro.y"].plot(ax=axes[axs_id], label="gyro")
data["controller.pitchRate"].plot(ax=axes[axs_id], label="target")
axes[axs_id].plot(- data["gyro.y"] - data["controller.pitchRate"], label="error")
axes[axs_id].axhline(0, 0, len(data["gyro.y"]))
axes[axs_id].set_ylim([-1200, 1200])
axes[axs_id].legend()
axs_id += 1
axes[axs_id].plot(data["snn_pid.pitchrate_p"] + data["snn_pid.pitchrate_i"] + data["snn_pid.pitchrate_d"], label="snn")
axes[axs_id].plot(data["pid_rate.pitch_outP"] + data["pid_rate.pitch_outI"] + data["pid_rate.pitch_outD"], label="pid")
axes[axs_id].axhline(0, 0, len(data["pid_rate.pitch_outP"]))
axes[axs_id].set_ylim([-15000, 15000])
axes[axs_id].legend()

plt.show()