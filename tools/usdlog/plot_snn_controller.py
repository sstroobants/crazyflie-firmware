import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("tools/usdlog/att_dist/lilac_wood.csv")

fig, axes = plt.subplots(nrows=6, ncols=1, sharex=True)

axs_id = 0
data["stateEstimate.roll"].plot(ax=axes[axs_id], label="roll")
data["controller.roll"].plot(ax=axes[axs_id], label="roll command")
# data["snn_control.torque_roll"].rolling(2).mean().plot(ax=axes[axs_id], label="snn")
# (data["pid_attitude.roll_outI"] * 100).plot(ax=axes[axs_id], label="pid I")
# data["pid_rate.roll_output"].plot(ax=axes[axs_id], label="pid")
# (data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]).plot(ax=axes[axs_id], label="pd")
# data["pid_rate.roll_outI"].plot(ax=axes[axs_id], label="pid I")
# axes[axs_id].axhline(0, 0, len(data["snn_control.torque_roll"]), color='k')
# axes[axs_id].set_ylim([-35000, 35000])
axes[axs_id].legend()
axes[axs_id].set_title("roll command")
axes[axs_id].set_xlabel("timestep (0.002ms)")
axs_id += 1
data["stateEstimate.pitch"].plot(ax=axes[axs_id], label="pitch")
data["controller.pitch"].plot(ax=axes[axs_id], label="pitch command")
# data["snn_control.torque_pitch"].rolling(2).mean().plot(ax=axes[axs_id], label="snn")
# (data["pid_attitude.pitch_outI"] * 100).plot(ax=axes[axs_id], label="pid I")
# data["pid_rate.pitch_output"].plot(ax=axes[axs_id], label="pid")
# (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).plot(ax=axes[axs_id], label="pd")
# data["pid_rate.pitch_outI"].plot(ax=axes[axs_id], label="pid I")
axes[axs_id].axhline(0, 0, len(data["snn_control.torque_pitch"]), color='k')
# axes[axs_id].set_ylim([-35000, 35000])
axes[axs_id].legend()
axes[axs_id].set_title("pitch command")
axes[axs_id].set_xlabel("timestep (0.002ms)")
axs_id += 1
data["snn_control.torque_pitch"].rolling(2).mean().plot(ax=axes[axs_id], label="snn")
(-data["gyro.y"]).plot(ax=axes[axs_id], label="gyroy")
# (data["snn_control.roll_integ"] * 100).plot(ax=axes[axs_id], label="snn x")
# (data["snn_control.pitch_integ"] * 100).plot(ax=axes[axs_id], label="snn y")
# axes[axs_id].set_ylim([-1200, 1200])
axes[axs_id].legend()
axes[axs_id].set_title("Pitch rate")
axs_id += 1
data["snn_control.torque_roll"].rolling(2).mean().plot(ax=axes[axs_id], label="snn")
data["gyro.x"].plot(ax=axes[axs_id], label="gyrox")
# (data["snn_control.torque_pitch"].rolling(2).mean() + data["pid_attitude.roll_outI"]*600).plot(ax=axes[axs_id], label="snn")
# data["pid_rate.pitch_output"].plot(ax=axes[axs_id], label='pid')
# (data["pid_attitude.pitch_outI"] * 100).plot(ax=axes[axs_id], label="pid I")
# ((data["controller.pitch"] - data["stateEstimate.pitch"]) * 6 - data["gyro.x"] * 0).plot(ax=axes[axs_id], label="pid")
# (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).plot(ax=axes[axs_id], label="pd")
# data["pid_rate.pitch_outI"].plot(ax=axes[axs_id], label="pid I")
axes[axs_id].axhline(0, 0, len(data["snn_control.torque_pitch"]), color='k')
# axes[axs_id].set_ylim([-35000, 35000])
axes[axs_id].legend()
axes[axs_id].set_title("Roll rate")
axes[axs_id].set_xlabel("timestep (0.002ms)")
# axs_id += 1
# ((data["snn_control.torque_roll"].rolling(2).mean() - data["gyro.x"])).cumsum().plot(ax=axes[axs_id], label="integ roll")
# (2 *data["snn_control.roll_integ"]).rolling(2).mean().plot(ax=axes[axs_id], label="snn roll")
# axes[axs_id].legend()
# axs_id += 1
# ((data["snn_control.torque_pitch"].rolling(2).mean() + data["gyro.y"])).cumsum().plot(ax=axes[axs_id], label="integ pitch")
# (2 * data["snn_control.pitch_integ"]).rolling(2).mean().plot(ax=axes[axs_id], label="snn pitch")
# axes[axs_id].legend()


# data["snn_control.pitch_integ"].rolling(2).mean().plot(ax=axes[axs_id], label="snn")
# data["controller.yaw"].plot(ax=axes[axs_id], label="yaw")
# data["controller.yawRate"].plot(ax=axes[axs_id], label="yawrate")
# data["pid_rate.yaw_output"].plot(ax=axes[axs_id], label="yawrate")
plt.show()

print((data["snn_control.torque_roll"] - data["gyro.x"]).mean())
print((data["snn_control.torque_pitch"] + data["gyro.y"]).mean())