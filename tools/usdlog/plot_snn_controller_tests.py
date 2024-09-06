import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_datasets(folder, axs_id, axes, color1, color2, title):
    datasets = os.listdir(folder)
    for dataset in datasets:
        if not dataset.endswith(".csv"):
            continue
        data = pd.read_csv(f"{folder}/{dataset}")
        data["timestamp"] = data["timestamp"] - data["timestamp"].iloc[0]
        data = data[data["timestamp"] > 5000]

        # # Plot time vs x  
        axes[axs_id].plot(data["timestamp"], data["locSrv.x"], color=color1, alpha=0.5, linewidth=1.5, label="OptiTrack")
        axes[axs_id].plot(data["timestamp"], data["ctrltarget.x"], color=color2, linewidth=3, label="Target")   

        # Plot x vs y
        # axes[axs_id].plot(data["locSrv.x"], data["locSrv.y"], color=color1, alpha=0.5, linewidth=1.5, label="OptiTrack")
        # axes[axs_id].plot(data["ctrltarget.x"], data["ctrltarget.y"], color=color2, linewidth=3, label="Target")

    axes[axs_id].set_title(title, fontsize=14)
    axes[axs_id].set_xlabel("Time (s)", fontsize=12)
    axes[axs_id].set_ylabel("Position (x)", fontsize=12)
    axes[axs_id].legend(["Optitrack", "Target"], fontsize=10)
    axes[axs_id].grid(True)

# Create a figure and axes
fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True, figsize=(6,6))

# Colorblind-friendly colors
color1 = "#1f77b4"  # Blue
color2 = "#ff7f0e"  # Orange

# SNN dataset
axs_id = 0
snn_folder = "tools/usdlog/step_20_08/snn"
plot_datasets(snn_folder, axs_id, axes, color1, color2, "x vs y (SNN)")

# PID dataset
axs_id += 1
pid_folder = "tools/usdlog/step_20_08/pid"
plot_datasets(pid_folder, axs_id, axes, color1, color2, "x vs y (PID)")

# Adjust layout
plt.tight_layout()
# plt.show()
plt.savefig("tools/usdlog/snn_pid_comparison.png", dpi=300)