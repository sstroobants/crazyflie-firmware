import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats


avg_pearson_list = []

fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True)
axs_id = 0
axes[axs_id].set_title("x vs y (SNN)")
datasets = [f"snn_tests/log{i:02d}" for i in range(7, 12)]
for dataset in datasets:
    data = pd.read_csv(f"tools/usdlog/{dataset}.csv")

    # plot x vs y   
    data.plot(x="locSrv.x", y="locSrv.y", color="red", ax=axes[axs_id])
    data.plot(x="posCtl.targetX", y="posCtl.targetY", color="blue", ax=axes[axs_id])
axes[axs_id].set_xlabel("x")
axes[axs_id].set_ylabel("y")
axes[axs_id].legend(["optitrack", "target"])

axs_id += 1
axes[axs_id].set_title("x vs y (PID)")
datasets = [f"pid_tests/log{i:02d}" for i in range(0, 5)]
for dataset in datasets:
    data = pd.read_csv(f"tools/usdlog/{dataset}.csv")

    # plot x vs y   
    data.plot(x="locSrv.x", y="locSrv.y", color="red", ax=axes[axs_id])
    data.plot(x="posCtl.targetX", y="posCtl.targetY", color="blue", ax=axes[axs_id])
axes[axs_id].set_xlabel("x")
axes[axs_id].set_ylabel("y")
axes[axs_id].legend(["optitrack", "target"])

plt.show()