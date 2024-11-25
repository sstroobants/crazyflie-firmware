# import pandas as pd
# import matplotlib.pyplot as plt
# import matplotlib.ticker as ticker
# import numpy as np
# import scipy.stats
# import scipy.signal
# import os

# fig, axs = plt.subplots(2, 1, figsize=[4, 3.5])
# # axs = [axs]
# rc_fonts = {
#     "font.family": "serif",
#     # "font.serif": "libertine",
#     # "font.size": 20,
#     # 'figure.figsize': (5, 3),
#     "text.usetex": True,
#     # # 'text.latex.preview': True,
#     # "text.latex.preamble": r"usepackage{libertine}",
# }
# plt.rc('text', usetex=True)
# plt.rc('text.latex', preamble=r'\usepackage{libertine}')
# font_serif = {"family": "serif"}
# plt.rcParams.update(rc_fonts)

# axs[0].set_facecolor("#EBEBEB")
# axs[0].grid()
# alpha = 1.0

# folders = ["tools/usdlog/logs/27_08/att_dist/crimson_dew"]
# # folders = ["tools/usdlog/logs/05_09/att_dist/lilac_wood", "tools/usdlog/logs/05_09/att_dist/crimson_dew", "tools/usdlog/logs/05_09/att_dist/desert_snow", "tools/usdlog/logs/05_09/att_dist/pid"]
# # fig = plt.figure(figsize=[3,3])
# colors = ["#2c7bb6", "", "", "#d7191c"]
# color_idx = 0
# for folder in folders:
#     avg_pearson_list = []
#     for dataset in os.listdir(folder):
#         # only use .csv files
#         if not dataset.endswith(".csv"):
#             continue
#         data = pd.read_csv(f"{folder}/{dataset}")
#         pitch_list = []
#         roll_list = []
#         avg_pearson = []
#         pearson_first = None
#         for i in range(0, -13, -1):
#             # calculate correlation coeff for different shifted outputs
#             pitch = scipy.stats.pearsonr((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]), data["snn_control.torque_pitch"].shift(i).fillna(0))
#             roll = scipy.stats.pearsonr((data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]), data["snn_control.torque_roll"].shift(i).fillna(0))
#             avg_prs = (pitch[0] + roll[0]) / 2
#             pitch_list.append(pitch[0])
#             roll_list.append(roll[0])

#             avg_pearson.append(np.array(avg_prs))
#             # print(avg_prs)
#         print(f"[{folder}/{dataset}] Avg max: {np.argmax(avg_pearson)}, pitch max: {np.argmax(pitch_list)}, roll max: {np.argmax(roll_list)}")
#         avg_pearson_list.append(avg_pearson)

#     # plot average correlation for all datasets
#     axs[0].plot(np.mean(avg_pearson_list, axis=0), label=f"{folder.split('/')[-1]}", color=colors[0])

#     color_idx += 1

# axs[0].legend(["SNN delay"], fancybox=True, shadow=True, loc="upper right")

# axs[0].tick_params(labelsize="x-small", direction="in")
# for tick in axs[0].get_xticklabels():
#     tick.set_fontname(font_serif["family"])
#     # tick.set_visible(False)

# for tick in axs[0].get_yticklabels():
#     tick.set_fontname(font_serif["family"])
# axs[0].set_ylabel("pearson correlation", fontname=font_serif["family"])
# axs[0].set_xlabel("timeshift $d$ [timesteps]", fontname=font_serif["family"])

# # inset_ax = axs[0].inset_axes(
# #    [0.25, 0.1, 0.4, 0.4],  # [x, y, width, height] w.r.t. axes
# #     xlim=[-0.2, 0.2], ylim=[0.695, 0.705], # sets viewport & tells relation to main axes
# #     xticklabels=[], yticklabels=[]
# # )
# inset_ax = axs[1]

# inset_ax.plot((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).rolling(5).mean(), color=colors[3]) 
# inset_ax.plot(data["snn_control.torque_pitch"].rolling(5).mean(), color=colors[0], alpha=0.8)
# # Also plot shifted data
# inset_ax.plot(data["snn_control.torque_pitch"].shift(-6).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dotted")
# inset_ax.plot(data["snn_control.torque_pitch"].shift(-12).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dashed")
# # axs[0].indicate_inset_zoom(inset_ax, edgecolor="black")
# inset_ax.set_xlim([1100, 1250])
# inset_ax.set_ylim([-8000, 8000])

# axs[1].set_facecolor("#EBEBEB")
# axs[1].grid()

# axs[1].tick_params(labelsize="x-small", direction="in")
# for tick in axs[1].get_xticklabels():
#     tick.set_fontname(font_serif["family"])
#     # tick.set_visible(False)

# for tick in axs[1].get_yticklabels():
#     tick.set_fontname(font_serif["family"])

# axs[1].set_ylabel("motor command", fontname=font_serif["family"])
# axs[1].set_xlabel("time [ms]", fontname=font_serif["family"])

# # add legend beneath plot on a row of its own
# axs[1].legend(["ref", "$d=0$", "$d=6$", "$d=12$"], 
#                 fancybox=True, 
#                 shadow=True, 
#                 loc="lower right", 
#                 ncols=4, 
#                 columnspacing=0.6)

# # Set the y-axis formatter to scientific notation
# formatter = ticker.ScalarFormatter(useMathText=True)
# formatter.set_powerlimits((-3, 3))  # Display power when outside this range
# axs[1].yaxis.set_major_formatter(formatter)

# # Move the scientific notation text to the top left corner and set its font to Libertine
# offset_text = axs[1].yaxis.get_offset_text()
# offset_text.set_position((0, 1))
# offset_text.set_verticalalignment('bottom')
# offset_text.set_horizontalalignment('left')

# fig.tight_layout()
# plt.savefig("../crazyflie_snn/figures/delay.png", dpi=400)

# plt.tight_layout()
# plt.show()

# import pandas as pd
# import matplotlib.pyplot as plt
# import matplotlib.ticker as ticker
# import numpy as np
# import scipy.stats
# import os

# # Set up fonts and LaTeX preamble for consistent use of Libertine
# rc_fonts = {
#     "text.usetex": True,
#     "font.family": "serif",  # Default to serif
#     "text.latex.preamble": r"\usepackage{libertine}"  # Use Libertine font
# }
# plt.rcParams.update(rc_fonts)

# fig, axs = plt.subplots(2, 1, figsize=[4, 3.5])

# axs[0].set_facecolor("#EBEBEB")
# axs[0].grid()
# alpha = 1.0

# folders = ["tools/usdlog/logs/27_08/att_dist/crimson_dew"]
# colors = ["#2c7bb6", "", "", "#d7191c"]
# color_idx = 0

# # Loop through folders and calculate correlations
# for folder in folders:
#     avg_pearson_list = []
#     for dataset in os.listdir(folder):
#         if not dataset.endswith(".csv"):
#             continue
#         data = pd.read_csv(f"{folder}/{dataset}")
#         pitch_list = []
#         roll_list = []
#         avg_pearson = []
#         for i in range(0, -13, -1):
#             pitch = scipy.stats.pearsonr(
#                 (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]),
#                 data["snn_control.torque_pitch"].shift(i).fillna(0)
#             )
#             roll = scipy.stats.pearsonr(
#                 (data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]),
#                 data["snn_control.torque_roll"].shift(i).fillna(0)
#             )
#             avg_prs = (pitch[0] + roll[0]) / 2
#             pitch_list.append(pitch[0])
#             roll_list.append(roll[0])
#             avg_pearson.append(np.array(avg_prs))
#         avg_pearson_list.append(avg_pearson)

#     axs[0].plot(np.mean(avg_pearson_list, axis=0), label=f"{folder.split('/')[-1]}", color=colors[0])
#     color_idx += 1

# axs[0].legend(["SNN delay"], fancybox=True, shadow=True, loc="upper right")
# axs[0].tick_params(labelsize="x-small", direction="in")
# axs[0].set_ylabel("pearson correlation")
# axs[0].set_xlabel("timeshift $d$ [timesteps]")

# inset_ax = axs[1]
# inset_ax.plot((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).rolling(5).mean(), color=colors[3]) 
# inset_ax.plot(data["snn_control.torque_pitch"].rolling(5).mean(), color=colors[0], alpha=0.8)
# inset_ax.plot(data["snn_control.torque_pitch"].shift(-6).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dotted")
# inset_ax.plot(data["snn_control.torque_pitch"].shift(-12).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dashed")
# inset_ax.set_xlim([1100, 1250])
# inset_ax.set_ylim([-8000, 8000])

# axs[1].set_facecolor("#EBEBEB")
# axs[1].grid()
# axs[1].tick_params(labelsize="x-small", direction="in")
# axs[1].set_ylabel("motor command")
# axs[1].set_xlabel("time [ms]")

# # Add legend
# axs[1].legend(["ref", "$d=0$", "$d=6$", "$d=12$"], fancybox=True, shadow=True, loc="lower right", ncols=4, columnspacing=0.6)

# # Set y-axis to scientific notation
# formatter = ticker.ScalarFormatter(useMathText=True)
# formatter.set_powerlimits((-3, 3))
# axs[1].yaxis.set_major_formatter(formatter)

# # Move scientific notation text to the top left corner
# offset_text = axs[1].yaxis.get_offset_text()
# offset_text.set_position((0, 1))
# offset_text.set_verticalalignment('bottom')
# offset_text.set_horizontalalignment('left')

# fig.tight_layout()
# plt.savefig("../crazyflie_snn/figures/delay.png", dpi=400)
# plt.show()






import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import scipy.stats
import os

# Set up fonts and LaTeX preamble for consistent use of Libertine
rc_fonts = {
    "text.usetex": True,
    "font.family": "serif",  # Default to serif
    "text.latex.preamble": r"\usepackage{libertine}"  # Use Libertine font
}
plt.rcParams.update(rc_fonts)

fig, axs = plt.subplots(2, 1, figsize=[4, 3.5])

# Reduce space between plots
plt.subplots_adjust(hspace=0)

axs[0].set_facecolor("#EBEBEB")
axs[0].grid()
alpha = 1.0

folders = ["tools/usdlog/logs/27_08/att_dist/crimson_dew"]
colors = ["#2c7bb6", "", "", "#d7191c"]
color_idx = 0

# Loop through folders and calculate correlations
for folder in folders:
    avg_pearson_list = []
    for dataset in os.listdir(folder):
        if not dataset.endswith(".csv"):
            continue
        data = pd.read_csv(f"{folder}/{dataset}")
        pitch_list = []
        roll_list = []
        avg_pearson = []
        for i in range(0, -13, -1):
            pitch = scipy.stats.pearsonr(
                (data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]),
                data["snn_control.torque_pitch"].shift(i).fillna(0)
            )
            roll = scipy.stats.pearsonr(
                (data["pid_rate.roll_output"] - data["pid_rate.roll_outI"]),
                data["snn_control.torque_roll"].shift(i).fillna(0)
            )
            avg_prs = (pitch[0] + roll[0]) / 2
            pitch_list.append(pitch[0])
            roll_list.append(roll[0])
            avg_pearson.append(np.array(avg_prs))
        avg_pearson_list.append(avg_pearson)

    axs[0].plot(np.mean(avg_pearson_list, axis=0), label=f"{folder.split('/')[-1]}", color=colors[0])
    color_idx += 1

axs[0].set_ylim([0.69, 0.81])
axs[0].legend(["SNN delay"], fancybox=True, shadow=True, loc="upper right")
axs[0].tick_params(labelsize="small", direction="in")
axs[0].set_ylabel("Pearson Correlation")
axs[0].set_xlabel("Timeshift $d$ [timesteps]")

inset_ax = axs[1]
inset_ax.plot((data["pid_rate.pitch_output"] - data["pid_rate.pitch_outI"]).rolling(5).mean(), color=colors[3]) 
inset_ax.plot(data["snn_control.torque_pitch"].rolling(5).mean(), color=colors[0], alpha=0.8)
inset_ax.plot(data["snn_control.torque_pitch"].shift(-6).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dotted")
inset_ax.plot(data["snn_control.torque_pitch"].shift(-12).fillna(0).rolling(5).mean(), color=colors[0], alpha=0.8, linestyle="dashed")
inset_ax.set_xlim([1100, 1250])
inset_ax.set_ylim([-8000, 8000])

axs[1].set_facecolor("#EBEBEB")
axs[1].grid()
axs[1].tick_params(labelsize="small", direction="in")
axs[1].set_ylabel("Motor Command")
axs[1].set_xlabel("Time [ms]")

# Add legend
axs[1].legend(["ref", "$d=0$", "$d=6$", "$d=12$"], fancybox=True, shadow=True, loc="lower right", ncols=4, columnspacing=0.6)

# Set y-axis to scientific notation
formatter = ticker.ScalarFormatter(useMathText=True)
formatter.set_powerlimits((-3, 3))
axs[1].yaxis.set_major_formatter(formatter)

# Move scientific notation text to the top left corner and decrease font size
offset_text = axs[1].yaxis.get_offset_text()
offset_text.set_position((0, 1))
offset_text.set_verticalalignment('bottom')
offset_text.set_horizontalalignment('left')
offset_text.set_fontsize(8)  # Adjust the font size as desired

fig.tight_layout()
plt.savefig("../crazyflie_snn/figures/delay.svg")
plt.show()
