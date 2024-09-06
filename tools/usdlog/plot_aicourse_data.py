import pandas as pd
import matplotlib.pyplot as plt


# def plot_aicourse_data(filename):
#     data = pd.read_csv(filename)
#     data.plot(x='locSrv.x', y='locSrv.y', label=filename[-10:])


if __name__ == "__main__":
    # fig = plt.figure()
    for i in range(0, 1):
        filename = f"tools/usdlog/korneel/log0{i}.csv"
        data = pd.read_csv(filename)
        # data = data[:2000]
        print("gyro", data["gyro.x"].std(), data["gyro.y"].std(), data["gyro.z"].std())
        print("acc", data["acc.x"].std(), data["acc.y"].std(), data["acc.z"].std())

        # plot gyro
        fig = plt.figure()
        data.plot(x='timestamp', y='gyro.x', label="gyro.x", ax=fig.gca())
        fig = plt.figure()
        data.plot(x='timestamp', y='acc.x', label="acc.x", ax=fig.gca())
    #     data.plot(x='timestamp', y='locSrv.qx', label=filename[-9:], ax=fig.gca())
        # plot_aicourse_data(f"tools/usdlog/aicourse/thrust_upgrade_rndwalk/log{i}.csv")
    # plot_aicourse_data("tools/usdlog/aicourse/thrust_upgrade_rnddist/log10.csv")


    plt.show()
