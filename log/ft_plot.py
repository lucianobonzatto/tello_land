import csv
import pandas as pd
import matplotlib.pyplot as plt

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

def plot_uav(ax, uav_x, uav_y, uav_z, uav_r):
    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["yaw_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)

    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["yaw_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)

    ax[0].set_ylabel('X')
    ax[1].set_ylabel('Y')
    ax[2].set_ylabel('Z')
    ax[3].set_ylabel('Yaw')

uav_1 = ler_csv("log/csv/ft/uav_1.csv")
uav_2 = ler_csv("log/csv/ft/uav_2.csv")
uav_3 = ler_csv("log/csv/ft/uav_3.csv")
uav_4 = ler_csv("log/csv/ft/uav_4.csv")
uav_5 = ler_csv("log/csv/ft/uav_5.csv")
uav_6 = ler_csv("log/csv/ft/uav_6.csv")
uav_7 = ler_csv("log/csv/ft/uav_7.csv")
uav_8= ler_csv("log/csv/ft/uav_8.csv")
uav_1["Time"] -= uav_1["Time"][0]
uav_2["Time"] -= uav_2["Time"][0]
uav_3["Time"] -= uav_3["Time"][0]
uav_4["Time"] -= uav_4["Time"][0]
uav_5["Time"] -= uav_5["Time"][0]
uav_6["Time"] -= uav_6["Time"][0]
uav_7["Time"] -= uav_7["Time"][0]
uav_8["Time"] -= uav_8["Time"][0]

time_x = [80, 10000]
time_y = [0, 10000]
time_z = [60, 10000]
time_r = [0, 25]

uav_x = uav_7
uav_y = uav_6
uav_z = uav_8
uav_r = uav_3
    
uav_xf = uav_x[(uav_x["Time"] >= time_x[0]) & (uav_x["Time"] <= time_x[1])]
uav_yf = uav_y[(uav_y["Time"] >= time_y[0]) & (uav_y["Time"] <= time_y[1])]
uav_zf = uav_z[(uav_z["Time"] >= time_z[0]) & (uav_z["Time"] <= time_z[1])]
uav_rf = uav_r[(uav_r["Time"] >= time_r[0]) & (uav_r["Time"] <= time_r[1])]

uav_xf.to_csv("log/csv/output/uav_x_v1.csv", index=False)
uav_yf.to_csv("log/csv/output/uav_y_v1.csv", index=False)
uav_zf.to_csv("log/csv/output/uav_z_v1.csv", index=False)
uav_rf.to_csv("log/csv/output/uav_r_v1.csv", index=False)

fig, ax = plt.subplots(4, 1, figsize=(15, 9))

# plot_uav(ax, uav_x, uav_y, uav_z, uav_r)
plot_uav(ax, uav_xf, uav_yf, uav_zf, uav_rf)

plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
