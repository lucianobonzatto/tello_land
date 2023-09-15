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

def plot_grafico2d(ax,vel_uav_x, vel_uav_y, vel_uav_z, vel_uav_r, cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_vel_r):
    ax[0].scatter(vel_uav_x["Time"].to_numpy(), vel_uav_x["X Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[1].scatter(vel_uav_y["Time"].to_numpy(), vel_uav_y["Y Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[2].scatter(vel_uav_z["Time"].to_numpy(), vel_uav_z["Z Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[3].scatter(vel_uav_r["Time"].to_numpy(), vel_uav_r["yaw"].to_numpy(), c='b', label='vel_uav', s=5)

    ax[0].scatter(cmd_vel_x["Time"].to_numpy(), cmd_vel_x["X Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[1].scatter(cmd_vel_y["Time"].to_numpy(), cmd_vel_y["Y Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[2].scatter(cmd_vel_z["Time"].to_numpy(), cmd_vel_z["Z Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[3].scatter(cmd_vel_r["Time"].to_numpy(), cmd_vel_r["yaw"].to_numpy(), c='r', label='cmd_vel', s=5)

    ax[0].set_ylabel('X')
    ax[1].set_ylabel('Y')
    ax[2].set_ylabel('Z')
    ax[3].set_ylabel('Yaw')

vel_uav_1 = ler_csv("log/csv/ft/vel_uav_1.csv")
cmd_vel_1 = ler_csv("log/csv/ft/cmd_vel_1.csv")
vel_uav_1["Time"] -= vel_uav_1["Time"][0]
cmd_vel_1["Time"] -= cmd_vel_1["Time"][0]

time_x = [0.67, 0.80]
time_y = [0.00, 0.08]
time_z = [0.17, 0.23]
time_r = [0.92, 0.98]

# if(False):
if(True):
    vel_uav_x = vel_uav_1[(vel_uav_1["Time"] >= time_x[0]*1e11) & (vel_uav_1["Time"] <= time_x[1]*1e11)]
    vel_uav_y = vel_uav_1[(vel_uav_1["Time"] >= time_y[0]*1e11) & (vel_uav_1["Time"] <= time_y[1]*1e11)]
    vel_uav_z = vel_uav_1[(vel_uav_1["Time"] >= time_z[0]*1e11) & (vel_uav_1["Time"] <= time_z[1]*1e11)]
    vel_uav_r = vel_uav_1[(vel_uav_1["Time"] >= time_r[0]*1e11) & (vel_uav_1["Time"] <= time_r[1]*1e11)]

    cmd_vel_x = cmd_vel_1[(cmd_vel_1["Time"] >= time_x[0]*1e11) & (cmd_vel_1["Time"] <= time_x[1]*1e11)]
    cmd_vel_y = cmd_vel_1[(cmd_vel_1["Time"] >= time_y[0]*1e11) & (cmd_vel_1["Time"] <= time_y[1]*1e11)]
    cmd_vel_z = cmd_vel_1[(cmd_vel_1["Time"] >= time_z[0]*1e11) & (cmd_vel_1["Time"] <= time_z[1]*1e11)]
    cmd_vel_r = cmd_vel_1[(cmd_vel_1["Time"] >= time_r[0]*1e11) & (cmd_vel_1["Time"] <= time_r[1]*1e11)]
else:
    vel_uav_x = vel_uav_1
    vel_uav_y = vel_uav_1
    vel_uav_z = vel_uav_1
    vel_uav_r = vel_uav_1

    cmd_vel_x = cmd_vel_1
    cmd_vel_y = cmd_vel_1
    cmd_vel_z = cmd_vel_1
    cmd_vel_r = cmd_vel_1

fig, ax = plt.subplots(4, 1, figsize=(15, 9))
plot_grafico2d(ax,vel_uav_x, vel_uav_y, vel_uav_z, vel_uav_r, cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_vel_r)
plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()