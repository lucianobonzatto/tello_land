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

def plot_grafico2d(ax, vel_uav, cmd_vel):
    # vel_uav = vel_uav.sort_values(by='Time')
    vel_uav["Time"] -= vel_uav["Time"][0]

    time_x = [0.67, 0.80]
    time_y = [0.00, 0.08]
    time_z = [0.17, 0.23]
    time_r = [0.92, 0.98]

    # if(False):
    if(True):
        vel_uav_x = vel_uav[(vel_uav["Time"] >= time_x[0]*1e11) & (vel_uav["Time"] <= time_x[1]*1e11)]
        vel_uav_y = vel_uav[(vel_uav["Time"] >= time_y[0]*1e11) & (vel_uav["Time"] <= time_y[1]*1e11)]
        vel_uav_z = vel_uav[(vel_uav["Time"] >= time_z[0]*1e11) & (vel_uav["Time"] <= time_z[1]*1e11)]
        vel_uav_r = vel_uav[(vel_uav["Time"] >= time_r[0]*1e11) & (vel_uav["Time"] <= time_r[1]*1e11)]

        cmd_vel_x = cmd_vel[(cmd_vel["Time"] >= time_x[0]*1e11) & (cmd_vel["Time"] <= time_x[1]*1e11)]
        cmd_vel_y = cmd_vel[(cmd_vel["Time"] >= time_y[0]*1e11) & (cmd_vel["Time"] <= time_y[1]*1e11)]
        cmd_vel_z = cmd_vel[(cmd_vel["Time"] >= time_z[0]*1e11) & (cmd_vel["Time"] <= time_z[1]*1e11)]
        cmd_vel_r = cmd_vel[(cmd_vel["Time"] >= time_r[0]*1e11) & (cmd_vel["Time"] <= time_r[1]*1e11)]
    else:
        vel_uav_x = vel_uav
        vel_uav_y = vel_uav
        vel_uav_z = vel_uav
        vel_uav_r = vel_uav

        cmd_vel_x = cmd_vel
        cmd_vel_y = cmd_vel
        cmd_vel_z = cmd_vel
        cmd_vel_r = cmd_vel

    # ax[0].plot(vel_uav["Time"].to_numpy(), c='b', label=f'vel_uav')
    # ax[0].plot(cmd_vel["Time"].to_numpy(), c='r', label=f'cmd_vel')

    # ax[0].plot(vel_uav["X Position"].to_numpy(), c='b', label=f'vel_uav')
    # ax[0].plot(cmd_vel["X Position"].to_numpy(), c='r', label=f'cmd_vel')

    # ax[1].plot(vel_uav["Y Position"].to_numpy(), c='b', label=f'vel_uav')
    # ax[1].plot(cmd_vel["Y Position"].to_numpy(), c='r', label=f'cmd_vel')

    # ax[2].plot(vel_uav["Z Position"].to_numpy(), c='b', label=f'vel_uav')
    # ax[2].plot(cmd_vel["Z Position"].to_numpy(), c='r', label=f'cmd_vel')

    # ax[3].plot(vel_uav["yaw"].to_numpy(), c='b', label=f'vel_uav')
    # ax[3].plot(cmd_vel["yaw"].to_numpy(), c='r', label=f'cmd_vel')

    # ax[0].scatter(vel_uav["Time"].to_numpy(), vel_uav["Time"].to_numpy(), c='b', label=f'tello')
    ax[0].scatter(vel_uav_x["Time"].to_numpy(), vel_uav_x["X Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[1].scatter(vel_uav_y["Time"].to_numpy(), vel_uav_y["Y Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[2].scatter(vel_uav_z["Time"].to_numpy(), vel_uav_z["Z Position"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[3].scatter(vel_uav_r["Time"].to_numpy(), vel_uav_r["yaw"].to_numpy(), c='b', label='vel_uav', s=5)

    ax[0].scatter(cmd_vel_x["Time"].to_numpy(), cmd_vel_x["X Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[1].scatter(cmd_vel_y["Time"].to_numpy(), cmd_vel_y["Y Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[2].scatter(cmd_vel_z["Time"].to_numpy(), cmd_vel_z["Z Position"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[3].scatter(cmd_vel_r["Time"].to_numpy(), cmd_vel_r["yaw"].to_numpy(), c='r', label='cmd_vel', s=5)


    for a in ax.flat:
        a.set_xlabel('Time')
        a.legend()

    ax[0].set_ylabel('X position (m)')
    ax[1].set_ylabel('Y position (m)')
    ax[2].set_ylabel('Z position (m)')
    ax[3].set_ylabel('Yaw position (rad)')

controller = ['Cascade']
fig, ax = plt.subplots(4, 1, figsize=(15, 9))

vel_uav = ler_csv("log/csv/ft/vel_uav.csv")
cmd_vel = ler_csv("log/csv/ft/cmd_vel.csv")

vel_uav["Time"] -= vel_uav["Time"][0]
cmd_vel["Time"] -= cmd_vel["Time"][0]

plot_grafico2d(ax, vel_uav, cmd_vel)

plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
