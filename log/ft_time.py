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

uav_x = ler_csv("log/csv/ft/uav_x.csv")
uav_y = ler_csv("log/csv/ft/uav_y.csv")
uav_z = ler_csv("log/csv/ft/uav_z.csv")
uav_r = ler_csv("log/csv/ft/uav_r.csv")

intervalo_tempo_x = uav_x['Time'].diff()
intervalo_tempo_y = uav_y['Time'].diff()
intervalo_tempo_z = uav_z['Time'].diff()
intervalo_tempo_r = uav_r['Time'].diff()

print("intervalo_tempo_x: ", intervalo_tempo_x.mean())
print("intervalo_tempo_y: ", intervalo_tempo_y.mean())
print("intervalo_tempo_z: ", intervalo_tempo_z.mean())
print("intervalo_tempo_r: ", intervalo_tempo_r.mean())


uav_x['Time'] = uav_x.index * intervalo_tempo_x.mean()
uav_y['Time'] = uav_y.index * intervalo_tempo_y.mean()
uav_z['Time'] = uav_z.index * intervalo_tempo_z.mean()
uav_r['Time'] = uav_r.index * intervalo_tempo_r.mean()

uav_x.to_csv("log/csv/ft/uav_x.csv", index=False)
uav_y.to_csv("log/csv/ft/uav_y.csv", index=False)
uav_z.to_csv("log/csv/ft/uav_z.csv", index=False)
uav_r.to_csv("log/csv/ft/uav_r.csv", index=False)
