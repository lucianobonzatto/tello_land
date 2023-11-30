import csv
import pandas as pd
import matplotlib.pyplot as plt

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        df["Time"] -= df["Time"][0]
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None
    
def plot_uav(ax, uav_x, uav_y, uav_z, uav_r):
    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["R_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)

    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["R_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)

    ax[0].set_ylabel('X')
    ax[1].set_ylabel('Y')
    ax[2].set_ylabel('Z')
    ax[3].set_ylabel('Yaw')

# uav_1 = ler_csv("log/csv/Gains/1.csv")
uav_brz = ler_csv("log/csv/Gains/rz.csv")
uav_by = ler_csv("log/csv/Gains/y.csv")
uav_bx = ler_csv("log/csv/Gains/x.csv")

time_x = [45, 1000]
time_y = [0, 1000]
time_z = [90, 130]
time_r = [180, 1000]

uav_x = uav_bx[(uav_bx["Time"] >= time_x[0]) & (uav_bx["Time"] <= time_x[1])]
uav_y = uav_by[(uav_by["Time"] >= time_y[0]) & (uav_by["Time"] <= time_y[1])]
uav_z = uav_brz[(uav_brz["Time"] >= time_z[0]) & (uav_brz["Time"] <= time_z[1])]
uav_r = uav_brz[(uav_brz["Time"] >= time_r[0]) & (uav_brz["Time"] <= time_r[1])]

uav_y["Y_vel_uav"] = uav_y["X_vel_uav"]
uav_y["Y_cmd_vel"] = uav_y["X_cmd_vel"]

uav_x["X_vel_uav"] = uav_x["Y_vel_uav"]
uav_x["X_cmd_vel"] = uav_x["Y_cmd_vel"]

uav_x = uav_x.drop_duplicates(subset=uav_x.columns.difference(['Time', 'R_vel_uav']))
uav_y = uav_y.drop_duplicates(subset=uav_y.columns.difference(['Time', 'R_vel_uav']))
uav_z = uav_z.drop_duplicates(subset=uav_z.columns.difference(['Time', 'R_vel_uav']))
uav_r = uav_r.drop_duplicates(subset=uav_r.columns.difference(['Time']))

uav_x.to_csv('log/csv/output/uav_x_V05.csv', index=False)
uav_y.to_csv('log/csv/output/uav_y_V05.csv', index=False)
uav_z.to_csv('log/csv/output/uav_z_V05.csv', index=False)
uav_r.to_csv('log/csv/output/uav_r_V05.csv', index=False)

print(uav_x["Time"].diff().dropna().mean())
print(uav_y["Time"].diff().dropna().mean())
print(uav_z["Time"].diff().dropna().mean())
print(uav_r["Time"].diff().dropna().mean())

fig, ax = plt.subplots(4, 1, figsize=(15, 9))
plot_uav(ax, uav_x, uav_y, uav_z, uav_r)
plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
