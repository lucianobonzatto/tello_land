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

def plot_pose(ax, time, pose, label):
    ax.scatter(time.to_numpy(), pose.to_numpy(), c='b', s=5)
    ax.set_ylabel(label)

time_z = [0, 32]
aruco_z = ler_csv("log/csv/pose/Gains/aruco_z.csv")
aruco_z = aruco_z[(aruco_z["Time"] >= time_z[0]) & (aruco_z["Time"] <= time_z[1])]

fig, ax = plt.subplots(2, 1, figsize=(15, 9))
plot_pose(ax[0], aruco_z["Time"], aruco_z["Z_pose_uav"], "Z_pose_uav")

plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
