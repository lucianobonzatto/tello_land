import csv
import pandas as pd
import matplotlib.pyplot as plt

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        # df["timestamp"] -= df["timestamp"][0]
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

files = ["log/csv/PID/line_v1_2023-12-08-17-25-11", # 0
         "log/csv/PID/line_v2_2023-12-08-17-27-24", # 1
         "log/csv/PID/line_v3_2023-12-08-17-28-51", # 2
         "log/csv/PID/line_v4_2023-12-08-17-29-56", # 3
         "log/csv/PID/move_v2_2023-12-08-18-02-21", # 4
         "log/csv/PID/rot_v1_2023-12-08-17-33-44",  # 5
         "log/csv/PID/rot_v1_2023-12-08-17-34-29",  # 6
         "log/csv/PID/rot_v2_2023-12-08-17-35-24",  # 7
         "log/csv/PID/rot_v3_2023-12-08-17-36-13",  # 8
         "log/csv/PID/z_v1_2023-12-08-18-11-21",    # 9
         "log/csv/PID/z_v2_2023-12-08-18-10-33"]    # 10

index = 1

magni_pose = ler_csv(files[index] + "/magni_pose.csv")
aruco_pose = ler_csv(files[index] + "/aruco_pose.csv")
tello_odom = ler_csv(files[index] + "/tello_odom.csv")

fig, ax = plt.subplots(3, 4, figsize=(15, 9))
ax[0][0].scatter(magni_pose["timestamp"].to_numpy(), magni_pose["X_pose"].to_numpy(), c='b', s=5)
ax[0][1].scatter(magni_pose["timestamp"].to_numpy(), magni_pose["Y_pose"].to_numpy(), c='b', s=5)
ax[0][2].scatter(magni_pose["timestamp"].to_numpy(), magni_pose["Z_pose"].to_numpy(), c='b', s=5)
ax[0][3].scatter(magni_pose["timestamp"].to_numpy(), magni_pose["R_pose"].to_numpy(), c='b', s=5)

ax[1][0].scatter(aruco_pose["timestamp"].to_numpy(), aruco_pose["X_pose"].to_numpy(), c='b', s=5)
ax[1][1].scatter(aruco_pose["timestamp"].to_numpy(), aruco_pose["Y_pose"].to_numpy(), c='b', s=5)
ax[1][2].scatter(aruco_pose["timestamp"].to_numpy(), aruco_pose["Z_pose"].to_numpy(), c='b', s=5)
ax[1][3].scatter(aruco_pose["timestamp"].to_numpy(), aruco_pose["Y_pose"].to_numpy(), c='b', s=5)

ax[2][0].scatter(tello_odom["timestamp"].to_numpy(), tello_odom["X_pose"].to_numpy(), c='b', s=5)
ax[2][1].scatter(tello_odom["timestamp"].to_numpy(), tello_odom["Y_pose"].to_numpy(), c='b', s=5)
ax[2][2].scatter(tello_odom["timestamp"].to_numpy(), tello_odom["Z_pose"].to_numpy(), c='b', s=5)
# ax[2][3].scatter(tello_odom["timestamp"].to_numpy(), tello_odom["R_pose"].to_numpy(), c='b', s=5)
plt.show()
