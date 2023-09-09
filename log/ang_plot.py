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

def plot_grafico2d(ax, data):
    data = data.sort_values(by='Time')
    data["Time"] -= data["Time"][0]
    ax[0].plot(data["Time"].to_numpy(), c='b', label=f'Time')
    ax[1].plot(data["X Position"].to_numpy(), c='b', label=f'X Position')
    ax[2].plot(data["Y Position"].to_numpy(), c='b', label=f'Y Position')
    ax[3].plot(data["yaw"].to_numpy(), c='b', label=f'yaw')

    # ax[0].scatter(data.index, data["Time"].to_numpy(), c='b', label=f'tello')
    # ax[1].scatter(data.index, data["X Position"].to_numpy(), c='b', label=f'tello')
    # ax[2].scatter(data.index, data["Y Position"].to_numpy(), c='b', label=f'tello')
    # ax[3].scatter(data.index, data["yaw"].to_numpy(), c='b', label=f'tello')

    
    for a in ax.flat:
        a.set_xlabel('Time')
        a.legend()

    ax[0].set_ylabel('Time (rad)')
    ax[1].set_ylabel('X position (m)')
    ax[2].set_ylabel('Y position (m)')
    ax[3].set_ylabel('Yaw position (rad)')

controller = ['Cascade']
fig, ax = plt.subplots(4, 1, figsize=(15, 9), sharex='col', sharey='row')

# pose = ler_csv("log/csv/Cascade/gul1_2023-09-01-18-17-36.csv")
# pose = ler_csv("log/csv/Cascade/gulv2_2023-09-01-18-20-11.csv")
# pose = ler_csv("log/csv/Cascade/gulv3_2023-09-01-18-20-30.csv")
# pose = ler_csv("log/csv/Cascade/hd_2023-09-01-18-28-35.csv")
# pose = ler_csv("log/csv/Cascade/hd_2023-09-01-18-31-55.csv")
# pose = ler_csv("log/csv/Cascade/linev1_2023-09-01-18-14-12.csv")
# pose = ler_csv("log/csv/Cascade/linev2_2023-09-01-18-15-35.csv")
pose = ler_csv("log/csv/Cascade/linev3_2023-09-01-18-16-22.csv")
# pose = ler_csv("log/csv/Cascade/linev4_2023-09-01-18-16-56.csv")

# pose = ler_csv("log/csv/Parallel/hdv4_2023-09-01-18-57-54.csv")
# pose = ler_csv("log/csv/Parallel/linev1_2023-09-01-18-50-51.csv")
# pose = ler_csv("log/csv/Parallel/linev2_2023-09-01-18-51-59.csv")
# pose = ler_csv("log/csv/Parallel/linev3_2023-09-01-18-52-38.csv")
# pose = ler_csv("log/csv/Parallel/linev4_2023-09-01-18-53-08.csv")
# pose = ler_csv("log/csv/Parallel/ng1_2023-09-01-18-53-47.csv")
# pose = ler_csv("log/csv/Parallel/ng2_2023-09-01-18-54-11.csv")
# pose = ler_csv("log/csv/Parallel/ng3_2023-09-01-18-54-27.csv")

plot_grafico2d(ax, pose)

plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
