import numpy as np
import pandas as pd
from scipy.spatial.distance import euclidean
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

os.chdir(dir_path)


def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        file_path = os.path.join(dir_path, nome_arquivo)
        print(f'Trying to open file at: {file_path}')
        df = pd.read_csv(file_path)
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

def plot_grafico3d(ax, controller, vel):
    iris_pose = ler_csv("log/csv/" + controller + vel + "/iris_pose.csv")
    magni_pose = ler_csv("log/csv/" + controller + vel + "/magni_pose.csv")
    
    ax.plot(iris_pose["X Position"].to_numpy(), iris_pose["Y Position"].to_numpy(), iris_pose["Z Position"].to_numpy(), c='b', label=f'iris_pose')
    ax.plot(magni_pose["X Position"].to_numpy(), magni_pose["Y Position"].to_numpy(), magni_pose["Z Position"].to_numpy(), c='r', label=f'magni_pose')
    
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')

    ax.set_xlim(0, 4)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 2.1)
    
    ax.legend()

def distancia_entre_pontos(p1, p2):
    return euclidean(p1, p2)

velocitys = ["03", "04", "05"]
controllers = ['pd', 'cascade', 'paralel']

for controller in controllers:
    for velocity in velocitys:
        iris_pose = ler_csv("csv/line/" + controller + "/iris_pose.csv")
        magni_pose = ler_csv("csv/line/" + controller + "/magni_pose.csv")
        # iris_pose = ler_csv("csv/sqr/cascate/iris_pose.csv")
        # magni_pose = ler_csv("csv/sqr/cascate/magni_pose.csv")

        x = iris_pose["X Position"].to_numpy()
        y = iris_pose["Y Position"].to_numpy()
        iris_trajetoria = np.array([x, y]).T

        x = magni_pose["X Position"].to_numpy()
        y = magni_pose["Y Position"].to_numpy()
        magni_trajetoria = np.array([x, y]).T

        num_pontos = len(iris_trajetoria)
        
        interpolador = interp1d(np.linspace(0, 1, len(magni_trajetoria)), magni_trajetoria, axis=0)
        trajetoria_gostaria_interpolada = interpolador(np.linspace(0, 1, num_pontos))

        eqm = sum((distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) ** 2 for i in range(num_pontos))) / num_pontos
        eam = sum(distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) for i in range(num_pontos)) / num_pontos

        erro_maximo_x = max(abs(trajetoria_gostaria_interpolada[i][0] - iris_trajetoria[i][0]) for i in range(num_pontos))
        erro_maximo_y = max(abs(trajetoria_gostaria_interpolada[i][1] - iris_trajetoria[i][1]) for i in range(num_pontos))

        print(controller + " " + velocity +
            " - EQM: " + str(eqm) + 
            " - EAM: " + str(eam) + 
            " - erro_maximo_x: " + str(erro_maximo_x) + 
            " - erro_maximo_y: " + str(erro_maximo_y))
    print("")

controllers = ['pd', 'cascade', 'paralel']
for controller in controllers:
    # iris_pose = ler_csv("csv/line/" + controller + velocity + "/iris_pose.csv")
    # magni_pose = ler_csv("csv/line/" + controller + velocity + "/magni_pose.csv")
    iris_pose = ler_csv("csv/sqr/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("csv/sqr/" + controller + "/magni_pose.csv")

    x = iris_pose["X Position"].to_numpy()
    y = iris_pose["Y Position"].to_numpy()
    iris_trajetoria = np.array([x, y]).T

    x = magni_pose["X Position"].to_numpy()
    y = magni_pose["Y Position"].to_numpy()
    magni_trajetoria = np.array([x, y]).T

    num_pontos = len(iris_trajetoria)
    
    interpolador = interp1d(np.linspace(0, 1, len(magni_trajetoria)), magni_trajetoria, axis=0)
    trajetoria_gostaria_interpolada = interpolador(np.linspace(0, 1, num_pontos))

    eqm = sum((distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) ** 2 for i in range(num_pontos))) / num_pontos
    eam = sum(distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) for i in range(num_pontos)) / num_pontos

    erro_maximo_x = max(abs(trajetoria_gostaria_interpolada[i][0] - iris_trajetoria[i][0]) for i in range(num_pontos))
    erro_maximo_y = max(abs(trajetoria_gostaria_interpolada[i][1] - iris_trajetoria[i][1]) for i in range(num_pontos))

    print(controller +
        " - EQM: " + str(eqm) + 
        " - EAM: " + str(eam) + 
        " - erro_maximo_x: " + str(erro_maximo_x) + 
        " - erro_maximo_y: " + str(erro_maximo_y))

    plt.figure(figsize=(10, 5))
    plt.plot(trajetoria_gostaria_interpolada[:, 0], trajetoria_gostaria_interpolada[:, 1], label='Interpolated')
    plt.plot(iris_trajetoria[:, 0], iris_trajetoria[:, 1], label='Original')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.show()
