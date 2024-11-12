import os
import rosbag
import numpy as np
import pandas as pd
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

class BagReader:
    def __init__(self, bag_filename):
        self.camera_matrix = np.array(
            [
                [3.02573320e03, 0.00000000e00, 1.02641519e03],
                [0.00000000e00, 2.98476190e03, 2.69918299e02],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.distortion_coeffs = np.array(
            [-0.31855945, -0.04039797, 0.00156687, 0.00949025, 0.09074052]
        )
        self.marker_sizes = {272: 0.15, 682: 0.08, 0: 0.25}
        self.dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.bag_filename = bag_filename
        self.data = pd.DataFrame(
            columns=[
                "time",
                "Marker ID",
                "Tx_landpad",
                "Ty_landpad",
                "Tz_landpad",
                "Roll_landpad",
                "Pitch_landpad",
                "Yaw_landpad",
            ]
        )

        self.fig, self.ax = plt.subplots()

        self.colors = {272: 'r', 682: 'g', 0: 'b'}
        self.labels = {272: 'Marker 272', 682: 'Marker 682', 0: 'Marker 0'}

        # Cria a matriz de transformação Landpad -> Aruco

        Position_272 = np.array([-0.255, -0.160, 0])
        Rotation_272 = np.array([
            [-1, 0, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0,-1, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0, 0, 1]    # Eixo Z permanece o mesmo
        ])

        Position_682 = np.array([0.043, 0.038, 0])
        Rotation_682 = np.eye(3)

        Position_000 = np.array([0.320, 0.215, 0])
        Rotation_000 = np.array([
            [-1, 0, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0,-1, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0, 0, 1]    # Eixo Z permanece o mesmo
        ])

        self.TM_Aruco_To_Landpad_272 = np.eye(4)
        self.TM_Aruco_To_Landpad_272[:3, :3] = Rotation_272
        self.TM_Aruco_To_Landpad_272[:3, 3] = Position_272

        self.TM_Aruco_To_Landpad_682 = np.eye(4)
        self.TM_Aruco_To_Landpad_682[:3, :3] = Rotation_682
        self.TM_Aruco_To_Landpad_682[:3, 3] = Position_682

        self.TM_Aruco_To_Landpad_000 = np.eye(4)
        self.TM_Aruco_To_Landpad_000[:3, :3] = Rotation_000
        self.TM_Aruco_To_Landpad_000[:3, 3] = Position_000

        self.TM_Landpad_To_Aruco_272 = np.linalg.inv(self.TM_Aruco_To_Landpad_272)
        self.TM_Landpad_To_Aruco_682 = np.linalg.inv(self.TM_Aruco_To_Landpad_682)
        self.TM_Landpad_To_Aruco_000 = np.linalg.inv(self.TM_Aruco_To_Landpad_000)

    def save_to_csv(self, filename):
        print("Saving data to:", filename)
        self.data.to_csv(filename, index=False)
        
    def show_bag(self):
        print("bag name: ", self.bag_filename)
        try:
            with rosbag.Bag(self.bag_filename, "r") as bag:
                for topic, msg, t in bag.read_messages():
                    self.topic_treatment(topic, msg, t)
                bag.close()

        except rosbag.ROSBagException as e:
            print("Erro ao reproduzir o arquivo de bag: %s", str(e))

    def topic_treatment(self, topic, msg, t):
        if topic == "/camera/image_raw":
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_points = aruco.detectMarkers(
                gray_frame, self.dictionary, parameters=self.parameters
            )

            if ids is not None:
                ids_to_process = [
                    (i, id[0])
                    for i, id in enumerate(ids)
                    if id[0] in self.marker_sizes
                ]

                for i, marker_id in ids_to_process:
                    marker_length = self.marker_sizes[marker_id]
                    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                        corners[i : i + 1],
                        marker_length,
                        self.camera_matrix,
                        self.distortion_coeffs,
                    )
                    #############
                    # frame = cv2.drawFrameAxes(frame,self.camera_matrix,self.distortion_coeffs,rvecs,tvecs,marker_length)
                    #############

                    if rvecs is not None and tvecs is not None:
                        tvecs = np.squeeze(tvecs)
                        rvecs = np.squeeze(rvecs)
                        pos_landpad_to_camera, rot_landpad_to_camera = self.landpad_to_camera(tvecs, rvecs, marker_id)

                        data_row = {
                            "time": t,
                            "Marker ID": marker_id,
                            "Tx_landpad": pos_landpad_to_camera[0],
                            "Ty_landpad": pos_landpad_to_camera[1],
                            "Tz_landpad": pos_landpad_to_camera[2],
                            "Roll_landpad": rot_landpad_to_camera[0],
                            "Pitch_landpad": rot_landpad_to_camera[1],
                            "Yaw_landpad": rot_landpad_to_camera[2],
                        }
                        self.data = pd.concat([self.data, pd.DataFrame([data_row])], ignore_index=True)

            #############
            cv2.imshow("Image", frame)
            self.update_plot()
            cv2.waitKey(1)
            #############

    def update_plot(self):
        Tx_landpad      = self.data["Tx_landpad"]
        Ty_landpad      = self.data["Ty_landpad"]
        Tz_landpad      = self.data["Tz_landpad"]
        Roll_landpad    = self.data["Roll_landpad"]
        Pitch_landpad   = self.data["Pitch_landpad"]
        Yaw_landpad     = self.data["Yaw_landpad"]
        marker_ids      = self.data["Marker ID"]
        colors = [self.colors[id] for id in marker_ids]

        self.ax.scatter(Tx_landpad, Ty_landpad, c=colors, marker='*')
        legend_elements = [Line2D([0], [0], marker='o', color='w', label=self.labels[id], markerfacecolor=color, markersize=10) for id, color in self.colors.items()]
        self.ax.legend(handles=legend_elements, loc='best')

        plt.draw()
        plt.pause(0.001)
        
        self.ax.cla()

    def landpad_to_camera(self, Tvec, Rvec, id):
        if id not in [272, 682, 0]:
            return np.array([0, 0, 0])
            # return np.array([-999.0, -999.0, -999.0])

        # Cria a matriz de transformação Aruco -> Câmera
        r = R.from_rotvec(Rvec)
        TM_Aruco_To_Camera = np.eye(4)
        TM_Aruco_To_Camera[:3, :3] = r.as_matrix()
        TM_Aruco_To_Camera[:3, 3] = Tvec

        # Cria a matriz de transformação Landpad -> Câmera
        TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_000
        if id == 272:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_272
        elif id == 682:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_682
        elif id == 0:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_000

        pos_landpad_to_camera = TM_Landpad_To_Camera[:3, 3]
        pos_landpad_to_camera[1] = -pos_landpad_to_camera[1]

        rotation_landpad_to_camera = R.from_matrix(TM_Landpad_To_Camera[:3, :3])
        rot_landpad_to_camera = rotation_landpad_to_camera.as_euler('ZYX', degrees=True) # roll, pitch, yaw

        return pos_landpad_to_camera, rot_landpad_to_camera


def main():
    app = BagReader("/home/lukn23/Desktop/rgb/teste.bag")
    app.show_bag()
    app.save_to_csv("dados_posicoes.csv")
    plt.waitforbuttonpress()

if __name__ == "__main__":
    main()
