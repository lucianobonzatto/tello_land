import os
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

class ImageReader:
    def __init__(self, image_folder):
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

        self.image_folder = image_folder
        self.data = pd.DataFrame(
            columns=[
                "Marker ID",
                "Tx (m)",
                "Ty (m)",
                "Tz (m)",
                "Rx (deg)",
                "Ry (deg)",
                "Rz (deg)",
            ]
        )

        self.fig, self.ax = plt.subplots(2, 2)

        self.colors = {272: 'r', 682: 'g', 0: 'b'}
        self.labels = {272: 'Marker 272', 682: 'Marker 682', 0: 'Marker 0'}

    def save_to_csv(self, filename):
        print("Saving data to:", filename)
        self.data.to_csv(filename, index=False)

    def process_images(self):
        # Get all image files from the folder
        image_files = [
            f for f in os.listdir(self.image_folder) if f.endswith((".png", ".jpg"))
        ]
        image_files = sorted(image_files)

        for image_file in image_files:
            image_path = os.path.join(self.image_folder, image_file)
            frame = cv2.imread(image_path)

            if frame is not None:
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

                        if rvecs is not None and tvecs is not None:
                            tvecs = np.squeeze(tvecs)
                            rvecs = np.squeeze(rvecs)
                            rotation_matrix_euler = R.from_rotvec(rvecs).as_euler('ZYX')
                            PCenterCameraFrame = self.LandpadFrameToCameraFrame(tvecs, rvecs, marker_id)

                            data_row = {
                                # "time": t,
                                "Marker ID": marker_id,
                                "Tx (m)": tvecs[1],
                                "Ty (m)": -tvecs[0],
                                "Tz (m)": tvecs[2],
                                "Rx (deg)": rotation_matrix_euler[0],
                                "Ry (deg)": rotation_matrix_euler[1],
                                "Rz (deg)": rotation_matrix_euler[2],
                                "Tx_cam": PCenterCameraFrame[0],
                                "Ty_cam": PCenterCameraFrame[1],
                                "Tz_cam": PCenterCameraFrame[2],
                            }
                            self.data = pd.concat([self.data, pd.DataFrame([data_row])], ignore_index=True)

                cv2.imshow("Processed Image", frame)
                self.update_plot()
                # cv2.waitKey(1)

    def update_plot(self):
        tx_values = self.data["Tx (m)"]
        ty_values = self.data["Ty (m)"]
        tz_values = self.data["Tz (m)"]
        tx_cam_values = self.data["Tx_cam"]
        ty_cam_values = self.data["Ty_cam"]
        tz_cam_values = self.data["Tz_cam"]
        marker_ids = self.data["Marker ID"]
        colors = [self.colors[id] for id in marker_ids]

        self.ax[0][0].scatter(tx_values, ty_values, c=colors)
        # self.ax[0][0].scatter(tx_cam_values, ty_cam_values, c=colors, marker='*')
        legend_elements = [Line2D([0], [0], marker='o', color='w', label=self.labels[id], markerfacecolor=color, markersize=10) for id, color in self.colors.items()]
        self.ax[0][0].legend(handles=legend_elements, loc='best')

        self.ax[0][1].set_title('Tx (m)')
        self.ax[0][1].scatter(range(len(tx_values)), tx_values, c=colors)
        self.ax[1][0].set_title('Ty (m)')
        self.ax[1][0].scatter(range(len(ty_values)), ty_values, c=colors)
        self.ax[1][1].set_title('Tz (m)')
        self.ax[1][1].scatter(range(len(tz_values)), tz_values, c=colors)

        plt.draw()
        plt.pause(0.001)
        
        self.ax[0][0].cla()
        self.ax[0][1].cla()
        self.ax[1][0].cla()
        self.ax[1][1].cla()

    def LandpadFrameToCameraFrame(self, Tvec, Rvec, id):
        if id not in [272, 682, 0]:
            return np.array([-999.0, -999.0, -999.0]), np.array([-999.0, -999.0, -999.0])
        
        # print("Tvec:", Tvec)
        # print("Rvec:", Rvec)

        r = R.from_rotvec(Rvec)
        TransformationMatrixCameraToAruco = np.eye(4)
        TransformationMatrixCameraToAruco[:3, :3] = r.as_matrix()
        TransformationMatrixCameraToAruco[:3, 3] = Tvec
        
        # print("\nTransformationMatrixCameraToAruco:")
        # print(TransformationMatrixCameraToAruco)

        PAr272 = np.array([0.320, 0.215, 0])
        PAr682 = np.array([0.043, 0.038, 0])
        PAr000 = np.array([-0.255, -0.160, 0])

        TransformationMatrixArucoToLandpad = np.eye(4)
        if id == 272:
            TransformationMatrixArucoToLandpad[0, 0] = -1
            TransformationMatrixArucoToLandpad[1, 1] = -1
            TransformationMatrixArucoToLandpad[:3, 3] = PAr272
        elif id == 682:
            TransformationMatrixArucoToLandpad[:3, 3] = PAr682
        elif id == 0:
            TransformationMatrixArucoToLandpad[0, 0] = -1
            TransformationMatrixArucoToLandpad[1, 1] = -1
            TransformationMatrixArucoToLandpad[:3, 3] = PAr000

        # print("\nTransformationMatrixArucoToLandpad:")
        # print(TransformationMatrixArucoToLandpad)

        TransformationMatrixCameraToLandpad = np.matmul(
            TransformationMatrixCameraToAruco,
            np.linalg.inv(TransformationMatrixArucoToLandpad),
        )

        # print("\nTransformationMatrixCameraToLandpad:")
        # print(TransformationMatrixCameraToLandpad)

        PCamera = np.array([Tvec[0], Tvec[1], Tvec[2], 1])
        PLandpadFrame = np.matmul(TransformationMatrixCameraToLandpad, PCamera)

        Delta = np.array([-999.0, -999.0, -999.0, 1])
        if id == 272:
            Delta[0] = PLandpadFrame[0] - PAr272[0]
            Delta[1] = PLandpadFrame[1] - PAr272[1]
            Delta[2] = 0
        elif id == 682:
            Delta[0] = PLandpadFrame[0] - PAr682[0]
            Delta[1] = PLandpadFrame[1] - PAr682[1]
            Delta[2] = 0
        elif id == 0:
            Delta[0] = PLandpadFrame[0] - PAr000[0]
            Delta[1] = PLandpadFrame[1] - PAr000[1]
            Delta[2] = 0

        PCenterCameraFrame = np.matmul(
            np.linalg.inv(TransformationMatrixCameraToLandpad), np.array(Delta)
        )
        # print("\nPCenterCameraFrame:")
        # print(PCenterCameraFrame)

        return PCenterCameraFrame[:3]


def main():
    app = ImageReader("/home/lukn23/Desktop/rgb/read")
    # app = ImageReader("/home/lukn23/Desktop/rgb/images")
    app.process_images()
    app.save_to_csv("dados_posicoes.csv")
    plt.waitforbuttonpress()


if __name__ == "__main__":
    main()
