import pandas as pd
import cv2
import rosbag
import cv2.aruco as aruco
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R


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
        self.data = pd.DataFrame(columns=["time", "Marker ID", "Tx (m)", "Ty (m)", "Tz (m)", "Rx (deg)", "Ry (deg)", "Rz (deg)"])

    def save_to_csv(self, filename):
        print("save: ", filename)
        self.data.to_csv(filename, index=False)

    def show_bag(self):
        print("bag name: ", self.bag_filename)
        try:
            with rosbag.Bag(self.bag_filename, "r") as bag:
                for topic, msg, t in bag.read_messages():
                    # print(t, end="\r")
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
                    (i, id[0]) for i, id in enumerate(ids) if id[0] in self.marker_sizes
                ]
                #############
                # frame = aruco.drawDetectedMarkers(frame, corners)
                #############

                for i, marker_id in ids_to_process:
                    marker_length = self.marker_sizes[marker_id]
                    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                        corners[i : i + 1],marker_length,
                        self.camera_matrix,self.distortion_coeffs,
                    )

                    if rvecs is not None and tvecs is not None:
                        #############
                        # frame = cv2.drawFrameAxes(
                        #     frame,self.camera_matrix,self.distortion_coeffs,
                        #     rvecs[0],tvecs[0],marker_length,
                        # )

                        # distance = np.linalg.norm(tvecs[0])
                        # text = f"ID {marker_id}: {distance:.2f} m"
                        # corner = tuple(corners[i][0][0].astype(int))
                        # cv2.putText(
                        #     frame,text,corner,cv2.FONT_HERSHEY_SIMPLEX,
                        #     0.5,(255, 0, 0),2,
                        # )
                        #############
                        tvecs = np.squeeze(tvecs)
                        rvecs = np.squeeze(rvecs)
                        rotation_matrix_euler = R.from_rotvec(rvecs).as_euler('ZYX')
                        data_row = {
                            "time": t,
                            "Marker ID": marker_id,
                            "Tx (m)": -tvecs[1],
                            "Ty (m)": -tvecs[0],
                            "Tz (m)": -tvecs[2],
                            "Rx (deg)": rotation_matrix_euler[0],
                            "Ry (deg)": rotation_matrix_euler[1],
                            "Rz (deg)": rotation_matrix_euler[2],
                        }
                        self.data = pd.concat([self.data, pd.DataFrame([data_row])], ignore_index=True)

                        result = self.LandpadFrameToCameraFrame(
                            [data_row["Tx (m)"],   data_row["Ty (m)"],   data_row["Tz (m)"], 
                             data_row["Rx (deg)"], data_row["Ry (deg)"], data_row["Rz (deg)"]],
                            data_row["Marker ID"],
                        )
                        print(result[2], data_row["Tz (m)"])

            cv2.imshow("Detected ArUco markers", frame)
            cv2.waitKey(1)

    def LandpadFrameToCameraFrame(self, P, id):

        if id not in [272, 682, 0]:
            return np.array([-999.0, -999.0, -999.0]), np.array([-999.0, -999.0, -999.0])

        Tvec = np.array([P[0], P[1], P[2]])
        Rvec = np.array([P[3], P[4], P[5]])
        r = R.from_rotvec(Rvec)

        TransformationMatrixCameraToAruco = np.eye(4)
        TransformationMatrixCameraToAruco[:3, :3] = r.as_matrix()
        TransformationMatrixCameraToAruco[:3, 3] = Tvec

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

        TransformationMatrixCameraToLandpad = np.matmul(
            TransformationMatrixCameraToAruco,
            np.linalg.inv(TransformationMatrixArucoToLandpad),
        )

        PCamera = np.array([P[0], P[1], P[2], 1])
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

        return PCenterCameraFrame[:3]

def main():
    app = BagReader("/home/lukn23/Desktop/rgb/teste.bag")
    app.show_bag()
    app.save_to_csv('dados_posicoes.csv')

if __name__ == "__main__":
    main()
