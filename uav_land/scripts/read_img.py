import os  # Import for handling directories
import cv2
import rosbag
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge


class BagReader:
    def __init__(self, bag_filename, output_dir="output_images"):
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

        self.output_dir = output_dir  # Define the output directory
        os.makedirs(self.output_dir, exist_ok=True)  # Create output directory if it doesn't exist

    def show_bag(self):
        print("bag name: ", self.bag_filename)
        self.cont = 0
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
                self.cont = self.cont + 1

                if (self.cont % 10):
                    image_filename = os.path.join(self.output_dir, f"frame_{t}.png")
                    cv2.imwrite(image_filename, frame)
                    print(f"Saved {image_filename}")

            cv2.imshow("Image", frame)
            cv2.waitKey(1)


def main():
    app = BagReader("/home/lukn23/Desktop/rgb/teste.bag", "/home/lukn23/Desktop/rgb/images")
    app.show_bag()


if __name__ == "__main__":
    main()
