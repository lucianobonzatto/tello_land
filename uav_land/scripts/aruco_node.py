#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import cv2.aruco as aruco
from math import pi, sin, cos
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

class ImageRepublisher:
    def __init__(self):
        self.image_pub = rospy.Publisher('/iris/usb_cam/aruco', Image, queue_size=10)
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/iris/pose', PoseStamped, queue_size=10)

        self.camera_matrix = np.array([[277.191356, 0.        , 320.5],
                                       [0.        , 277.191356, 240.5],
                                       [0.        , 0.        , 1.   ]])

        self.distortion_coeffs = np.array([[0, 0, 0, 0, 0]])
        self.dictionary = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        markers, ids, _ = aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)

        if len(markers) > 0:
            ids = ids.flatten()
            image = aruco.drawDetectedMarkers(image, markers, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(markers, 0.12, self.camera_matrix, self.distortion_coeffs)
    
            for i in range(len(ids)):
                if (ids[i] != 0):
                    continue
          
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                rotation_matrix_euler = R.from_rotvec(rvec).as_euler('ZYX')
                

                teste = {'id': -1, 'position':[0,0,0], 'orientation':[0,0,0]}
                teste['id'] = ids[i]
                teste['position'] = tvec
                teste['orientation'] = np.degrees(rotation_matrix_euler)
                print(teste)

                pose_msg = PoseStamped()

                pose_msg.pose.position.x = tvec[1] + 0.438340129 * tvec[2]
                pose_msg.pose.position.y = tvec[0] + 0.581929754 * tvec[2]
                pose_msg.pose.position.z = tvec[2]

                pose_msg.pose.orientation.x = rotation_matrix_euler[0]
                pose_msg.pose.orientation.y = rotation_matrix_euler[1]
                pose_msg.pose.orientation.z = rotation_matrix_euler[2]

                self.pose_pub.publish(pose_msg)

        republished_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        self.image_pub.publish(republished_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('aruco_node')

    republisher = ImageRepublisher()
    republisher.run()
