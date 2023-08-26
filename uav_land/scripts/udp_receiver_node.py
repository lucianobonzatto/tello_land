#!/usr/bin/env python

import rospy
import socket
import struct
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

def publish_odom(udp_data):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.twist.twist.linear.x = udp_data[0]
    odom_msg.twist.twist.linear.y = udp_data[1]
    odom_msg.twist.twist.linear.z = udp_data[2]
    odom_msg.twist.twist.angular.z = udp_data[3]

    # Publicar a mensagem de odometria
    odom_pub.publish(odom_msg)

def udp_listener():
    udp_ip = "0.0.0.0"  # Escuta em todos os endereços de IP
    udp_port = 9001   # Porta UDP para escuta

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))
    udp_socket.settimeout(0.1)

    rospy.loginfo("UDP listener is now listening on {}:{}".format(udp_ip, udp_port))

    while not rospy.is_shutdown():
        try:
            udp_data, udp_addr = udp_socket.recvfrom(4096)
            pose_list = list(struct.unpack('ffff', udp_data))
            print(pose_list)
            publish_odom(pose_list)
        except socket.timeout:
            rospy.logwarn("Socket timeout occurred. No data received.")
        except Exception as e:
            rospy.logerr("An error occurred: {}".format(e))

    udp_socket.close()

if __name__ == '__main__':
    rospy.init_node('odom_udp_receiver')

    odom_pub = rospy.Publisher("/magni/odom", Odometry, queue_size=10)

    udp_listener()
