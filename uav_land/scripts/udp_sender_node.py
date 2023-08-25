#!/usr/bin/env python

import rospy
import socket
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

def publish_odom(udp_data):
    odom_msg = Odometry()
    odom_msg.deserialize(udp_data)

    # Publicar a mensagem de odometria
    odom_pub.publish(odom_msg)

def udp_listener():
    udp_ip = "0.0.0.0"  # Escuta em todos os endereços de IP
    udp_port = 12345   # Porta UDP para escuta

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))

    rospy.loginfo("UDP listener is now listening on {}:{}".format(udp_ip, udp_port))

    while not rospy.is_shutdown():
        udp_data, udp_addr = udp_socket.recvfrom(4096)
        publish_odom(udp_data)

    udp_socket.close()

if __name__ == '__main__':
    rospy.init_node('udp_to_odom')

    odom_pub = rospy.Publisher("/received_odom", Odometry, queue_size=10)

    udp_listener()
