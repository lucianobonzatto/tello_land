#!/usr/bin/env python

import rospy
import socket
import struct
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

def odom_callback(odom_msg):
    linear = odom_msg.twist.twist.linear
    angular = odom_msg.twist.twist.angular
    velocity = [linear.x, linear.y, linear.z, angular.z]
    velocity_bytes = struct.pack('ffff', *velocity)

    print(velocity)
    print(velocity_bytes)
    udp_socket.sendto(velocity_bytes, (udp_ip, udp_port))

if __name__ == '__main__':
    rospy.init_node('odom_udp_sender')

    udp_ip = "127.0.0.1"  # Endereço IP do destino UDP
    udp_port = 9001      # Porta UDP do destino

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.spin()

    udp_socket.close()
