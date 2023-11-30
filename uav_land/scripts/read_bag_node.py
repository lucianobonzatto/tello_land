import os
import csv
import glob
import rospy
import rosbag
import rospkg
import datetime
import subprocess
import tkinter as tk
from tkinter import Label, Entry, Button, Radiobutton, StringVar
from uav_land.msg import controllers_gain


class BagReader:
    def __init__(self):
        controllers = ["Cascade", "Parallel", "PD"]
        # controllers = ["Gains", "Gains/aruco", "PD"]

        self.csv_headers = [
            "Time",
            "X_vel_uav",
            "Y_vel_uav",
            "Z_vel_uav",
            "R_vel_uav",
            "X_cmd_vel",
            "Y_cmd_vel",
            "Z_cmd_vel",
            "R_cmd_vel",
        ]
        self.dir_name = "/media/lukn23/bgs/bag/tello"
        self.bags = []
        self.index = -1

        for value in controllers:
            image_folder = os.path.join(self.dir_name, value)
            data = glob.glob(image_folder + "/*.bag")
            print(image_folder, ": ", data.__len__(), " bags")

            for bag in data:
                self.bags.append(bag)

        print("total: ", self.bags.__len__(), " bags")

    def next_bag(self):
        self.index = self.index + 1
        if self.index >= self.bags.__len__():
            return -1

        bag_filename = self.bags[self.index]
        print(self.index, "\t-> ", bag_filename)

        try:
            print(bag_filename)
            bag = rosbag.Bag(bag_filename, "r")
            topics = bag.get_type_and_topic_info().topics.keys()
            for topic, msg, t in bag.read_messages():
                print(t, end="\r")
                self.topic_treatment(topic, msg)
                if rospy.is_shutdown():
                    bag.close()
                    return -1
            bag.close()

        except rosbag.ROSBagException as e:
            rospy.logerr("Erro ao reproduzir o arquivo de bag: %s", str(e))

    def show_bags(self):
        while not rospy.is_shutdown():
            print("-----")

            self.out_value = {
                "Time": [],
                "X_vel_uav": [],
                "Y_vel_uav": [],
                "Z_vel_uav": [],
                "R_vel_uav": [],
                "X_cmd_vel": [],
                "Y_cmd_vel": [],
                "Z_cmd_vel": [],
                "R_cmd_vel": [],
            }

            self.x_cmd_vel = 0
            self.y_cmd_vel = 0
            self.z_cmd_vel = 0
            self.R_cmd_vel = 0

            if self.next_bag() == -1:
                break
            self.save_csv_uav_pose()

    def topic_treatment(self, topic, msg):
        if topic == "/tello/odom":
            # rospy.loginfo("Reproduzindo mensagem em %s", topic)
            self.out_value["X_vel_uav"].append(msg.twist.twist.linear.x)
            self.out_value["Y_vel_uav"].append(msg.twist.twist.linear.y)
            self.out_value["Z_vel_uav"].append(msg.twist.twist.linear.z)
            self.out_value["R_vel_uav"].append(msg.twist.twist.angular.z)
            self.out_value["Time"].append(msg.header.stamp.to_sec())

            self.out_value["X_cmd_vel"].append(self.x_cmd_vel)
            self.out_value["Y_cmd_vel"].append(self.y_cmd_vel)
            self.out_value["Z_cmd_vel"].append(self.z_cmd_vel)
            self.out_value["R_cmd_vel"].append(self.R_cmd_vel)

        # elif topic == "/aruco/pose":
        #     # rospy.loginfo("Reproduzindo mensagem em %s", topic)
        #     self.out_value["Time"].append(msg.header.stamp.to_sec())
        #     self.out_value["X_vel_uav"].append(msg.pose.position.x)
        #     self.out_value["Y_vel_uav"].append(msg.pose.position.y)
        #     self.out_value["Z_vel_uav"].append(msg.pose.position.z)
        #     self.out_value["R_vel_uav"].append(0)
        
        #     self.out_value["X_cmd_vel"].append(self.x_cmd_vel)
        #     self.out_value["Y_cmd_vel"].append(self.y_cmd_vel)
        #     self.out_value["Z_cmd_vel"].append(self.z_cmd_vel)
        #     self.out_value["R_cmd_vel"].append(self.R_cmd_vel)

        elif topic == "/tello/cmd_vel":
            # rospy.loginfo("Reproduzindo mensagem em %s", topic)
            self.x_cmd_vel = msg.linear.x
            self.y_cmd_vel = msg.linear.y
            self.z_cmd_vel = msg.linear.z
            self.R_cmd_vel = msg.angular.z

    def save_csv_uav_pose(self):
        filename = self.bags[self.index]
        filename = filename.replace(self.dir_name, "~/tello_ws/src/uav_land/log/csv")
        filename = filename.replace("bag", "csv")
        filename = os.path.expanduser(filename)
        size = len(self.out_value["Time"])

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.csv_headers)
            for i in range(size):
                writer.writerow(
                    [
                        self.out_value["Time"][i],
                        self.out_value["X_vel_uav"][i],
                        self.out_value["Y_vel_uav"][i],
                        self.out_value["Z_vel_uav"][i],
                        self.out_value["R_vel_uav"][i],
                        self.out_value["X_cmd_vel"][i],
                        self.out_value["Y_cmd_vel"][i],
                        self.out_value["Z_cmd_vel"][i],
                        self.out_value["R_cmd_vel"][i],
                    ]
                )

        print(f'\n{size} dados salvos em "{filename}"')

def main():
    rospy.init_node("read_bag_node", anonymous=True)
    app = BagReader()
    app.show_bags()

if __name__ == "__main__":
    main()
