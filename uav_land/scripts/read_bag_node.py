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
    self.csv_headers = ['Time', 'X Position', 'Y Position', 'Z Position', 'yaw']
    self.x_uav = []
    self.y_uav = []
    self.z_uav = []
    self.yaw_uav = []
    self.timestamp_uav = []

    controllers = ["Cascade", "Parallel"]
    self.bags = []
    self.index = -1

    for value in controllers:
      image_folder = os.path.join("/home/lukn23/bag/tello", value)
      data = glob.glob(image_folder + "/*")
      print(image_folder, ": ", data.__len__(), " bags")

      for bag in data:
        self.bags.append(bag)

    print("total: ", self.bags.__len__(), " bags")
    
  def save_gains(self):
      file_name = self.package_path + "/parameters/gains.txt"
      print(file_name)

      with open(file_name, 'w') as file:
          for key, entry in self.entries.items():
              value = entry.get()
              file.write(f"{key}: {value}\n")
          file.write(f"controller_mode: {self.controller_mode.get()}\n")

  def next_bag(self):
    self.index = self.index + 1
    if(self.index >= self.bags.__len__()):
      return -1
    
    bag_filename = self.bags[self.index]
    print(self.index, "\t-> ", bag_filename)

    try:
      with rosbag.Bag(bag_filename, 'r') as bag:
        topics = bag.get_type_and_topic_info().topics.keys()
          
        for topic, msg, t in bag.read_messages():
          # print(t)
          print(t, end='\r')
          self.topic_treatment(topic, msg)
          if rospy.is_shutdown():
            bag.close()
            return -1

        bag.close()

    except rosbag.ROSBagException as e:
      rospy.logerr("Erro ao reproduzir o arquivo de bag: %s", str(e))

  def show_bags(self):
    self.x_uav = []
    self.y_uav = []
    self.z_uav = []
    self.yaw_uav = []
    self.timestamp_uav = []
    while not rospy.is_shutdown():
      if(self.next_bag() == -1):
        break

      self.save_csv_uav_pose()
      print("-----")

  def topic_treatment(self, topic, msg):
    if(topic == "/aruco/pose"):
      # rospy.loginfo("Reproduzindo mensagem em %s", topic)
      # print(msg)

      self.x_uav.append(msg.pose.position.x)
      self.y_uav.append(msg.pose.position.y)
      self.z_uav.append(msg.pose.position.z)
      self.yaw_uav.append(msg.pose.position.x)
      self.timestamp_uav.append(msg.header.stamp.to_sec())

  def save_csv_uav_pose(self):
    filename = self.bags[self.index]
    filename = filename.replace("/home/lukn23/bag/tello/", "")
    filename = filename.replace("bag", "csv")
    print(filename)

    with open(filename, 'w') as csvfile:
      writer = csv.writer(csvfile)
      writer.writerow(self.csv_headers)
      for i in range(len(self.x_uav)):
        writer.writerow([self.timestamp_uav[i],
                         self.x_uav[i],
                         self.y_uav[i],
                         self.z_uav[i],
                         self.yaw_uav[i]])
      

def main():
    rospy.init_node('read_bag_node', anonymous=True)
    app = BagReader()
    app.show_bags()

if __name__ == "__main__":
    main()
