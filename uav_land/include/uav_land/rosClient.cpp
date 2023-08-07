#include "rosClient.h"
#include "manager.h"

ROSClient::ROSClient(ros::NodeHandle *handle)
{
  this->nh = handle;
}

void ROSClient::init(Manager *const manager)
{
  pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/iris/pose", 10, &Manager::poseCallback, manager);
  odom_sub = nh->subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &Manager::odomCallback, manager);
  joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy_control", 1, &Manager::joyCallback, manager);
  parameters_sub = nh->subscribe<std_msgs::Float32MultiArray>("/PID/parameters", 1, &Manager::parametersCallback, manager);
}

void ROSClient::setParam(const std::string &key, double d)
{
  nh->setParam(key, d);
}
