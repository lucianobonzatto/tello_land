#include "ros_client.h"

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <geographic_msgs/GeoPoseStamped.h>

ROSClient::ROSClient(ros::NodeHandle *handle)
{
  this->nh_ = handle;
  avoidCollision_ = false;
}

void ROSClient::init(MavrosInterface *const drone_control)
{
  state_sub_ = nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosInterface::state_cb, drone_control);
  extended_state_sub_ = nh_->subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &MavrosInterface::extended_state_cb, drone_control);
  local_pos_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MavrosInterface::local_position_cb, drone_control);
  global_pos_sub_ = nh_->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &MavrosInterface::global_position_cb, drone_control);

  global_setpoint_pos_pub_ = nh_->advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
  setpoint_pos_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
 // vision_pos_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  velocity_pub = nh_->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  arming_client_ = nh_->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  land_client_ = nh_->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  set_mode_client_ = nh_->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void ROSClient::setParam(const std::string &key, double d)
{
  nh_->setParam(key, d);
}
