#include "general.h"
#include "manager.h"
#include "ros_client.h"
#include "mavros_interface.h"

Manager principal;

static void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  principal.set_pose(*msg);
}
static void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  principal.set_odom(*msg);
}
static void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  principal.set_joy(*msg);
}
static void parametersCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  principal.set_parameters(*msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle *nh = new ros::NodeHandle();
  ros::Rate loop_rate(20);

  ROSClient ros_client(nh);
  MavrosInterface drone_control(&ros_client);

  ros::Subscriber pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/iris/pose", 10, &poseCallback);
  ros::Subscriber odom_sub = nh->subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &odomCallback);
  ros::Subscriber joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy_control", 1, &joyCallback);
  ros::Subscriber parameters_sub = nh->subscribe<std_msgs::Float32MultiArray>("/PID/parameters", 1, &parametersCallback);

  principal.Init(&drone_control, 5, 1);

  while(ros::ok()){
    ros::spinOnce();
    principal.print_parameters();
    principal.update();
    
    loop_rate.sleep();
  }

  delete nh;
}