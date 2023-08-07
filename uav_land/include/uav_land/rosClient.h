#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include "mavros_interface.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class MavrosInterface; // Forward declaration because of circular reference

class ROSClient
{
  public:
    ROSClient(ros::NodeHandle *handle);

    void init(MavrosInterface *const drone_control);

    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber global_pos_sub_;
    ros::Subscriber setpoint_pos_sub_;

    ros::Publisher global_setpoint_pos_pub_;
    ros::Publisher setpoint_pos_pub_;
//    ros::Publisher vision_pos_pub_;
    ros::Publisher velocity_pub;

    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient set_mode_client_;

    void setParam(const std::string &key, double d);

    bool avoidCollision_;

  private:
    ros::NodeHandle *nh_;
};

#endif /* ROS_CLIENT_H */