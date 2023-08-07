#ifndef MANAGER_H
#define MANAGER_H

#include "general.h"
#include "state_machine.h"
#include "follow_controller.h"
#include "land_controller.h"
#include "rosClient.h"

class Manager
{
public:
    Manager();
    ~Manager();

    void Init(ROSClient *drone_control,
              double joyLinearVelocity,
              double joyAngularVelocity);

    void print_parameters();
    void update();

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void parametersCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

private:
    geometry_msgs::PoseStamped pose;
    sensor_msgs::Joy joy;
    nav_msgs::Odometry odom;
    std_msgs::Float32MultiArray parameters;

    ROSClient *ROS_client;
    State_Machine state_machine;
    Follow_Controller follow_controller;
    Land_Controller land_controller;

    void STOPPED_action();
    void TAKE_OFF_action();
    void LAND_action();
    void JOY_CONTROL_action();
    void LAND_CONTROL_action();
    void FOLLOW_CONTROL_action();
    void send_velocity(double x_linear, double y_linear, double z_linear, double angular);
    void send_velocity(geometry_msgs::Twist velocity);

    // joy parameters
    ros::Time joy_last_timestamp;
    double joy_linear_velocity;
    double joy_angular_velocity;
};

#endif // MANAGER_H
