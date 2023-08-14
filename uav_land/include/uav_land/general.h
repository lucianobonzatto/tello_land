#ifndef CONFIG
#define CONFIG

#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <ros/ros.h>
#include <cmath>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <tello_driver/TelloStatus.h>
#include <uav_land/controllers_gain.h>

using namespace std;

#define TAG_ID 0

enum JOY_BUTTONS
{
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START,
    LOGITECH,
    ANALOGIC_LEFT,
    ANALOGIC_RIGHT
};

enum JOY_AXES
{
    HORIZONTAL_ANALOGIC_LEFT,
    VERTICAL_ANALOGIC_LEFT,
    LT,
    HORIZONTAL_ANALOGIC_RIGHT,
    VERTICAL_ANALOGIC_RIGHT,
    RT,
    HORIZONTAL_ARROW,
    VERTICAL_ARROW
};

enum STATES
{
    STOPPED = 0,
    TAKE_OFF,
    LAND,
    JOY_CONTROL,
    LAND_CONTROL,
    FOLLOW_CONTROL
};

static std::string states_name[6] = {
    "STOPPED",
    "TAKE_OFF",
    "LAND",
    "JOY_CONTROL",
    "LAND_CONTROL",
    "FOLLOW_CONTROL"
};

enum CONTROLERS
{
    NENHUM = 0,
    PD,
    CASCADE,
    PARALLEL,
};

static std::string controlers_name[6] = {
    "NENHUM",
    "PD",
    "CASCADE",
    "PARALLEL"
};

#endif