#ifndef CONFIG
#define CONFIG

#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
// #include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Joy.h>

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

#endif