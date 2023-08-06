#ifndef LAND_CONTROLLER_H
#define LAND_CONTROLLER_H

#include "general.h"

class Land_Controller
{
public:
    Land_Controller();
    ~Land_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity();

private:
    geometry_msgs::Twist velocity;

    ros::Time track_last_timestamp;
};

#endif // LAND_CONTROLLER_H
