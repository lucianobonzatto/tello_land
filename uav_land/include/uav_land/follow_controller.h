#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include "general.h"
#include "tello_controllers.h"

class Follow_Controller
{
public:
    Follow_Controller();
    ~Follow_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped);    
    void update_parameters(uav_land::controllers_gain newParameters);

private:
    ros::Time track_last_timestamp;
    TelloPDController pdController;
    Pose setpoint;
};

#endif // FOLLOW_CONTROLLER_H
