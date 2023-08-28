#include "land_controller.h"

Land_Controller::Land_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;
    controller_mode = 0;
}

Land_Controller::~Land_Controller()
{
}

void Land_Controller::print_parameters()
{
    cout << "Land_Controller: " << endl;
    // cout << "\ttrack: " << track.header.stamp << endl;
}

void Land_Controller::update_parameters(uav_land::controllers_gain newParameters)
{
}

geometry_msgs::Twist Land_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped, Speed drone_vel)
{
    geometry_msgs::Twist velocity;

    if (poseStamped.header.stamp.isZero())
    {
        return velocity;
    }

    Pose measurement;
    measurement.x = -poseStamped.pose.position.x;
    measurement.y = -poseStamped.pose.position.y;
    measurement.z = -poseStamped.pose.position.z;
    measurement.theta = poseStamped.pose.orientation.x;

    Speed vel = get_align_velocity(measurement, drone_vel);

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;
    return velocity;
}

Speed Land_Controller::get_align_velocity(Pose poseStamped, Speed drone_vel)
{
    Speed vel;
    vel.vx = 0;
    vel.vy = 0;
    vel.vz = 0;
    vel.vtheta = 0;

    return vel;
}
