#include "land_controller.h"

Land_Controller::Land_Controller()
{
}

Land_Controller::~Land_Controller()
{
}

void Land_Controller::print_parameters()
{
    cout << "Land_Controller: " << endl;
    // cout << "\ttrack: " << track.header.stamp << endl;
}

geometry_msgs::Twist Land_Controller::get_velocity()
{
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    // if (track.header.stamp.isZero())
    // {
    //     return velocity;
    // }
    // if (track_last_timestamp == track.header.stamp)
    // {
    //     return velocity;
    // }
    return velocity;
}
