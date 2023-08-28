#ifndef LAND_CONTROLLER_H
#define LAND_CONTROLLER_H

#include "general.h"
#include "tello_controllers.h"

class Land_Controller
{
public:
    Land_Controller();
    ~Land_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped, Speed drone_vel);
    void update_parameters(uav_land::controllers_gain newParameters);

private:
    TelloPDController pdController;
    TelloCascadePDPIController cascadeController;
    TelloParallelPDPIController parallelController;
    TelloPIDController pidController;
    Pose setpoint;
    int controller_mode;

    Speed get_align_velocity(Pose poseStamped, Speed drone_vel);
};

#endif // LAND_CONTROLLER_H
