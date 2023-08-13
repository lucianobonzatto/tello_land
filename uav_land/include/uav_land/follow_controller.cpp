#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1.5;
    setpoint.theta = 0;

    PID::Builder builder_pd_x;
    PID::Builder builder_pd_y;
    PID::Builder builder_pd_z;
    PID::Builder builder_pd_theta;

    TelloPDController controller(
        builder_pd_x,
        builder_pd_y,
        builder_pd_z,
        builder_pd_theta);

    pdController = controller;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    cout << "Follow_Controller: " << endl;

    double Kp, Kd;
    pdController.get_x(Kp, Kd);
    cout << "\tKp_x: " << Kp << "\tKd_x: " << Kd << endl;
    pdController.get_y(Kp, Kd);
    cout << "\tKp_y: " << Kp << "\tKd_y: " << Kd << endl;
    pdController.get_z(Kp, Kd);
    cout << "\tKp_z: " << Kp << "\tKd_z: " << Kd << endl;
    pdController.get_theta(Kp, Kd);
    cout << "\tKp_theta: " << Kp << "\tKd_theta: " << Kd << endl;
}

void Follow_Controller::update_parameters(uav_land::controllers_gain newParameters)
{
    pdController.update_x(newParameters.pd_ctrl.x.p_gain, newParameters.pd_ctrl.x.d_gain);
    pdController.update_y(newParameters.pd_ctrl.y.p_gain, newParameters.pd_ctrl.y.d_gain);
    pdController.update_z(newParameters.pd_ctrl.z.p_gain, newParameters.pd_ctrl.z.d_gain);
    pdController.update_theta(newParameters.pd_ctrl.yaw.p_gain, newParameters.pd_ctrl.yaw.d_gain);
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped)
{
    geometry_msgs::Twist velocity;
    geometry_msgs::Pose pose = poseStamped.pose;

    if ((pose.position.x == 0) &&
        (pose.position.y == 0) &&
        (pose.position.z == 0) &&
        (pose.orientation.x == 0) &&
        (pose.orientation.y == 0) &&
        (pose.orientation.z == 0) &&
        (pose.orientation.w == 0))
    {
        return velocity;
    }

    Pose measurement;
    measurement.x = pose.position.x;
    measurement.y = pose.position.y;
    measurement.z = pose.position.z;
    measurement.theta = pose.orientation.x;

    Speed vel = pdController.control(setpoint, measurement);

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;

    // cout << "x -> " << setpoint.x << "\t" << pose.position.x << "\t" << velocity.linear.x << endl;
    // cout << "y -> " << setpoint.y << "\t" << pose.position.y << "\t" << velocity.linear.y << endl;
    // cout << "z -> " << setpoint.z << "\t" << pose.position.z << "\t" << velocity.linear.z << endl;
    // cout << "theta -> " << setpoint.theta << "\t" << pose.position.z << "\t" << velocity.linear.z << endl;

    return velocity;
}
