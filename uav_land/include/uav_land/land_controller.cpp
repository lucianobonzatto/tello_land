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
    pdController.update_x(newParameters.pd_ctrl.x.p_gain, newParameters.pd_ctrl.x.d_gain);
    pdController.update_y(newParameters.pd_ctrl.y.p_gain, newParameters.pd_ctrl.y.d_gain);
    pdController.update_z(newParameters.pd_ctrl.z.p_gain, newParameters.pd_ctrl.z.d_gain);
    pdController.update_theta(newParameters.pd_ctrl.yaw.p_gain, newParameters.pd_ctrl.yaw.d_gain);

    cascadeController.update_x(newParameters.cascade_ctrl.x.pd_ctrl.p_gain,
                               newParameters.cascade_ctrl.x.pd_ctrl.d_gain,
                               newParameters.cascade_ctrl.x.pi_ctrl.p_gain,
                               newParameters.cascade_ctrl.x.pi_ctrl.i_gain);
    cascadeController.update_y(newParameters.cascade_ctrl.y.pd_ctrl.p_gain,
                               newParameters.cascade_ctrl.y.pd_ctrl.d_gain,
                               newParameters.cascade_ctrl.y.pi_ctrl.p_gain,
                               newParameters.cascade_ctrl.y.pi_ctrl.i_gain);
    cascadeController.update_z(newParameters.cascade_ctrl.z.pd_ctrl.p_gain,
                               newParameters.cascade_ctrl.z.pd_ctrl.d_gain,
                               newParameters.cascade_ctrl.z.pi_ctrl.p_gain,
                               newParameters.cascade_ctrl.z.pi_ctrl.i_gain);
    cascadeController.update_theta(newParameters.cascade_ctrl.yaw.pd_ctrl.p_gain,
                                   newParameters.cascade_ctrl.yaw.pd_ctrl.d_gain,
                                   newParameters.cascade_ctrl.yaw.pi_ctrl.p_gain,
                                   newParameters.cascade_ctrl.yaw.pi_ctrl.i_gain);

    parallelController.update_x(newParameters.paralel_ctrl.x.pd_ctrl.p_gain,
                                newParameters.paralel_ctrl.x.pd_ctrl.d_gain,
                                newParameters.paralel_ctrl.x.pi_ctrl.p_gain,
                                newParameters.paralel_ctrl.x.pi_ctrl.i_gain);
    parallelController.update_y(newParameters.paralel_ctrl.y.pd_ctrl.p_gain,
                                newParameters.paralel_ctrl.y.pd_ctrl.d_gain,
                                newParameters.paralel_ctrl.y.pi_ctrl.p_gain,
                                newParameters.paralel_ctrl.y.pi_ctrl.i_gain);
    parallelController.update_z(newParameters.paralel_ctrl.z.pd_ctrl.p_gain,
                                newParameters.paralel_ctrl.z.pd_ctrl.d_gain,
                                newParameters.paralel_ctrl.z.pi_ctrl.p_gain,
                                newParameters.paralel_ctrl.z.pi_ctrl.i_gain);
    parallelController.update_theta(newParameters.paralel_ctrl.yaw.pd_ctrl.p_gain,
                                    newParameters.paralel_ctrl.yaw.pd_ctrl.d_gain,
                                    newParameters.paralel_ctrl.yaw.pi_ctrl.p_gain,
                                    newParameters.paralel_ctrl.yaw.pi_ctrl.i_gain);

    controller_mode = newParameters.mode;
    setpoint.z = newParameters.altitude;
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

double Land_Controller::calc_vel(double valor_in)
{
    const double MAX = 1.5;    // Valor máximo permitido
    const double MIN = 0.1;    // Valor mínimo permitido
    double valorRetorno = 0.6; // Valor a ser retornado
    double return_value;
    double valor = valor_in;
    if (valor_in < 0)
        valor = -valor;

    if (valor >= MIN && valor <= MAX)
    {
        return_value = valorRetorno;
    }
    else if (valor < MIN)
    {
        double slope = valorRetorno / (MIN - 0.0); // Inclinação da reta
        double intercept = -slope * 0.0;           // Intercepto da reta
        return_value = slope * valor + intercept;
    }
    else if (valor > MAX)
    {
        if (valor == 2 * MAX)
        {
            return_value = 0.0;
        }
        else
        {
            double slope = -valorRetorno / (2 * MAX - MAX); // Inclinação da reta
            double intercept = -slope * MAX + valorRetorno; // Intercepto da reta
            return_value = slope * valor + intercept;
        }
    }
    else
    {
        // Trate outros casos se necessário
        return_value = valor;
    }
    if (return_value < 0)
        return_value = 0;

    if (valor_in > 0)
        return_value = -return_value;

    return return_value;
}