#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;
    controller_mode = 0;

    PID::Builder builder;
    builder.setDt(0.05);

    TelloPDController pd_controller(
        builder,
        builder,
        builder,
        builder);
    pdController = pd_controller;

    TelloCascadePDPIController cascade_controller(
        builder, builder,
        builder, builder,
        builder, builder,
        builder, builder);
    cascadeController = cascade_controller;

    TelloParallelPDPIController parallel_controller(
        builder, builder,
        builder, builder,
        builder, builder,
        builder, builder);
    parallelController = parallel_controller;

    TelloPIDController pid_Controller(
        builder,
        builder,
        builder,
        builder);
    pidController = pidController;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    cout << "Follow_Controller: " << endl;
    cout << "\tx: " << setpoint.x << "\ty: " << setpoint.y
         << "\tz: " << setpoint.z << "\ttheta: " << setpoint.theta << endl;

    if (controller_mode == CONTROLERS::_PD)
    {
        cout << "\tPD" << endl;
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
    else if (controller_mode == CONTROLERS::_PID)
    {
        cout << "\tPID" << endl;
        double kp, ki, kd;
        kp = ki = kd = 0;
        pidController.get_x(kp, ki, kd);
        cout << "\tx_kp: " << kp << "\tx_ki: " << ki << "\tx_kd: " << kd << endl;

        pidController.get_y(kp, ki, kd);
        cout << "\ty_kp: " << kp << "\ty_ki: " << ki << "\ty_kd: " << kd << endl;

        pidController.get_z(kp, ki, kd);
        cout << "\tz_kp: " << kp << "\tz_ki: " << ki << "\tz_kd: " << kd << endl;

        pidController.get_theta(kp, ki, kd);
        cout << "\tt_kp: " << kp << "\tt_ki: " << ki << "\tt_kd: " << kd << endl;
    }
    else if (controller_mode == CONTROLERS::_CASCADE)
    {
        cout << "\tCascade" << endl;
        double kp_pd, kd_pd, kp_pi, ki_pi;
        kp_pd = kd_pd = kp_pi = ki_pi = 0;
        cascadeController.get_x(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tx_p_pd: " << kp_pd << "\tx_d_pd: " << kd_pd
             << "\tx_p_pi: " << kp_pi << "\tx_i_pi: " << ki_pi << endl;

        cascadeController.get_y(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\ty_p_pd: " << kp_pd << "\ty_d_pd: " << kd_pd
             << "\ty_p_pi: " << kp_pi << "\ty_i_pi: " << ki_pi << endl;

        cascadeController.get_z(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tz_p_pd: " << kp_pd << "\tz_d_pd: " << kd_pd
             << "\tz_p_pi: " << kp_pi << "\tz_i_pi: " << ki_pi << endl;

        cascadeController.get_theta(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tt_p_pd: " << kp_pd << "\tt_d_pd: " << kd_pd
             << "\tt_p_pi: " << kp_pi << "\tt_i_pi: " << ki_pi << endl;
    }
    else if (controller_mode == CONTROLERS::_PARALLEL)
    {
        cout << "\tParallel" << endl;
        double kp_pd, kd_pd, kp_pi, ki_pi;
        kp_pd = kd_pd = kp_pi = ki_pi = 0;
        parallelController.get_x(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tx_p_pd: " << kp_pd << "\tx_d_pd: " << kd_pd
             << "\tx_p_pi: " << kp_pi << "\tx_i_pi: " << ki_pi << endl;

        parallelController.get_y(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\ty_p_pd: " << kp_pd << "\ty_d_pd: " << kd_pd
             << "\ty_p_pi: " << kp_pi << "\ty_i_pi: " << ki_pi << endl;

        parallelController.get_z(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tz_p_pd: " << kp_pd << "\tz_d_pd: " << kd_pd
             << "\tz_p_pi: " << kp_pi << "\tz_i_pi: " << ki_pi << endl;

        parallelController.get_theta(kp_pd, kd_pd, kp_pi, ki_pi);
        cout << "\tt_p_pd: " << kp_pd << "\tt_d_pd: " << kd_pd
             << "\tt_p_pi: " << kp_pi << "\tt_i_pi: " << ki_pi << endl;
    }
    else
    {
        cout << "\t*****" << endl;
    }
}

void Follow_Controller::update_parameters(uav_land::controllers_gain newParameters)
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
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped, Speed drone_vel)
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

    Speed vel;
    if (controller_mode == CONTROLERS::_PD)
    {
        vel = pdController.control(setpoint, measurement);
    }
    else if (controller_mode == CONTROLERS::_PID)
    {
        vel = pidController.control(setpoint, measurement);
    }
    else if (controller_mode == CONTROLERS::_CASCADE)
    {
        vel = cascadeController.control(setpoint, measurement, drone_vel);
    }
    else if (controller_mode == CONTROLERS::_PARALLEL)
    {
        Speed vel_setpoint;

        vel_setpoint.vx = calc_vel(measurement.x - setpoint.x);
        vel_setpoint.vy = calc_vel(measurement.y - setpoint.y);
        vel_setpoint.vz = calc_vel(measurement.z - setpoint.z);
        vel_setpoint.vtheta = calc_vel(measurement.theta - setpoint.theta);
        cout << "measurement: (" << measurement.x << ", " << measurement.y << ", " << measurement.z << ", " << measurement.theta << ")" << endl;
        cout << "vel_setpoint: (" << vel_setpoint.vx << ", " << vel_setpoint.vy << ", " << vel_setpoint.vz << ", " << vel_setpoint.vtheta << ")" << endl;

        Speed vel = parallelController.control(setpoint, measurement, vel_setpoint, drone_vel);
    }
    else
    {
        cout << "xxxxxxxxxxxxxxxxxxxx" << endl;
        vel.vx = 0;
        vel.vy = 0;
        vel.vz = 0;
        vel.vtheta = 0;
    }

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;

    return velocity;
}

double Follow_Controller::calc_vel(double valor_in)
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