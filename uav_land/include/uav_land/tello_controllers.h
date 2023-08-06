#ifndef TELLO_CONTROLLERS_H
#define TELLO_CONTROLLERS_H

#include "pidArchitectures.h"

struct Pose {
    double x, y, z, theta;
};

struct Speed {
    double vx, vy, vz, vtheta;
};

class TelloPDController {
private:
    PDController controller_x;
    PDController controller_y;
    PDController controller_z;
    PDController controller_theta;

public:
    TelloPDController(){}
    
    TelloPDController(PID::Builder builder_x, PID::Builder builder_y, PID::Builder builder_z, PID::Builder builder_theta)
            : controller_x(builder_x),
              controller_y(builder_y),
              controller_z(builder_z),
              controller_theta(builder_theta) {}

    void update_x(double kp, double kd){controller_x.update(kp, kd);}
    void get_x(double &kp, double &kd){controller_x.getParameters(kp, kd);}

    void update_y(double kp, double kd){controller_y.update(kp, kd);}
    void get_y(double &kp, double &kd){controller_y.getParameters(kp, kd);}

    void update_z(double kp, double kd){controller_z.update(kp, kd);}
    void get_z(double &kp, double &kd){controller_z.getParameters(kp, kd);}

    void update_theta(double kp, double kd){controller_theta.update(kp, kd);}
    void get_theta(double &kp, double &kd){controller_theta.getParameters(kp, kd);}

    Speed control(const Pose& setpoint, const Pose& measurement) {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x);
        speed.vy = controller_y.control(setpoint.y, measurement.y);
        speed.vz = controller_z.control(setpoint.z, measurement.z);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta);
        return speed;
    }
};

class TelloCascadePDPI_FFController {
private:
    CascadePDPI_FFController controller_x;
    CascadePDPI_FFController controller_y;
    CascadePDPI_FFController controller_z;
    CascadePDPI_FFController controller_theta;

public:
    TelloCascadePDPI_FFController(PID::Builder builder_x_pd, PID::Builder builder_x_pi, PID::Builder builder_y_pd, PID::Builder builder_y_pi,
                                  PID::Builder builder_z_pd, PID::Builder builder_z_pi, PID::Builder builder_theta_pd, PID::Builder builder_theta_pi)
            : controller_x(builder_x_pd, builder_x_pi),
              controller_y(builder_y_pd, builder_y_pi),
              controller_z(builder_z_pd, builder_z_pi),
              controller_theta(builder_theta_pd, builder_theta_pi) {}

    Speed control(const Pose& setpoint, const Pose& measurement, const Speed& est_speed) {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x, est_speed.vx);
        speed.vy = controller_y.control(setpoint.y, measurement.y, est_speed.vy);
        speed.vz = controller_z.control(setpoint.z, measurement.z, est_speed.vz);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta, est_speed.vtheta);
        return speed;
    }
};

class TelloParallelPDPI_FFController {
private:
    ParallelPDPI_FFController controller_x;
    ParallelPDPI_FFController controller_y;
    ParallelPDPI_FFController controller_z;
    ParallelPDPI_FFController controller_theta;

public:
    TelloParallelPDPI_FFController(PID::Builder builder_x_pd, PID::Builder builder_x_pi, PID::Builder builder_x_ff,
                                   PID::Builder builder_y_pd, PID::Builder builder_y_pi, PID::Builder builder_y_ff,
                                   PID::Builder builder_z_pd, PID::Builder builder_z_pi, PID::Builder builder_z_ff,
                                   PID::Builder builder_theta_pd, PID::Builder builder_theta_pi, PID::Builder builder_theta_ff)
            : controller_x(builder_x_pd, builder_x_pi, builder_x_ff),
              controller_y(builder_y_pd, builder_y_pi, builder_y_ff),
              controller_z(builder_z_pd, builder_z_pi, builder_z_ff),
              controller_theta(builder_theta_pd, builder_theta_pi, builder_theta_ff) {}

    Speed control(const Pose& setpoint, const Pose& measurement) {
        Speed speed;
        speed.vx = controller_x.control(setpoint.x, measurement.x);
        speed.vy = controller_y.control(setpoint.y, measurement.y);
        speed.vz = controller_z.control(setpoint.z, measurement.z);
        speed.vtheta = controller_theta.control(setpoint.theta, measurement.theta);
        return speed;
    }
};

#endif //TELLO_CONTROLLERS_H
