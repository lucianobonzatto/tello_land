#ifndef PIDARCHITECTURES_H
#define PIDARCHITECTURES_H

#include "PIDlib.h"

class PDController {
private:
    PID pdController;

public:
    PDController() {}
    PDController(PID::Builder builder) : pdController(builder.build()) {}

    void update(double kp, double kd)
    {
        pdController.setKp(kp);
        pdController.setKd(kd);
    }

    void getParameters(double &kp, double &kd)
    {
        kp = pdController.getKp();
        kd = pdController.getKd();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
    }

    double control(double setpoint, double measurement) {
        pdController.compute(setpoint, measurement);
        return pdController.getOutput();
    }
};

class CascadePDPI_FFController {
private:
    PID pdController;
    PID piController;

public:
    CascadePDPI_FFController(PID::Builder builder_pd, PID::Builder builder_pi)
            : pdController(builder_pd.build()),
              piController(builder_pi.build()) {}

    double control(double setpoint, double pd_measurement, double pi_measurement) {
        pdController.compute(setpoint, pd_measurement);
        double pd_output = pdController.getOutput();
        piController.compute(pd_output, pi_measurement);
        return piController.getOutput();
    }
};

class ParallelPDPI_FFController {
private:
    PID pdController;
    PID piController;
    PID ffController;

public:
    ParallelPDPI_FFController(PID::Builder builder_pd, PID::Builder builder_pi, PID::Builder builder_ff)
            : pdController(builder_pd.build()),
              piController(builder_pi.build()),
              ffController(builder_ff.build()) {}

    double control(double setpoint, double measurement) {
        pdController.compute(setpoint, measurement);
        double pd_output = pdController.getOutput();
        piController.compute(setpoint, measurement);
        double pi_output = piController.getOutput();
        ffController.compute(setpoint, measurement);
        double ff_output = ffController.getOutput();
        return pd_output + pi_output + ff_output;
    }
};

#endif //PIDARCHITECTURES_H