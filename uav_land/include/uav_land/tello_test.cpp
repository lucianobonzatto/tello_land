#include "Tello/tello_controllers.h"

int main() {
    // Create builders and configure PID parameters
    PID::Builder builder_pd_x;
    builder_pd_x.setKp(1.0);
    PID::Builder builder_pd_y;
    builder_pd_y.setKp(0.5);
    PID::Builder builder_pd_z;
    builder_pd_z.setKp(0.2);
    PID::Builder builder_pd_theta;
    builder_pd_theta.setKp(0.3);

    PID::Builder builder_cascade_pd_x;
    builder_cascade_pd_x.setKp(1.0);
    PID::Builder builder_cascade_pi_x;
    builder_cascade_pi_x.setKi(0.5);
    PID::Builder builder_cascade_pd_y;
    builder_cascade_pd_y.setKp(0.2);
    PID::Builder builder_cascade_pi_y;
    builder_cascade_pi_y.setKi(0.3);
    PID::Builder builder_cascade_pd_z;
    builder_cascade_pd_z.setKp(0.4);
    PID::Builder builder_cascade_pi_z;
    builder_cascade_pi_z.setKi(0.6);
    PID::Builder builder_cascade_pd_theta;
    builder_cascade_pd_theta.setKp(0.7);
    PID::Builder builder_cascade_pi_theta;
    builder_cascade_pi_theta.setKi(0.8);

    PID::Builder builder_parallel_pd_x;
    builder_parallel_pd_x.setKp(1.0);
    PID::Builder builder_parallel_pi_x;
    builder_parallel_pi_x.setKi(0.5);
    PID::Builder builder_parallel_ff_x;
    builder_parallel_ff_x.setKf(0.2);
    PID::Builder builder_parallel_pd_y;
    builder_parallel_pd_y.setKp(0.3);
    PID::Builder builder_parallel_pi_y;
    builder_parallel_pi_y.setKi(0.4);
    PID::Builder builder_parallel_ff_y;
    builder_parallel_ff_y.setKf(0.6);
    PID::Builder builder_parallel_pd_z;
    builder_parallel_pd_z.setKp(0.7);
    PID::Builder builder_parallel_pi_z;
    builder_parallel_pi_z.setKi(0.8);
    PID::Builder builder_parallel_ff_z;
    builder_parallel_ff_z.setKf(0.9);
    PID::Builder builder_parallel_pd_theta;
    builder_parallel_pd_theta.setKp(0.1);
    PID::Builder builder_parallel_pi_theta;
    builder_parallel_pi_theta.setKi(0.2);
    PID::Builder builder_parallel_ff_theta;
    builder_parallel_ff_theta.setKf(0.3);

    // Create controllers with configured PID parameters
    TelloPDController pdController(
            builder_pd_x,
            builder_pd_y,
            builder_pd_z,
            builder_pd_theta
    );

    TelloCascadePDPI_FFController cascadePDPI_FFController(
            builder_cascade_pd_x,
            builder_cascade_pi_x,
            builder_cascade_pd_y,
            builder_cascade_pi_y,
            builder_cascade_pd_z,
            builder_cascade_pi_z,
            builder_cascade_pd_theta,
            builder_cascade_pi_theta
    );

    TelloParallelPDPI_FFController parallelPDPI_FFController(
            builder_parallel_pd_x,
            builder_parallel_pi_x,
            builder_parallel_ff_x,
            builder_parallel_pd_y,
            builder_parallel_pi_y,
            builder_parallel_ff_y,
            builder_parallel_pd_z,
            builder_parallel_pi_z,
            builder_parallel_ff_z,
            builder_parallel_pd_theta,
            builder_parallel_pi_theta,
            builder_parallel_ff_theta
    );

    // Create dummy pose and speed for testing
    Pose pose {0, 0, 0, 0};
    Speed speed {0, 0, 0, 0};

    // Use controllers
    Speed pdResult = pdController.control(pose, pose);
    Speed cascadeResult = cascadePDPI_FFController.control(pose, pose, speed);
    Speed parallelResult = parallelPDPI_FFController.control(pose, pose);

    return 0;
}
