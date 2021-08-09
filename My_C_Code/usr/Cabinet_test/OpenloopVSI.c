

#include "usr/Cabinet_test/OpenloopVSI.h"
#include "usr/Cabinet_test/definitions.h"
#include "sys/scheduler.h"

#include <math.h>
#include <stdint.h>

// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

// Example logging variables for testing
double LOG_va = 0;
double LOG_vb = 0;
double LOG_vc = 0;

static double Ts = 1.0 / 10000.0; // [sec]
static double theta = 0.0;        // [rad]



void OpenLoop_VSI(double freq, double amp, double *command_volatge)
{
    // Update theta
    double omega;
    omega = 2.0*PI*freq;
    theta += (Ts * omega);
    theta = fmod(theta, 2.0 * PI); // Wrap to 2*pi

    // Calculate desired duty ratios
    command_volatge[0] = amp * cos(theta);
    command_volatge[1] = amp * cos(theta + 2.0 * PI / 3.0);
    command_volatge[2] = amp * cos(theta + 4.0 * PI / 3.0);

    // Update logging variables
    LOG_vsi_a = (double) (command_volatge[0]);
    LOG_vsi_b = (double) (command_volatge[1] );
    LOG_vsi_c = (double) (command_volatge[2] );
}


