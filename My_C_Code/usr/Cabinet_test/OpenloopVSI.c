

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



void OpenLoop_VSI(OpenLoop_Command *OpenLoop)
{
    // Update theta
    double omega;
    omega = 2.0*PI*OpenLoop->freq;
    theta += (Ts * omega);
    theta = fmod(theta, 2.0 * PI); // Wrap to 2*pi

    // Calculate desired duty ratios
    OpenLoop->command_volatge[0] = OpenLoop->amp * cos(theta);
    OpenLoop->command_volatge[1] = OpenLoop->amp * cos(theta + 2.0 * PI / 3.0);
    OpenLoop->command_volatge[2] = OpenLoop->amp * cos(theta + 4.0 * PI / 3.0);

    // Update logging variables
    LOG_va = (double) (OpenLoop->command_volatge[0]);
    LOG_vb = (double) (OpenLoop->command_volatge[1] );
    LOG_vc = (double) (OpenLoop->command_volatge[2] );
}

OpenLoop_Command *init_OpenLoop_Command(void)
{
    // Update theta
    VSI_Openloop_command.enable = 0;
    VSI_Openloop_command.Num_inv = -1;
    VSI_Openloop_command.freq = 0;
    VSI_Openloop_command.amp = 0;
    return &VSI_Openloop_command;
}


