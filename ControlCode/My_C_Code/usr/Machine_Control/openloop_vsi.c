

#include "usr/Machine_Control/openloop_vsi.h"
#include "usr/Machine_Control/definitions.h"
#include "sys/scheduler.h"
#include "drv/encoder.h"
#include "usr/Machine_Control/control_structure.h"
#include <math.h>
#include <stdint.h>

// Scheduler TCB which holds task "context"
static task_control_block_t tcb;

// Example logging variables for testing
double LOG_va = 0;
double LOG_vb = 0;
double LOG_vc = 0;
double LOG_theta_OL = 0;
double LOG_theta_OL_raw = 0;

const double Ts = 1.0/10000.0;

static double theta_vsi = 0.0;        // [rad]

OpenLoop_Command VSI_Openloop_command;

void OpenLoop_VSI(OpenLoop_Command *OpenLoop)
{
    // Update theta
    double omega;
    omega = 2.0*PI*OpenLoop->freq;
    theta_vsi += (Ts * omega);
    theta_vsi = fmod(theta_vsi, 2.0 * PI); // Wrap to 2*pi

    // Calculate desired duty ratios
    OpenLoop->command_volatge[0] = OpenLoop->amp * cos(theta_vsi);
    OpenLoop->command_volatge[1] = OpenLoop->amp * cos(theta_vsi - 2.0 * PI / 3.0);
    OpenLoop->command_volatge[2] = OpenLoop->amp * cos(theta_vsi - 4.0 * PI / 3.0);

    // Update logging variables
    /*LOG_va = (double) (OpenLoop->command_volatge[0]);
    LOG_vb = (double) (OpenLoop->command_volatge[1] );
    LOG_vc = (double) (OpenLoop->command_volatge[2] );
    LOG_theta_OL_raw = get_encoder_pos();*/
    //uint32_t theta_raw;
    //encoder_get_position(&theta_raw);
    //LOG_theta_OL_raw = (double) theta_raw;
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


