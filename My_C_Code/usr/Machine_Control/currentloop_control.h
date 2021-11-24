#ifndef CURRENTLOOP_CONTROL_H
#define CURRENTLOOP_CONTROL_H

#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/para_machine_c_loop.h"
//#include "usr/Machine_Control//BIM/bim_para_machine.h"
//#include "usr/Machine_Control/controllers.h"
#include <stdbool.h>


typedef enum {
    InvFour = 1,
    Inv_Series = 0,
} config;

typedef struct para_PI_discrete{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double tau_p;
    double Ap;
    double Bp;
    double wd;
    double tau_d;
    double Ad;
    double Bd;
    double state_1[3];
    double state_2[3];
    double state_3[3];

} para_PI_discrete;

typedef struct para_PR{

    //Controller Parameters Calculated from Bilinear Transform
    int enable;
    double Ts;
    double wd;
    double Kr;
    double Kp;
    double state_1[3];
    double state_2[3];
    double v_PR[3];

} para_PR;

typedef struct c_loop_threephase_data{
    int Num_inv;
    InverterThreePhase_t *inv;
    double Iabc[3];
    double vabc_ref[3];

} c_loop_threephase_data;

typedef struct c_loop_control_data{
    double error[3];
    double Iabc[3];
    double Idq0[3];
    double Idq0_m1[3];
    double Idq0_ref[3];
    double Idq0_ref_inject[3];
    double vdq0_ref[3];
    double vdq0_ref_inject[3];
    double vdq0_decouple[3];
    double theta_rad;
    double we;
    para_PI_discrete *PI_regulator;
    para_PR *PR_regulator;

}c_loop_control_data;

typedef struct currentloop_control{
    bool is_init;
    config sel_config;

    //inv1
    c_loop_threephase_data c_loop_inv1;
    //inv2
    c_loop_threephase_data c_loop_inv2;
    //inv3
    c_loop_threephase_data c_loop_inv3;
    //inv4
    c_loop_threephase_data c_loop_inv4;

    c_loop_control_data tq;
    c_loop_control_data s1;
    c_loop_control_data s2;
    c_loop_control_data tq2;

    para_c_loop_machine_control *para_machine;


} currentloop_control;


static para_PI_discrete PI_tq;
static para_PI_discrete PI_s1;
static para_PI_discrete PI_s2;
static para_PI_discrete PI_tq2;
extern currentloop_control c_loop_control;

currentloop_control *init_currentloop(void);
currentloop_control *deinit_currentloop(void);

void reset_regulator(void);
void exp_jtheta(double theta, double *in_dq, double*out_dq);
void current_regulation (currentloop_control *data);
void get_inverter_current_abc(c_loop_threephase_data *c_loop);
void get_all_inverter_current_abc(currentloop_control* data);
void cal_invI_to_controlI_configseries(currentloop_control* data);
void cal_invI_to_controlI_configInvFour(currentloop_control* data);
void update_control_current(c_loop_control_data *data);
void regulator_PI_current_dq(c_loop_control_data *data_ctrl, para_c_loop_machine_control_single para_m, int tq);
void decouple(currentloop_control *data);
void get_theta_we(currentloop_control *data);
void regulator_PR_current(c_loop_control_data *data_ctrl);
#endif// TWINBEARINGLESS_CONTROL_H
