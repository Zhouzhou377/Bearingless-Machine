
#include "usr/Cabinet_test/twinbearingless_control.h"
#include "usr/Cabinet_test/para_machine.h"
#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/analog_sensor.h"
#include "usr/Cabinet_test/mb_sensor.h"
#include "usr/Cabinet_test/controllers.h"
#include "usr/Cabinet_test/transforms.h"
#include "usr/Cabinet_test/definitions.h"
#include <math.h>
#include <stdbool.h>


#define INV1 (5)
#define INV2 (2)
#define INV3 (3)
#define TS (1/100000)
#define WD_TQ (2*PI*100)
#define WD_S1 (2*PI*100)
#define WD_S2 (2*PI*100)


para_PI_discrete PI_tq;
para_PI_discrete PI_s1;
para_PI_discrete PI_s2;
twinbearingless_control twin_control;

para_PI_discrete *init_PI_para(double Ts, para_PI_discrete PI_inv1, para_twinmachine_control_single *para_machine, double wd){
    PI_inv1.wd = wd;
    PI_inv1.Ts = Ts;
    PI_inv1.tau_p = para_machine->L/para_machine->R;
    PI_inv1.Ap = (double)exp(-PI_inv1.Ts/PI_inv1.tau_p);
    PI_inv1.Bp = 1 - PI_inv1.Ap;
    PI_inv1.tau_d = 1/PI_inv1.wd;
    PI_inv1.Ad = (double)exp(-PI_inv1.Ts/PI_inv1.tau_d);
    PI_inv1.Bd = 1 - PI_inv1.Ad;
    PI_inv1.state_1[0] = 0.0;
    PI_inv1.state_1[1] = 0.0;
    PI_inv1.state_1[2] = 0.0;
    PI_inv1.state_2[0] = 0.0;
    PI_inv1.state_2[1] = 0.0;
    PI_inv1.state_2[2] = 0.0;
    PI_inv1.state_3[0] = 0.0;
    PI_inv1.state_3[1] = 0.0;
    PI_inv1.state_3[2] = 0.0;
}


twinbearingless_control *init_twinbearingless(void){

    twin_control.is_init = 1;
    twin_control.twin_inv1.Num_inv = INV1;
    twin_control.twin_inv2.Num_inv = INV2;
    twin_control.twin_inv3.Num_inv = INV3;
    twin_control.twin_inv1.inv = get_three_phase_inverter(twin_control.twin_inv1.Num_inv);
    twin_control.twin_inv2.inv = get_three_phase_inverter(twin_control.twin_inv2.Num_inv);
    twin_control.twin_inv3.inv = get_three_phase_inverter(twin_control.twin_inv3.Num_inv);
    // init machine control parameters

    twin_control.para_machine = init_para_twinmachine_control(void);

    twin_control.tq.PI_regulator = init_PI_para(TS, PI_tq, &twin_control.para_machine->para_tq, WD_TQ);
    twin_control.s1.PI_regulator = init_PI_para(TS, PI_s1, &twin_control.para_machine->para_s1, WD_S1);
    twin_control.s2.PI_regulator = init_PI_para(TS, PI_s2, &twin_control.para_machine->para_s2, WD_S2);
    return &twin_control;
}

twinbearingless_control *deinit_twinbearingless(void){

    twin_control.is_init = 0;
    // init machine control parameters
    return &twin_control;
}


void get_inverter_current_abc(twin_threephase_data *twin){
    if (twin->inv->HW->sensor.enable){
        get_currents_three_phase_abc(twin->Iabc, twin->inv);
    }else{
        get_mb_currents_three_phase_abc(twin.Iabc, twin->inv);
    }
}

void get_all_inverter_current_abc(twinbearingless_control* data){
    get_inverter_current_abc(&(data->twin_inv1));
    get_inverter_current_abc(&(data->twin_inv2));
    get_inverter_current_abc(&(data->twin_inv3));
}

void cal_invI_to_controlI(twinbearingless_control* data){
    
    //torque current calculation
    data->tq.Iabc[0] = 2.0*data->twin_inv1.Iabc[0] + data->twin_inv2.Iabc[0] + data->twin_inv3.Iabc[0]; 
    data->tq.Iabc[1] = 2.0*data->twin_inv1.Iabc[1] + data->twin_inv2.Iabc[1] + data->twin_inv3.Iabc[1];
    data->tq.Iabc[2] = 2.0*data->twin_inv1.Iabc[2] + data->twin_inv2.Iabc[2] + data->twin_inv3.Iabc[2];

    // suspension 1 
    data->s1.Iabc[0] = -0.5*data->twin_inv2.Iabc[0];
    data->s1.Iabc[1] = -0.5*data->twin_inv2.Iabc[1];
    data->s1.Iabc[2] = -0.5*data->twin_inv2.Iabc[2];

    // suspension 2 
    data->s2.Iabc[0] = -0.5*data->twin_inv3.Iabc[0];
    data->s2.Iabc[1] = -0.5*data->twin_inv3.Iabc[1];
    data->s2.Iabc[2] = -0.5*data->twin_inv3.Iabc[2];
}



void update_control_current(twin_control_data *data){
    data->Idq0_m1[0] = data->Idq0[0];
    data->Idq0_m1[1] = data->Idq0[1];
    data->Idq0_m1[2] = data->Idq0[2];

    abc_to_dq0(data->Iabc, data->Idq0, data->theta_rad);

}

void exp_jtheta(double theta, double *in_dq, double*out_dq){
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    out_dq[0] = cos_theta*in_dq[0] - sin_theta*in_dq[1];
    out_dq[1] = sin_theta*in_dq[0] + cos_theta*in_dq[1];
}

void regulator_PI_current_dq(twin_control_data *data_ctrl, para_twinmachine_control_single para_m){
    double K1 = para_m.R*data_ctrl->PI_regulator->Bd/data_ctrl->PI_regulator->Bp;
    double u1[2];
    double u2[2];
    u1[0] = data_ctrl->error[0] * K1;
    u1[1] = data_ctrl->error[1] * K1;
    u2[0] = u1[0] - data_ctrl->PI_regulator->state_1[0] + data_ctrl->PI_regulator->state_2[0] + data_ctrl->PI_regulator->state_3[0];
    u2[1] = u1[1] - data_ctrl->PI_regulator->state_1[1] + data_ctrl->PI_regulator->state_2[1] + data_ctrl->PI_regulator->state_3[1];

    double theta;
    theta = data_ctrl->we*2.0*data_ctrl->PI_regulator->Ts;
    exp_jtheta(theta, u2, data_ctrl->vdq0_ref);

    // delay states
    // update state 1
    theta = data_ctrl->we*-1.0*data_ctrl->PI_regulator->Ts;
    exp_jtheta(theta, u1, data_ctrl->PI_regulator->state_1);
    data_ctrl->PI_regulator->state_1[0] = data_ctrl->PI_regulator->state_1[0]*data_ctrl->PI_regulator->Ap;
    data_ctrl->PI_regulator->state_1[1] = data_ctrl->PI_regulator->state_1[1]*data_ctrl->PI_regulator->Ap;

    //update state 2
    data_ctrl->PI_regulator->state_2[0] = u2[0]*data_ctrl->PI_regulator->Ad;
    data_ctrl->PI_regulator->state_2[1] = u2[1]*data_ctrl->PI_regulator->Ad;

    //update state 3
    data_ctrl->PI_regulator->state_3[0] = u2[0]*data_ctrl->PI_regulator->Bd;
    data_ctrl->PI_regulator->state_3[1] = u2[1]*data_ctrl->PI_regulator->Bd;
}

void decouple(twinbearingless_control *data){
    data->tq.vdq0_decouple[0] = 0.0;
    data->tq.vdq0_decouple[1] = 0.0;
    data->tq.vdq0_decouple[2] = 0.0;

    double vabc[3];
    double vdecouple[3];
    dq0_to2_abc(vabc, data->tq.vdq0_ref, data->tq.theta_rad);
    abc_to_dq0(vabc, vdecouple, data->s1.theta_rad);

    data->s1.vdq0_decouple[0] = 0.5*vdecouple[0]; 
    data->s1.vdq0_decouple[1] = 0.5*vdecouple[1]; 
    data->s1.vdq0_decouple[2] = 0.5*vdecouple[2]; 

    abc_to_dq0(vabc, vdecouple, data->s2.theta_rad);
    data->s2.vdq0_decouple[0] = 0.5*vdecouple[0]; 
    data->s2.vdq0_decouple[1] = 0.5*vdecouple[1]; 
    data->s2.vdq0_decouple[2] = 0.5*vdecouple[2]; 
}


void get_theta_we(twinbearingless_control *data){
    data->tq.theta_rad += data->tq.we*data->tq.PI_regulator->Ts;
    data->tq.theta_rad = fmod(data->tq.theta_rad, 2.0 * PI); // Wrap to 2*pi

    data->s1.theta_rad += data->s1.we*data->s1.PI_regulator->Ts;
    data->s1.theta_rad = fmod(data->s1.theta_rad, 2.0 * PI); // Wrap to 2*pi

    data->s2.theta_rad += data->s2.we*data->s2.PI_regulator->Ts;
    data->s2.theta_rad = fmod(data->s2.theta_rad, 2.0 * PI); // Wrap to 2*pi
}



void current_regulation (twinbearingless_control *data)
{
    if(!data->is_init || data == 0x00000000){
        data = init_twinbearingless(data);
    }

    //update sensed currents
    get_all_inverter_current_abc(data);
    //update theta and we
    get_theta_we(data);
    //calculate torque and suspension currents
    cal_invI_to_controlI(data);

    update_control_current(&data->tq);
    update_control_current(&data->s1);
    update_control_current(&data->s2);

    regulator_PI_current_dq(&data->tq, data->para_machine->para_tq);
    regulator_PI_current_dq(&data->s1, data->para_machine->para_s1);
    regulator_PI_current_dq(&data->s2, data->para_machine->para_s2);

    decouple(data);
    //
    double vdq0[3];
    vdq0[0] = data->tq.vdq0_ref[0] + data->tq.vdq0_decouple[0];
    vdq0[1] = data->tq.vdq0_ref[1] + data->tq.vdq0_decouple[1];
    vdq0[2] = data->tq.vdq0_ref[2] + data->tq.vdq0_decouple[2];

    dq0_to2_abc(data->twin_inv1.vabc_ref, vdq0, data->tq.theta_rad);

    vdq0[0] = -data->s1.vdq0_ref[0] + data->s1.vdq0_decouple[0];
    vdq0[1] = -data->s1.vdq0_ref[1] + data->s1.vdq0_decouple[1];
    vdq0[2] = -data->s1.vdq0_ref[2] + data->s1.vdq0_decouple[2];

    dq0_to2_abc(data->twin_inv2.vabc_ref, vdq0, data->s1.theta_rad);

    vdq0[0] = -data->s2.vdq0_ref[0] + data->s2.vdq0_decouple[0];
    vdq0[1] = -data->s2.vdq0_ref[1] + data->s2.vdq0_decouple[1];
    vdq0[2] = -data->s2.vdq0_ref[2] + data->s2.vdq0_decouple[2];

    dq0_to2_abc(data->twin_inv3.vabc_ref, vdq0, data->s2.theta_rad);
}
