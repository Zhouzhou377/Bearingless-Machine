#ifndef TWINBEARINGLESS_CONTROL_H
#define TWINBEARINGLESS_CONTROL_H

#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/para_machine.h"
//#include "usr/Cabinet_test/controllers.h"
#include <stdbool.h>

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

typedef struct twin_threephase_data{
    int Num_inv;
    InverterThreePhase_t *inv;
    double Iabc[3];
    double vabc_ref[3];

} twin_threephase_data;

typedef struct twin_control_data{
    double error[3];
    double Iabc[3];
    double Idq0[3];
    double Idq0_m1[3];
    double Idq0_ref[3];
    double vdq0_ref[3];
    double vdq0_decouple[3];
    double theta_rad;
    double we;
    para_PI_discrete *PI_regulator;

}twin_control_data;

typedef struct twinbearingless_control{
    bool is_init;

    //inv1
    twin_threephase_data twin_inv1;
    //inv2
    twin_threephase_data twin_inv2;
    //inv2
    twin_threephase_data twin_inv3;

    twin_control_data tq;
    twin_control_data s1;
    twin_control_data s2;
    para_twinmachine_control *para_machine;


} twinbearingless_control;


extern para_PI_discrete PI_tq;
extern para_PI_discrete PI_s1;
extern para_PI_discrete PI_s2;
extern twinbearingless_control twin_control;

twinbearingless_control *init_twinbearingless(void);
twinbearingless_control *deinit_twinbearingless(void);

void reset_regulator(void);
void current_regulation (twinbearingless_control *data);
void get_inverter_current_abc(twin_threephase_data *twin);
void get_all_inverter_current_abc(twinbearingless_control* data);
void cal_invI_to_controlI(twinbearingless_control* data);
void update_control_current(twin_control_data *data);
void regulator_PI_current_dq(twin_control_data *data_ctrl, para_twinmachine_control_single para_m);
void decouple(twinbearingless_control *data);
void get_theta_we(twinbearingless_control *data);

#endif// TWINBEARINGLESS_CONTROL_H
