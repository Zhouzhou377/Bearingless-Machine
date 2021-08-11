#ifndef TWINBEARINGLESS_CONTROL
#define TWINBEARINGLESS_CONTROL

#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/para_machine.h"
//#include "usr/Cabinet_test/controllers.h"
//#include <stdbool.h>

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
    bool enable;
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

twinbearingless_control twin_control;


twinbearingless_control *init_twinbearingless(void);
void current_regulation (twinbearingless_control *data);
#endif
