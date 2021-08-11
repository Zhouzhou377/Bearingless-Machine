#ifndef TWINBEARINGLESS_CONTROL
#define TWINBEARINGLESS_CONTROL

#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/controllers.h"



typedef struct twin_data{
    int Num_inv;
    InverterThreePhase_t *inv;
    double Iabc[3];
    double Idq0[3];
    double Idq0_m1[3];
    double Idq0_ref[3];
    double vabc_ref[3];
    double theta_rad;

    PI_discrete_para *PI;

} twin_data;

typedef struct twinbearingless_control{
    bool is_init = 0;
    //inv1
    twin_data twin_inv1;
    //inv2
    twin_data twin_inv2;
    //inv2
    twin_data twin_inv3;

} twinbearingless_control;



void init_twinbearingless(void);
#endif