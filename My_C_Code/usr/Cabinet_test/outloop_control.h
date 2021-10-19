#ifndef OUTLOOP_CONTROL_H
#define OUTLOOP_CONTROL_H

#include "usr/Cabinet_test/twinbearingless_control.h"
#include <stdint.h>



typedef struct para_PI_discrete_normal{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double Kp;
    double Ki;
    double state_1[3];

} para_PI_discrete_normal;

typedef struct para_lpf{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double fs;

    double state_1[3];

} para_lpf;

typedef struct para_observer{
    double enable;
    double Ts;
    double Kd;
    double Kj;
    double state1;
    double state2;
    para_PI_discrete_normal para_PI;

} para_observer;

typedef struct para_anti_windup{
    
    double sat_low;
    double sat_high;
    double k;

} para_anti_windup;

typedef struct para_levitation_control{
    int enable;
    para_PI_discrete_normal para_PI;
    double Ts;
    double ka;
    double ba;
    double state_1[3];
    para_anti_windup para_anti_wp;
    para_lpf para_lpf;
    para_lpf para_delta_lpf;

} para_levitation_control;

typedef struct para_velocity_control{
    double wrm_max;
    para_lpf para_lpf;
    para_PI_discrete_normal para_PI;


} para_velocity_control;


typedef struct bim_velocity_control{

    int enable;
    double Ts;
    double wrm_ref;
    double wrm_ref_lpf;
    double wrm_ref_inject;
    double err_wrm;
    double Te_ref;
    double Idq0_ref[3];
    double lamda_m_ref;
    double wsl_ref;
    double theta_re_ref;
    double wre_ref;
    double CFO_state;
    double w_test;

    double theta_rm_mes;
    double theta_rm_mes_pre[2];
    
    uint32_t time_pre;
    int32_t step_pre;
    double wrm_mes;

    double wrm_est;
    double wrm_est_hf;
    double theta_rm_est;
    para_velocity_control para_velocity_control;
    para_observer para_ob;

}bim_velocity_control;


typedef struct bim_levitation_control{
    int enable;
    double delta_ref[2];
    double delta_ref_lpf[2];
    double Ixy0_ref[3];
    double Ixy0_log[3];
    double F_xy[2];
    double F_xy_inject[2];
    double F_xy_out[2];
    double delta_mes[2];
    double delta_mes_lpf[2];
    double err_delta[2];
    double out_antiwp[3];

    double diff_state[2];

    double w_inject;
    double mag_inject;
    para_levitation_control para_levi_control;

}bim_levitation_control;

typedef struct bim_control {
    int is_init;
    double theta_rm_mes_pre[2];
    bim_velocity_control bim_v_control;
    bim_levitation_control bim_lev_control;
    twinbearingless_control *current_control;
    para_bim *BIM_PARA;
}bim_control;


extern bim_control bim_control_data;
//extern static double theta_pre;

bim_control *init_bim(void);
bim_control *deinit_bim(void);
bim_control *reset_bim(void);

void bim_controlloop(bim_control *data);
#endif// TWINBEARINGLESS_CONTROL_H
