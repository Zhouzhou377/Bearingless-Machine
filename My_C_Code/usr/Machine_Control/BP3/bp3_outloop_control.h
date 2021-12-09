#ifndef bp3_OUTLOOP_CONTROL_H
#define bp3_OUTLOOP_CONTROL_H

#include "usr/Machine_Control/currentloop_control.h"
#include "usr/Machine_Control/control_structure.h"
#include "usr/Machine_Control//BP3/bp3_para_machine.h"
#include <stdint.h>



typedef struct bp3_velocity_control{

    int enable;
    int is_start;
    int enable_encoder;
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

    double theta_rm_mes_offset;
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

}bp3_velocity_control;


typedef struct bp3_levitation_control{
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

}bp3_levitation_control;

typedef struct bp3_control {
    int is_init;
    double theta_rm_mes_pre[2];
    bp3_velocity_control bp3_v_control;
    bp3_levitation_control bp3_lev_control;
    currentloop_control *current_control;
    para_bp3 *BP3_PARA;
}bp3_control;


extern bp3_control bp3_control_data;
//extern static double theta_pre;

bp3_control *init_bp3(void);
bp3_control *deinit_bp3(void);
bp3_control *reset_bp3(void);
void bm_start_theta(bp3_control* data);
void bp3_controlloop(bp3_control *data);
#endif// TWINBEARINGLESS_CONTROL_H
