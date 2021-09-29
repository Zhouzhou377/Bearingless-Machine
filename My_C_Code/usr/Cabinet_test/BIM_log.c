#include "usr/Cabinet_test/BIM_log.h"
#include "usr/Cabinet_test/outloop_control.h"


double LOG_Iabc1_a = 0.0;
double LOG_Iabc1_b = 0.0;
double LOG_Iabc1_c = 0.0;
double LOG_Iabc2_a = 0.0;
double LOG_Iabc2_b = 0.0;
double LOG_Iabc2_c = 0.0;

double LOG_Itq_d_ref = 0.0;
double LOG_Itq_q_ref = 0.0;

double LOG_Is1_x_ref = 0.0;
double LOG_Is1_y_ref = 0.0;

double LOG_Itq_d = 0.0;
double LOG_Itq_q = 0.0;

double LOG_Is1_x = 0.0;
double LOG_Is1_y = 0.0;

double LOG_Te_ref = 0.0;

double LOG_wrm = 0.0;
double LOG_theta_rm = 0.0;
double LOG_theta_rm_pre = 0.0;

double LOG_wsl = 0.0;
double LOG_theta_e = 0.0;
double LOG_we = 0.0;

double LOG_Te = 0.0;

double LOG_va1_ref = 0.0;
double LOG_vb1_ref = 0.0;
double LOG_vc1_ref = 0.0;

double LOG_va2_ref = 0.0;
double LOG_vb2_ref = 0.0;
double LOG_vc2_ref = 0.0;

double LOG_vd_ref = 0.0;
double LOG_vq_ref = 0.0;
double LOG_v0_ref = 0.0;

double LOG_vx_ref = 0.0;
double LOG_vy_ref = 0.0;

double LOG_delta_x = 0.0;
double LOG_delta_y = 0.0;

double LOG_delta_x_ref = 0.0;
double LOG_delta_y_ref = 0.0;

double LOG_delta_x_ref_lpf = 0.0;
double LOG_delta_y_ref_lpf = 0.0;


void BIM_log (bim_control *data){
    LOG_wrm = data->bim_v_control.wrm_mes;
	LOG_theta_rm = data->bim_v_control.theta_rm_mes;
    LOG_wsl = data->bim_v_control.wsl_ref;
	LOG_theta_e = data->bim_v_control.theta_re_ref;
    //LOG_theta_rm_pre = data->bim_v_control.theta_rm_mes_pre[0];
	//LOG_Te_ref
	

	LOG_Itq_d_ref = data->current_control->tq.Idq0_ref[0];
	LOG_Itq_q_ref = data->current_control->tq.Idq0_ref[1];

	LOG_Itq_d = data->current_control->tq.Idq0[0];
	LOG_Itq_q = data->current_control->tq.Idq0[1];

	LOG_Iabc1_a = data->current_control->twin_inv1.Iabc[0];
	LOG_Iabc1_b = data->current_control->twin_inv1.Iabc[1];
	LOG_Iabc1_c = data->current_control->twin_inv1.Iabc[2];
	LOG_Iabc2_a = data->current_control->twin_inv2.Iabc[0];
	LOG_Iabc2_b = data->current_control->twin_inv2.Iabc[1];
	LOG_Iabc2_c = data->current_control->twin_inv2.Iabc[2];

	LOG_Te = data->bim_v_control.Te_ref;

	LOG_va1_ref = data->current_control->twin_inv1.vabc_ref[0];
	LOG_vb1_ref = data->current_control->twin_inv1.vabc_ref[1];
	LOG_vc1_ref = data->current_control->twin_inv1.vabc_ref[2];

	LOG_va2_ref = data->current_control->twin_inv2.vabc_ref[0];
	LOG_vb2_ref = data->current_control->twin_inv2.vabc_ref[1];
	LOG_vc2_ref = data->current_control->twin_inv2.vabc_ref[2];

	LOG_vd_ref = data->current_control->tq.vdq0_ref[0];
	LOG_vq_ref = data->current_control->tq.vdq0_ref[1];
	LOG_v0_ref = data->current_control->tq.vdq0_ref[2];

	LOG_vx_ref = data->current_control->s1.vdq0_ref[0];
	LOG_vy_ref = data->current_control->s1.vdq0_ref[1];

	LOG_Is1_x = data->current_control->s1.Idq0[0];
	LOG_Is1_y = data->current_control->s1.Idq0[1];
	LOG_Is1_x_ref = data->current_control->s1.Idq0_ref[0];
	LOG_Is1_y_ref = data->current_control->s1.Idq0_ref[1];

	LOG_delta_x = data->bim_lev_control.delta_mes[0];
	LOG_delta_y = data->bim_lev_control.delta_mes[1];

	LOG_delta_x_ref = data->bim_lev_control.delta_ref[0];
	LOG_delta_y_ref = data->bim_lev_control.delta_ref[1];

	LOG_delta_x_ref_lpf = data->bim_lev_control.delta_ref_lpf[0];
	LOG_delta_y_ref_lpf = data->bim_lev_control.delta_ref_lpf[1];

}
