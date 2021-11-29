#include "usr/Machine_Control//BP3/bp3_para_machine.h"
#include "usr/Machine_Control/definitions.h"



static para_bp3 BP3_PARA = {
	.para_machine.rs = 0.5,
	.para_machine.Lls = 0.00148,
	.para_machine.Lm = 0.00502,
	.para_machine.Ls = 0.00074 + 0.00502,
	.para_machine.rr = 0.47,
	.para_machine.Llr = 0.00074,
	.para_machine.Lr = 0.00074 + 0.00502,
	.para_machine.tau_r = (0.00074 + 0.00502)/0.47,    //.para_machine.Lr/.para_machine.rr,
	.para_machine.Lss = 0.000285,
	.para_machine.theta_offset_xy = -PI/24,
	.para_machine.p = 1,
	.para_machine.ps = 2,

	.para_machine.kt = 0.07,
	.para_machine.kf = 13.333333,
	.para_machine.kf_theta_rad = 130.0/180.0*PI,
	.para_machine.kf_a2 = 0.0130322,
	.para_machine.kf_a1 = 0.797446,
	.para_machine.kf_a0 = 0.0190783,
	.para_machine.k_delta_a2 = 0.42365,
	.para_machine.k_delta_a1 = -1.16487,
	.para_machine.k_delta_a0 = 0.397244,
	.para_machine.k_delta = 70000.0,
	.para_machine.ki = 0.0,
	.para_machine.delta_limit = 0.00265,
	.para_machine.m_rotor = 1.488*0.0 + 1.5,
	.para_machine.J = 0.005,
	
	.para_machine.iabc_max = 30.0,
	.para_machine.wrm_max = 5000.0,
	.para_machine.id_ref = 2.0,

	.para_control.v_pi_Kp = 0.1,
	.para_control.v_pi_Ki= 0.05,
	.para_control.v_lpf_f= 0.1,
	.para_control.lev_pi_Kp= 0.0,
	.para_control.lev_pi_Ki= 0.0,
	.para_control.lev_lpf_f= 300.0,
	.para_control.lev_delta_lpf_f= 1,
	.para_control.lev_ka= 2.1804e+05,
	.para_control.lev_ba= 942.4778,
	.para_control.lev_sat_low= -10.0,
	.para_control.lev_sat_high= 10.0,
	.para_control.lev_antiwp_k= 1.4286e-04,
	.para_control.ob_theta_fd = 200.0*1.8,
	.para_control.ob_theta_fp = 40.0*1.8,
	.para_control.ob_theta_fi = 8.0*1.8

};

para_bp3 *get_para_bp3(void){

	return &BP3_PARA;
}

void update_para_bp3_activedamping(para_bp3 *data, double id){

	data->para_machine.k_delta = data->para_machine.k_delta_a2*id*id+data->para_machine.k_delta_a1*id+data->para_machine.k_delta_a0;
	data->para_machine.k_delta = (data->para_machine.k_delta*1000.0)*1.0;
	data->para_machine.kf = (data->para_machine.kf_a2*id*id+data->para_machine.kf_a1*id+data->para_machine.kf_a0)*1.0;
	double f_bw = 15.0;
	double w_bw = PI2*f_bw;
	data->para_control.lev_ka = (w_bw*w_bw*data->para_machine.m_rotor+data->para_machine.k_delta)*1.00;
	data->para_control.lev_ba = 2.0*w_bw*data->para_machine.m_rotor*1.8;
	data->para_control.lev_antiwp_k = 0.0/data->para_machine.k_delta*10.0;
	data->para_control.lev_pi_Ki = ((data->para_control.lev_ka - data->para_machine.k_delta)*w_bw/10.0)*1.0;
	data->para_control.lev_pi_Kp = (data->para_control.lev_pi_Ki/10)*1.0;

}
