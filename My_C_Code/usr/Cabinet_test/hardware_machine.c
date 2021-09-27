#include "usr/Cabinet_test/hardware_machine.h"
#include "usr/Cabinet_test/definitions.h"



static para_bim BIM_PARA = {
	.para_machine.rs = 0.5,
	.para_machine.Lls = 0.00148,
	.para_machine.Lm = 0.00502,
	.para_machine.Ls = 0.00074 + 0.00502,
	.para_machine.rr = 0.47,
	.para_machine.Llr = 0.00074,
	.para_machine.Lr = 0.00074 + 0.00502,
	.para_machine.tau_r = (0.00074 + 0.00502)/0.47,    //.para_machine.Lr/.para_machine.rr,
	.para_machine.Lss = 0.000285,
	.para_machine.p = 1,
	.para_machine.ps = 2,

	.para_machine.kf = 13.333333,
	.para_machine.k_delta_a2 = 0.42365,
	.para_machine.k_delta_a1 = -1.16487,
	.para_machine.k_delta_a0 = 0.397244,
	.para_machine.k_delta = 70000.0,
	.para_machine.ki = 0.0,
	.para_machine.delta_limit = 0.00265,
	.para_machine.m_rotor = 1.488,
	.para_machine.J = 0.01,
	
	.para_machine.iabc_max = 40.0,
	.para_machine.wrm_max = 1000.0,
	.para_machine.id_ref = 1.0,

	.para_control.v_pi_Kp = 0.01,
	.para_control.v_pi_Ki= 0.001,
	.para_control.v_lpf_f= 0.1,
	.para_control.lev_pi_Kp= 7000.0,
	.para_control.lev_pi_Ki= 70000.0,
	.para_control.lev_lpf_f= 1000.0,
	.para_control.lev_ka= 2.1804e+05,
	.para_control.lev_ba= 942.4778,
	.para_control.lev_sat_low= -20.0,
	.para_control.lev_sat_high= 20.0,
	.para_control.lev_antiwp_k= 1.4286e-04,
	.para_control.ob_theta_fd = 200.0,
	.para_control.ob_theta_fp = 40.0,
	.para_control.ob_theta_fi = 8.0

};

para_bim *get_para_BIM(void){

	return &BIM_PARA;
}

void update_para_activedamping(para_bim *data, double id){

	data->para_machine.k_delta = data->para_machine.k_delta_a2*id*id+data->para_machine.k_delta_a1*id+data->para_machine.k_delta_a0;
	double f_bw = 50.0;
	double w_bw = PI2*f_bw;
	data->para_control.lev_ka = w_bw*w_bw*data->para_machine.m_rotor+data->para_machine.k_delta;
	data->para_control.lev_ba = 2.0*w_bw*data->para_machine.m_rotor;
	data->para_control.lev_antiwp_k = 1.0/data->para_machine.k_delta*10.0;
}
