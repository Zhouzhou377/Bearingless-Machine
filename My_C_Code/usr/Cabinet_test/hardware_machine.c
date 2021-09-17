#include "usr/Cabinet_test/hardware_machine.h"



static const para_bim BIM_PARA = {
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
	.para_machine.k_delta = 70000.0,
	.para_machine.ki = 0.0,
	.para_machine.delta_limit = 0.00265,
	.para_machine.m_rotor = 1.488,
	.para_machine.iabc_max = 15.0,
	.para_machine.wrm_max = 30.0,
	.para_machine.id_ref = 6.0,

	.para_control.v_pi_Kp = 31.4159,
	.para_control.v_pi_Ki= 36.9388,
	.para_control.v_lpf_f= 1.0,
	.para_control.lev_pi_Kp= 100000.0,
	.para_control.lev_pi_Ki= 1000000.0,
	.para_control.lev_lpf_f= 1000.0,
	.para_control.lev_ka= 2.1804e+05,
	.para_control.lev_ba= 942.4778,
	.para_control.lev_sat_low= -20.0,
	.para_control.lev_sat_high= 20.0,
	.para_control.lev_antiwp_k= 1.4286e-04

};

para_bim *get_para_BIM(void){
	return &BIM_PARA;
}

