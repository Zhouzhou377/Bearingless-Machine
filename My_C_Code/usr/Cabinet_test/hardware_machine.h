#ifndef HARDWARE_MACHINE_H
#define HARDWARE_MACHINE_H

#define BM_ENABLE 1
#define TS (1.0/10000.0)
typedef struct para_bim_machine{

	double rs;
	double Lls;
	double Ls;
	double Lm;
	double rr;
	double Llr;
	double Lr;
	double tau_r;
	double Lss;
	int p;
	int ps;


	double kf;
	double k_delta;
	double ki;
	double delta_limit;
	double m_rotor;
	double J;

	double iabc_max;
	double wrm_max;

	double id_ref;
	


} para_bim_machine;


typedef struct para_bim_control{

	double v_pi_Kp;
	double v_pi_Ki;
	double v_lpf_f;
	double lev_pi_Kp;
	double lev_pi_Ki;
	double lev_lpf_f;
	double lev_ka;
	double lev_ba;
	double lev_sat_low;
	double lev_sat_high;
	double lev_antiwp_k;
	double ob_theta_fi;
	double ob_theta_fp;
	double ob_theta_fd;
}para_bim_control;

typedef struct para_bim{
	para_bim_machine para_machine;
	para_bim_control para_control;
} para_bim;


para_bim *get_para_BIM(void);

#endif // HARDWARE_H
