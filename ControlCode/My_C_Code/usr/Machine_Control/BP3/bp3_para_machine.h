#ifndef BP3_PARA_MACHINE_H
#define BP3_PARA_MACHINE_H


#define TS (1.0/10000.0)
typedef struct para_bp3_machine{

	double rs;
	double Lls;
	double Ls;
	double Lm;
	double rr;
	double Llr;
	double Lr;
	double tau_r;
	double Lss;
	double theta_offset_xy;
	int p;
	int ps;

	double kt;
	double kf;
	double kf_a2;
	double kf_a1;
	double kf_a0;
	double kf_theta_rad;
	double k_delta;
	double k_delta_a2;
	double k_delta_a1;
	double k_delta_a0;
	double ki;
	double delta_limit;
	double m_rotor;
	double J;

	double iabc_max;
	double wrm_max;

	double id_ref;
	


} para_bp3_machine;


typedef struct para_bp3_control{

	double v_pi_Kp;
	double v_pi_Ki;
	double v_lpf_f;
	double lev_pi_Kp;
	double lev_pi_Ki;
	double lev_lpf_f;
	double lev_delta_lpf_f;
	double lev_ka;
	double lev_ba;
	double lev_sat_low;
	double lev_sat_high;
	double lev_antiwp_k;
	double ob_theta_fi;
	double ob_theta_fp;
	double ob_theta_fd;
}para_bp3_control;

typedef struct para_bp3{
	para_bp3_machine para_machine;
	para_bp3_control para_control;
} para_bp3;


para_bp3 *get_para_bp3(void);
void update_para_bp3_activedamping(para_bp3 *data, double id);
#endif // HARDWARE_H
