#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Cabinet_test/OpenloopVSI.h"
#include "usr/Cabinet_test/inverter.h"
#include "usr/Cabinet_test/task_cabinet.h"
#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/twinbearingless_control.h"

#include "sys/scheduler.h"
#include "sys/debug.h"


//LOGGING VARIABLES

double LOG_vdc = 0.0;
double LOG_mb_I1 = 0.0;
double LOG_mb_I2 = 0.0;
double LOG_mb_I3 = 0.0;
double LOG_mb_I4 = 0.0;
double LOG_mb_I5 = 0.0;
double LOG_mb_I6 = 0.0;



double LOG_Iabc1_a = 0.0;
double LOG_Iabc1_b = 0.0;
double LOG_Iabc1_c = 0.0;
double LOG_Iabc2_a = 0.0;
double LOG_Iabc2_b = 0.0;
double LOG_Iabc2_c = 0.0;
double LOG_Iabc3_a = 0.0;
double LOG_Iabc3_b = 0.0;
double LOG_Iabc3_c = 0.0;

double LOG_vabc1_a = 0.0;
double LOG_vabc1_b = 0.0;
double LOG_vabc1_c = 0.0;
double LOG_vabc2_a = 0.0;
double LOG_vabc2_b = 0.0;
double LOG_vabc2_c = 0.0;
double LOG_vabc3_a = 0.0;
double LOG_vabc3_b = 0.0;
double LOG_vabc3_c = 0.0;

double LOG_Itq_d_ref = 0.0;
double LOG_Itq_q_ref = 0.0;

double LOG_Is1_d_ref = 0.0;
double LOG_Is1_q_ref = 0.0;

double LOG_Is2_d_ref = 0.0;
double LOG_Is2_q_ref = 0.0;

double LOG_Itq_d = 0.0;
double LOG_Itq_q = 0.0;

double LOG_Is1_d = 0.0;
double LOG_Is1_q = 0.0;

double LOG_Is2_d = 0.0;
double LOG_Is2_q = 0.0;

double LOG_err_tq_d = 0.0;
double LOG_err_tq_q = 0.0;

double LOG_err_s1_d = 0.0;
double LOG_err_s1_q = 0.0;

double LOG_err_s2_d = 0.0;
double LOG_err_s2_q = 0.0;

double LOG_theta_tq = 0.0;
double LOG_theta_s1 = 0.0;
double LOG_theta_s2 = 0.0;

//sensed values
double LOG_x = 0.0;
double LOG_y = 0.0;
double LOG_z = 0.0;

double LOG_Ix = 0.0;
double LOG_Iy = 0.0;
double LOG_Iz = 0.0;

//filtered values
double LOG_xf = 0.0;
double LOG_yf = 0.0;
double LOG_zf = 0.0;

double LOG_Ixf = 0.0;
double LOG_Iyf = 0.0;
double LOG_Izf = 0.0;

para_PI_discrete PI_tq;
para_PI_discrete PI_s1;
para_PI_discrete PI_s2;
twinbearingless_control twin_control;
para_twinmachine_control para_machine_control;
para_machine_h para_machine_data;
twinbearingless_control *twin_data;

OpenLoop_Command VSI_Openloop_command;
OpenLoop_Command *OpenLoop;

cmd_signal cmd_enable;

#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static task_control_block_t tcb;

//register and begin task
void task_cabinet_init(void)
{
	if (task_cabinet_is_inited){
		return;
	}
	//populate struct
	scheduler_tcb_init(&tcb, task_cabinet_callback, NULL, "cabinet", TASK_CABINET_INTERVAL_USEC);
	scheduler_tcb_register(&tcb);

	default_inverter_setup(0);
	OpenLoop = init_OpenLoop_Command();
	twin_data = init_twinbearingless();
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_currentcontrol = 0;
	cmd_enable.enable_log = 0;
}

//stop task
void task_cabinet_deinit(void)
{
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_currentcontrol = 0;
	cmd_enable.enable_log = 0;
	scheduler_tcb_unregister(&tcb);
	twin_data = deinit_twinbearingless();
}

//function to determine if task has been started
bool task_cabinet_is_inited(void)
{	
	return scheduler_tcb_is_registered(&tcb);
}

//call back function to be run repeatedly
void task_cabinet_callback(void *arg)
{
	InverterThreePhase_t *inv;

	if (cmd_enable.enable_openloop){
		OpenLoop = &VSI_Openloop_command;
		OpenLoop_VSI(OpenLoop);
		inv = get_three_phase_inverter(OpenLoop->Num_inv);
		set_line_volts_three_phase(OpenLoop->command_volatge[0], OpenLoop->command_volatge[1], OpenLoop->command_volatge[2], inv);
	}
	else if(cmd_enable.enable_currentcontrol)
	{
		twin_data = &twin_control;
		current_regulation (twin_data);

		set_line_volts_three_phase(twin_data->twin_inv1.vabc_ref[0], twin_data->twin_inv1.vabc_ref[1], twin_data->twin_inv1.vabc_ref[2], twin_data->twin_inv1.inv);
		set_line_volts_three_phase(twin_data->twin_inv2.vabc_ref[0], twin_data->twin_inv2.vabc_ref[1], twin_data->twin_inv2.vabc_ref[2], twin_data->twin_inv2.inv);
		set_line_volts_three_phase(twin_data->twin_inv3.vabc_ref[0], twin_data->twin_inv3.vabc_ref[1], twin_data->twin_inv3.vabc_ref[2], twin_data->twin_inv3.inv);
	}

	if (cmd_enable.enable_log){
		twin_data = &twin_control;
		//log three phase inv current
		LOG_Iabc1_a = twin_data->twin_inv1.Iabc[0];
		LOG_Iabc1_b = twin_data->twin_inv1.Iabc[1];
		LOG_Iabc1_c = twin_data->twin_inv1.Iabc[2];
		LOG_Iabc2_a = twin_data->twin_inv2.Iabc[0];
		LOG_Iabc2_b = twin_data->twin_inv2.Iabc[1];
		LOG_Iabc2_c = twin_data->twin_inv2.Iabc[2];
		LOG_Iabc3_a = twin_data->twin_inv3.Iabc[0];
		LOG_Iabc3_b = twin_data->twin_inv3.Iabc[1];
		LOG_Iabc3_c = twin_data->twin_inv3.Iabc[2];
		//log torque current

		LOG_Itq_d_ref = twin_data->tq.Idq0_ref[0];
		LOG_Itq_q_ref = twin_data->tq.Idq0_ref[1];

		LOG_Itq_d = twin_data->tq.Idq0[0];
		LOG_Itq_q = twin_data->tq.Idq0[1];

		LOG_Is1_d_ref = twin_data->s1.Idq0_ref[0];
		LOG_Is1_q_ref = twin_data->s1.Idq0_ref[1];

		LOG_Is1_d = twin_data->s1.Idq0[0];
		LOG_Is1_q = twin_data->s1.Idq0[1];

		LOG_Is2_d_ref = twin_data->s2.Idq0_ref[0];
		LOG_Is2_q_ref = twin_data->s2.Idq0_ref[2];

		LOG_Is2_d = twin_data->s2.Idq0[0];
		LOG_Is2_q = twin_data->s2.Idq0[2];

		LOG_vabc1_a = twin_data->twin_inv1.vabc_ref[0];
		LOG_vabc1_b = twin_data->twin_inv1.vabc_ref[1];
		LOG_vabc1_c = twin_data->twin_inv1.vabc_ref[2];
		LOG_vabc2_a = twin_data->twin_inv2.vabc_ref[0];
		LOG_vabc2_b = twin_data->twin_inv2.vabc_ref[1];
		LOG_vabc2_c = twin_data->twin_inv2.vabc_ref[2];
		LOG_vabc3_a = twin_data->twin_inv3.vabc_ref[0];
		LOG_vabc3_b = twin_data->twin_inv3.vabc_ref[1];
		LOG_vabc3_c = twin_data->twin_inv3.vabc_ref[2];

		LOG_err_tq_d = twin_data->tq.error[0];
		LOG_err_tq_q = twin_data->tq.error[1];

		LOG_err_s1_d = twin_data->s1.error[0];
		LOG_err_s1_q = twin_data->s1.error[1];

		LOG_err_s2_d = twin_data->s2.error[0];
		LOG_err_s2_q = twin_data->s2.error[1];

		LOG_theta_tq = twin_data->tq.theta_rad;
		LOG_theta_s1 = twin_data->s1.theta_rad;
		LOG_theta_s2 = twin_data->s2.theta_rad;

	}


}


void task_cabinet_stats_print(void)
{
    task_stats_print(&tcb.stats);
}

void task_cabinet_stats_reset(void)
{
    task_stats_reset(&tcb.stats);
}

#endif //APP_CABINET_TEST
