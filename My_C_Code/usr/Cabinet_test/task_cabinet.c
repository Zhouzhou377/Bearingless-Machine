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



double LOG_Iabc1_1 = 0.0;
double LOG_Iabc1_2 = 0.0;
double LOG_Iabc1_3 = 0.0;
double LOG_Iabc2_1 = 0.0;
double LOG_Iabc2_2 = 0.0;
double LOG_Iabc2_3 = 0.0;
double LOG_Iabc3_1 = 0.0;
double LOG_Iabc3_2 = 0.0;
double LOG_Iabc3_3 = 0.0;

double LOG_Itq_d = 0.0;
double LOG_Itq_q = 0.0;

double LOG_Is1_d = 0.0;
double LOG_Is1_q = 0.0;

double LOG_Is2_d = 0.0;
double LOG_Is2_q = 0.0;



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
twinbearingless_control *twin_data;

OpenLoop_Command *OpenLoop;

#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static task_control_block_t tcb;

//register and begin task
void task_cabinet_init(void)
{
	//populate struct
	scheduler_tcb_init(&tcb, task_cabinet_callback, NULL, "cabinet", TASK_CABINET_INTERVAL_USEC);
	scheduler_tcb_register(&tcb);

	default_inverter_setup(0);
	OpenLoop = init_OpenLoop_Command();
	twin_data = init_twinbearingless();
}

//stop task
void task_cabinet_deinit(void)
{
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_currentcontrol = 0;
	scheduler_tcb_unregister(&tcb);

	//(get_mc());

}

//function to determine if task has been started
uint8_t task_cabinet_is_inited(void)
{	
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_currentcontrol = 0;
	return scheduler_tcb_is_registered(&tcb);
}

//call back function to be run repeatedly
void task_cabinet_callback(void *arg)
{
	InverterThreePhase_t *inv;
	OpenLoop = &VSI_Openloop_command;
	if (cmd_enable.enable_openloop){
		OpenLoop_VSI(OpenLoop);
		inv = get_three_phase_inverter(OpenLoop->Num_inv);
		set_line_volts_three_phase(OpenLoop->command_volatge[0], OpenLoop->command_volatge[1], OpenLoop->command_volatge[2], inv);
	}
	else if(cmd_enable.enable_currentcontrol)
	{
		current_regulation (twin_data);

		set_line_volts_three_phase(twin_data->twin_inv1.vabc_ref[0], twin_data->twin_inv1.vabc_ref[1], twin_data->twin_inv1.vabc_ref[2], twin_data->twin_inv1.inv);
		set_line_volts_three_phase(twin_data->twin_inv2.vabc_ref[0], twin_data->twin_inv2.vabc_ref[1], twin_data->twin_inv2.vabc_ref[2], twin_data->twin_inv2.inv);
		set_line_volts_three_phase(twin_data->twin_inv3.vabc_ref[0], twin_data->twin_inv3.vabc_ref[1], twin_data->twin_inv3.vabc_ref[2], twin_data->twin_inv3.inv);
	}

	if (cmd_enable.enable_log && cmd_enable.enable_currentcontrol){
		LOG_Iabc1_1 = twin_data->twin_inv1.Iabc[0];
		LOG_Iabc1_2 = twin_data->twin_inv1.Iabc[1];
		LOG_Iabc1_3 = twin_data->twin_inv1.Iabc[3];
		LOG_Iabc2_1 = twin_data->twin_inv2.Iabc[0];
		LOG_Iabc2_2 = twin_data->twin_inv2.Iabc[1];
		LOG_Iabc2_3 = twin_data->twin_inv2.Iabc[2];
		LOG_Iabc3_1 = twin_data->twin_inv3.Iabc[0];
		LOG_Iabc3_2 = twin_data->twin_inv3.Iabc[1];
		LOG_Iabc3_3 = twin_data->twin_inv3.Iabc[2];

		LOG_Itq_d = twin_data->tq.Idq0[0];
		LOG_Itq_q = twin_data->tq.Idq0[1];

		LOG_Is1_d = twin_data->s1.Idq0[0];
		LOG_Is1_q = twin_data->s1.Idq0[1];

		LOG_Is2_d = twin_data->s2.Idq0[0];
		LOG_Is2_q = twin_data->s2.Idq0[2];
	}



	//CRAMB_ctxt *cramb = get_CRAMB_ctxt();
	//CRAMB_Callback(cramb);
/*
	LOG_x_star = cramb->mc->controller_x.r_star;
	LOG_y_star = cramb->mc->controller_y.r_star;
	LOG_z_star = cramb->mc->controller_z.r_star;
	LOG_Fx_star = cramb->mc->Fxyz_star[0];
	LOG_Fy_star = cramb->mc->Fxyz_star[1];
	LOG_Fz_star = cramb->mc->Fxyz_star[2];
	LOG_Ix_star = cramb->cc_rad->cc_x.r_star;
	LOG_Iy_star = cramb->cc_rad->cc_y.r_star;
	LOG_Iz_star = cramb->cc_ax->cc_z.r_star;
	LOG_Va_star = cramb->cc_rad->Vabc_star[0];
	LOG_Vb_star = cramb->cc_rad->Vabc_star[1];
	LOG_Vc_star = cramb->cc_rad->Vabc_star[2];
	LOG_Vz_star = cramb->cc_ax->V_star;

	LOG_x = cramb->mc->xyz[0];
	LOG_y = cramb->mc->xyz[1];
	LOG_z = cramb->mc->xyz[2];

	LOG_Ix = cramb->cc_rad->Ixy[0];
	LOG_Iy = cramb->cc_rad->Ixy[1];
	LOG_Iz = cramb->cc_ax->I;

	LOG_xf = cramb->mc->xyz_filtered[0];
	LOG_yf = cramb->mc->xyz_filtered[1];
	LOG_zf = cramb->mc->xyz_filtered[2];

	LOG_Ixf = cramb->cc_rad->Ixy_filtered[0];
	LOG_Iyf = cramb->cc_rad->Ixy_filtered[1];
	LOG_Izf = cramb->cc_ax->I_filtered;*/
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
