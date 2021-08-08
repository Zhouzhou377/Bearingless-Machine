#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Cabinet_test/task_cabinet.h"
#include "usr/Cabinet_test/current_control.h"

#include "sys/scheduler.h"
#include "sys/debug.h"

//LOGGING VARIABLES
double LOG_x_star = 0.0;
double LOG_y_star = 0.0;
double LOG_z_star = 0.0;
double LOG_Fx_star = 0.0;
double LOG_Fy_star = 0.0;
double LOG_Fz_star = 0.0;
double LOG_Ix_star = 0.0;
double LOG_Iy_star = 0.0;
double LOG_Iz_star = 0.0;
double LOG_Va_star = 0.0;
double LOG_Vb_star = 0.0;
double LOG_Vc_star = 0.0;
double LOG_Vz_star = 0.0;

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

#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static task_control_block_t tcb;

//register and begin task
void task_cabinet_init(void)
{
	//populate struct
	scheduler_tcb_init(&tcb, task_cabinet_callback, NULL, "cabinet", TASK_CABINET_INTERVAL_USEC);
	scheduler_tcb_register(&tcb);
}

//stop task
void task_cabinet_deinit(void)
{
	scheduler_tcb_unregister(&tcb);
	disable_all_cc();
	//(get_mc());

}

//function to determine if task has been started
uint8_t task_cabinet_is_inited(void)
{
	return scheduler_tcb_is_registered(&tcb);
}

//call back function to be run repeatedly
void task_cabinet_callback(void *arg)
{
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
