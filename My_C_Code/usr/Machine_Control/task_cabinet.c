#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/openloop_vsi.h"
#include "usr/Machine_Control/inverter.h"
#include "usr/Machine_Control/task_cabinet.h"
#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/currentloop_control.h"
#include "usr/Machine_Control/BIM/bim_outloop_control.h"
#include "usr/Machine_Control//BIM/bim_para_machine.h"
#include "usr/Machine_Control/BIM/bim_id.h"
#include "usr/Machine_Control/BIM/bim_log.h"
#include "usr/Machine_Control/BP3/bp3_outloop_control.h"
#include "usr/Machine_Control//BP3/bp3_para_machine.h"
#include "usr/Machine_Control/BP3/bp3_id.h"
#include "usr/Machine_Control/BP3/bp3_log.h"

#include "sys/scheduler.h"
#include "sys/debug.h"
#include "drv/cpu_timer.h"
#include "drv/pwm.h"



currentloop_control *c_loop_data;
//bim_control bim_control_data;
bim_control *bim_control_pt;

bp3_control *bp3_control_pt;

OpenLoop_Command VSI_Openloop_command;
OpenLoop_Command *OpenLoop;

cmd_signal cmd_enable;
inj_ctx_t *ctx = &inj_ctx_ctrl[0];
//#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static task_control_block_t tcb;

//register and begin task
void task_cabinet_init(void)
{
	/*if (task_cabinet_is_inited){
		return;
	}*/
	//populate struct
	scheduler_tcb_init(&tcb, task_cabinet_callback, NULL, "cabinet", TASK_CABINET_INTERVAL_USEC);
	scheduler_tcb_register(&tcb);

	default_inverter_setup(0);
	OpenLoop = init_OpenLoop_Command();
	if(!BM_ENABLE){
		c_loop_data = init_currentloop();
	}else{
		bim_control_pt = init_bim();
	}
	bim_injection_init();
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_current_control = 0;
	cmd_enable.enable_bim_control = 0;
	cmd_enable.enable_bp3_control = 0;
	cmd_enable.enable_log = 0;
	cmd_enable.enable_inject_tq_cctrl= 0;
    cmd_enable.enable_inject_s1_cctrl= 0;
    cmd_enable.enable_inject_Fxy= 0;
    cmd_enable.enable_inject_tq_vref= 0;
    cmd_enable.enable_inject_s1_vref= 0;
}

//stop task
void task_cabinet_deinit(void)
{
	
	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_current_control = 0;
	cmd_enable.enable_bim_control = 0;
	cmd_enable.enable_bp3_control = 0;
	cmd_enable.enable_log = 0;
	cmd_enable.enable_inject_tq_cctrl= 0;
    cmd_enable.enable_inject_s1_cctrl= 0;
    cmd_enable.enable_inject_Fxy= 0;
    cmd_enable.enable_inject_tq_vref= 0;
    cmd_enable.enable_inject_s1_vref= 0;
	scheduler_tcb_unregister(&tcb);
	c_loop_data = deinit_currentloop();
	injection_ctx_unregister(ctx);
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
	if(cmd_enable.enable_testloop){
			task_loop_test();
		}
	if (cmd_enable.enable_openloop){
		OpenLoop = &VSI_Openloop_command;
		OpenLoop_VSI(OpenLoop);
		inv = get_three_phase_inverter(OpenLoop->Num_inv);
		set_line_volts_three_phase(OpenLoop->command_volatge[0], OpenLoop->command_volatge[1], OpenLoop->command_volatge[2], inv);
	}
	else if(cmd_enable.enable_current_control)
	{
		c_loop_data = &c_loop_control;
		current_regulation(c_loop_data);
		 if(c_loop_data->sel_config == InvFour){
			set_line_volts_three_phase(c_loop_data->c_loop_inv1.vabc_ref[0], c_loop_data->c_loop_inv1.vabc_ref[1], c_loop_data->c_loop_inv1.vabc_ref[2], c_loop_data->c_loop_inv1.inv);
			set_line_volts_three_phase(c_loop_data->c_loop_inv2.vabc_ref[0], c_loop_data->c_loop_inv2.vabc_ref[1], c_loop_data->c_loop_inv2.vabc_ref[2], c_loop_data->c_loop_inv2.inv);
			set_line_volts_three_phase(c_loop_data->c_loop_inv3.vabc_ref[0], c_loop_data->c_loop_inv3.vabc_ref[1], c_loop_data->c_loop_inv3.vabc_ref[2], c_loop_data->c_loop_inv3.inv);
			set_line_volts_three_phase(c_loop_data->c_loop_inv4.vabc_ref[0], c_loop_data->c_loop_inv4.vabc_ref[1], c_loop_data->c_loop_inv4.vabc_ref[2], c_loop_data->c_loop_inv4.inv);

		 }else{
			set_line_volts_three_phase(c_loop_data->c_loop_inv1.vabc_ref[0], c_loop_data->c_loop_inv1.vabc_ref[1], c_loop_data->c_loop_inv1.vabc_ref[2], c_loop_data->c_loop_inv1.inv);
			set_line_volts_three_phase(c_loop_data->c_loop_inv2.vabc_ref[0], c_loop_data->c_loop_inv2.vabc_ref[1], c_loop_data->c_loop_inv2.vabc_ref[2], c_loop_data->c_loop_inv2.inv);
			set_line_volts_three_phase(c_loop_data->c_loop_inv3.vabc_ref[0], c_loop_data->c_loop_inv3.vabc_ref[1], c_loop_data->c_loop_inv3.vabc_ref[2], c_loop_data->c_loop_inv3.inv);
		}

	}
	else if(cmd_enable.enable_bim_control){
		bim_control_pt = &bim_control_data;
		if(pwm_is_enabled()){
			bim_controlloop(bim_control_pt);
		}
		//theta_pre = bim_control_data.bim_v_control.theta_rm_mes;
		//bim_controlloop(bim_control_pt);
		//bim_control_pt = &bim_control_data;
		
		set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv1.vabc_ref[0], bim_control_pt->current_control->c_loop_inv1.vabc_ref[1], bim_control_pt->current_control->c_loop_inv1.vabc_ref[2], bim_control_pt->current_control->c_loop_inv1.inv);
		set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv2.vabc_ref[0], bim_control_pt->current_control->c_loop_inv2.vabc_ref[1], bim_control_pt->current_control->c_loop_inv2.vabc_ref[2], bim_control_pt->current_control->c_loop_inv2.inv);


	}

	else if(cmd_enable.enable_bp3_control){
		bp3_control_pt = &bp3_control_data;
		if(pwm_is_enabled()){
			bp3_controlloop(bp3_control_pt);
		}
		//theta_pre = bim_control_data.bim_v_control.theta_rm_mes;
		//bim_controlloop(bim_control_pt);
		//bim_control_pt = &bim_control_data;
		
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv1.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv1.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv1.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv1.inv);
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv2.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv2.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv2.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv2.inv);


	}



	if (cmd_enable.enable_log){
		if(!BM_ENABLE){
		c_loop_data = &c_loop_control;
		//log three phase inv current
		/*LOG_Iabc1_a = c_loop_data->c_loop_inv1.Iabc[0];
		LOG_Iabc1_b = c_loop_data->c_loop_inv1.Iabc[1];
		LOG_Iabc1_c = c_loop_data->c_loop_inv1.Iabc[2];
		LOG_Iabc2_a = c_loop_data->c_loop_inv2.Iabc[0];
		LOG_Iabc2_b = c_loop_data->c_loop_inv2.Iabc[1];
		LOG_Iabc2_c = c_loop_data->c_loop_inv2.Iabc[2];
		LOG_Iabc3_a = c_loop_data->c_loop_inv3.Iabc[0];
		LOG_Iabc3_b = c_loop_data->c_loop_inv3.Iabc[1];
		LOG_Iabc3_c = c_loop_data->c_loop_inv3.Iabc[2];

		LOG_Ia1_a = c_loop_data->c_loop_inv1.Iabc[0]*0.5 + c_loop_data->c_loop_inv2.Iabc[0];
		LOG_Ia1_b = c_loop_data->c_loop_inv1.Iabc[1]*0.5 + c_loop_data->c_loop_inv2.Iabc[1];
		LOG_Ia1_c = c_loop_data->c_loop_inv1.Iabc[2]*0.5 + c_loop_data->c_loop_inv2.Iabc[2];
		LOG_Ib1_a = c_loop_data->c_loop_inv1.Iabc[0]*0.5;
		LOG_Ib1_b = c_loop_data->c_loop_inv1.Iabc[1]*0.5;
		LOG_Ib1_c = c_loop_data->c_loop_inv1.Iabc[2]*0.5;
		LOG_Ia2_a = c_loop_data->c_loop_inv1.Iabc[0]*0.5 + c_loop_data->c_loop_inv3.Iabc[0];
		LOG_Ia2_b = c_loop_data->c_loop_inv1.Iabc[1]*0.5 + c_loop_data->c_loop_inv3.Iabc[1];
		LOG_Ia2_c = c_loop_data->c_loop_inv1.Iabc[2]*0.5 + c_loop_data->c_loop_inv3.Iabc[2];
		LOG_Ib2_a = c_loop_data->c_loop_inv1.Iabc[0]*0.5;
		LOG_Ib2_b = c_loop_data->c_loop_inv1.Iabc[1]*0.5;
		LOG_Ib2_c = c_loop_data->c_loop_inv1.Iabc[2]*0.5;*/
		//log torque current

		/*LOG_Itq_d_ref = c_loop_data->tq.Idq0_ref[0];
		LOG_Itq_q_ref = c_loop_data->tq.Idq0_ref[1];

		LOG_Itq_d = c_loop_data->tq.Idq0[0];
		LOG_Itq_q = c_loop_data->tq.Idq0[1];

		LOG_Is1_d_ref = c_loop_data->s1.Idq0_ref[0];
		LOG_Is1_q_ref = c_loop_data->s1.Idq0_ref[1];

		LOG_Is1_d = c_loop_data->s1.Idq0[0];
		LOG_Is1_q = c_loop_data->s1.Idq0[1];

		LOG_Is2_d_ref = c_loop_data->s2.Idq0_ref[0];
		LOG_Is2_q_ref = c_loop_data->s2.Idq0_ref[1];

		LOG_Is2_d = c_loop_data->s2.Idq0[0];
		LOG_Is2_q = c_loop_data->s2.Idq0[1];

		if(c_loop_data->sel_config == InvFour){
			LOG_Itq2_d_ref = c_loop_data->tq2.Idq0_ref[0];
			LOG_Itq2_q_ref = c_loop_data->tq2.Idq0_ref[1];

			LOG_Itq2_d = c_loop_data->tq2.Idq0[0];
			LOG_Itq2_q = c_loop_data->tq2.Idq0[1];
		}

		/*LOG_vabc1_a = c_loop_data->c_loop_inv1.vabc_ref[0];
		LOG_vabc1_b = c_loop_data->c_loop_inv1.vabc_ref[1];
		LOG_vabc1_c = c_loop_data->c_loop_inv1.vabc_ref[2];
		LOG_vabc2_a = c_loop_data->c_loop_inv2.vabc_ref[0];
		LOG_vabc2_b = c_loop_data->c_loop_inv2.vabc_ref[1];
		LOG_vabc2_c = c_loop_data->c_loop_inv2.vabc_ref[2];
		LOG_vabc3_a = c_loop_data->c_loop_inv3.vabc_ref[0];
		LOG_vabc3_b = c_loop_data->c_loop_inv3.vabc_ref[1];
		LOG_vabc3_c = c_loop_data->c_loop_inv3.vabc_ref[2];

		LOG_err_tq_d = c_loop_data->tq.error[0];
		LOG_err_tq_q = c_loop_data->tq.error[1];

		LOG_err_s1_d = c_loop_data->s1.error[0];
		LOG_err_s1_q = c_loop_data->s1.error[1];

		LOG_err_s2_d = c_loop_data->s2.error[0];
		LOG_err_s2_q = c_loop_data->s2.error[1];

		LOG_theta_tq = c_loop_data->tq.theta_rad;
		LOG_theta_s1 = c_loop_data->s1.theta_rad;
		LOG_theta_s2 = c_loop_data->s2.theta_rad;

		LOG_vr_tq_d = c_loop_data->tq.PR_regulator->v_PR[0];
		LOG_vr_tq_q = c_loop_data->tq.PR_regulator->v_PR[1];

		LOG_v_tq_d = c_loop_data->tq.vdq0_ref[0];
		LOG_v_tq_q = c_loop_data->tq.vdq0_ref[1];
		LOG_we_tq = c_loop_data->tq.we;*/
		}else{
			if(BIM_ENABLE){
				bim_log (bim_control_pt);
			}
			else if(BP3_ENABLE){
				bp3_log (bp3_control_pt);
			}
			

		}

	}


}
uint32_t time_testloop;
void task_loop_test(void){
	c_loop_data = &c_loop_control;
	uint32_t time_now = cpu_timer_now();
		double time_test = cpu_timer_ticks_to_sec(time_now - time_testloop);
		if (time_test<0.05){
			if(c_loop_data->sel_config == InvFour){
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 0.0;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			c_loop_data->tq2.Idq0_ref[0] = 0.0;
			c_loop_data->tq2.Idq0_ref[1] = 0.0;
			c_loop_data->tq2.Idq0_ref[2] = 0.0;
			}else{
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 0.0;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			}
			c_loop_data->s1.Idq0_ref[0] = 0.0;
			c_loop_data->s1.Idq0_ref[1] = 0.0;
			c_loop_data->s1.Idq0_ref[2] = 0.0;
			c_loop_data->s2.Idq0_ref[0] = 0.0;
			c_loop_data->s2.Idq0_ref[1] = 0.0;
			c_loop_data->s2.Idq0_ref[2] = 0.0;
		}else if(0.05<=time_test && time_test<0.1){
			if(c_loop_data->sel_config == InvFour){
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 1.5;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			c_loop_data->tq2.Idq0_ref[0] = 0.0;
			c_loop_data->tq2.Idq0_ref[1] = 1.5;
			c_loop_data->tq2.Idq0_ref[2] = 0.0;
			}else{
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 3.0;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			}
			c_loop_data->s1.Idq0_ref[0] = 0.0;
			c_loop_data->s1.Idq0_ref[1] = 0.0;
			c_loop_data->s1.Idq0_ref[2] = 0.0;
			c_loop_data->s2.Idq0_ref[0] = 0.0;
			c_loop_data->s2.Idq0_ref[1] = 0.0;
			c_loop_data->s2.Idq0_ref[2] = 0.0;
		}else if(0.1<=time_test && time_test<0.15){
			if(c_loop_data->sel_config == InvFour){
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 1.5;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			c_loop_data->tq2.Idq0_ref[0] = 0.0;
			c_loop_data->tq2.Idq0_ref[1] = 1.5;
			c_loop_data->tq2.Idq0_ref[2] = 0.0;
			}else{
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 3.0;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			}
			c_loop_data->s1.Idq0_ref[0] = 1.0;
			c_loop_data->s1.Idq0_ref[1] = 0.0;
			c_loop_data->s1.Idq0_ref[2] = 0.0;
			c_loop_data->s2.Idq0_ref[0] = 0.0;
			c_loop_data->s2.Idq0_ref[1] = 0.0;
			c_loop_data->s2.Idq0_ref[2] = 0.0;
		}else if(0.15<=time_test && time_test<0.2){
			if(c_loop_data->sel_config == InvFour){
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 1.5;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			c_loop_data->tq2.Idq0_ref[0] = 0.0;
			c_loop_data->tq2.Idq0_ref[1] = 1.5;
			c_loop_data->tq2.Idq0_ref[2] = 0.0;
			}else{
			c_loop_data->tq.Idq0_ref[0] = 0.0;
			c_loop_data->tq.Idq0_ref[1] = 3.0;
			c_loop_data->tq.Idq0_ref[2] = 0.0;
			}
			c_loop_data->s1.Idq0_ref[0] = 1.0;
			c_loop_data->s1.Idq0_ref[1] = 0.0;
			c_loop_data->s1.Idq0_ref[2] = 0.0;
			c_loop_data->s2.Idq0_ref[0] = 0.0;
			c_loop_data->s2.Idq0_ref[1] = 0.5;
			c_loop_data->s2.Idq0_ref[2] = 0.0;
	
		}else{
			time_testloop = time_now;
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
