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
#include "drv/encoder.h"
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
inj_ctx_t *ctx;
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
	//if(!BM_ENABLE){
		c_loop_data = init_currentloop();
	//}else{
	if(BIM_ENABLE){
		bim_control_pt = init_bim();
		bim_injection_init();}
	//}
	if(BP3_ENABLE){
		bp3_control_pt = init_bp3();
		bp3_injection_init ();}
	//bim_injection_init();

	cmd_enable.enable_openloop = 0;
	cmd_enable.enable_current_control = 0;
	cmd_enable.enable_bim_control = 0;
	cmd_enable.enable_bp3_control = 0;
	cmd_enable.enable_bp3_align = 0;
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
	cmd_enable.enable_bp3_align = 0;
	cmd_enable.enable_log = 0;
	cmd_enable.enable_inject_tq_cctrl= 0;
    cmd_enable.enable_inject_s1_cctrl= 0;
    cmd_enable.enable_inject_Fxy= 0;
    cmd_enable.enable_inject_tq_vref= 0;
    cmd_enable.enable_inject_s1_vref= 0;

	scheduler_tcb_unregister(&tcb);
	c_loop_data = deinit_currentloop();
	ctx = &inj_ctx_ctrl_bim[0];
	injection_ctx_unregister(ctx);
	ctx = &inj_ctx_ctrl_bp3[0];
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
		get_all_inverter_current_abc(c_loop_data);
		get_all_inverter_Vdc(c_loop_data);
			if(SENSE_V_ENABLE){
				c_loop_data->c_loop_inv1.inv->Vdc = c_loop_data->c_loop_inv1.Vdc_mes;
				c_loop_data->c_loop_inv2.inv->Vdc = c_loop_data->c_loop_inv2.Vdc_mes;
				c_loop_data->c_loop_inv3.inv->Vdc = c_loop_data->c_loop_inv3.Vdc_mes;
				c_loop_data->c_loop_inv4.inv->Vdc = c_loop_data->c_loop_inv4.Vdc_mes;
			}
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
	else if(cmd_enable.enable_bim_VFcontrol){
		bim_control_pt = &bim_control_data;
		double theta = get_encoder_pos();
    	bim_control_pt->bim_v_control.theta_rm_mes = -1.0*theta + PI2;
		get_all_inverter_current_abc(bim_control_pt->current_control);
		get_all_inverter_Vdc(bim_control_pt->current_control);
		if(SENSE_V_ENABLE){
			bim_control_pt->current_control->c_loop_inv1.inv->Vdc = bim_control_pt->current_control->c_loop_inv1.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv2.inv->Vdc = bim_control_pt->current_control->c_loop_inv2.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv3.inv->Vdc = bim_control_pt->current_control->c_loop_inv3.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv4.inv->Vdc = bim_control_pt->current_control->c_loop_inv4.Vdc_mes;
		}
		bim_vf(bim_control_pt);
		set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv1.vabc_ref[0], bim_control_pt->current_control->c_loop_inv1.vabc_ref[1], bim_control_pt->current_control->c_loop_inv1.vabc_ref[2], bim_control_pt->current_control->c_loop_inv1.inv);
		//set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv2.vabc_ref[0], bim_control_pt->current_control->c_loop_inv2.vabc_ref[1], bim_control_pt->current_control->c_loop_inv2.vabc_ref[2], bim_control_pt->current_control->c_loop_inv2.inv);


	}
	else if(cmd_enable.enable_bim_control){
		bim_control_pt = &bim_control_data;
		double theta = get_encoder_pos();
    	bim_control_pt->bim_v_control.theta_rm_mes = -1.0*theta + PI2;
		get_all_inverter_current_abc(bim_control_pt->current_control);
        get_all_inverter_Vdc(bim_control_pt->current_control);
		if(SENSE_V_ENABLE){
			bim_control_pt->current_control->c_loop_inv1.inv->Vdc = bim_control_pt->current_control->c_loop_inv1.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv2.inv->Vdc = bim_control_pt->current_control->c_loop_inv2.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv3.inv->Vdc = bim_control_pt->current_control->c_loop_inv3.Vdc_mes;
			bim_control_pt->current_control->c_loop_inv4.inv->Vdc = bim_control_pt->current_control->c_loop_inv4.Vdc_mes;
		}
		if(pwm_is_enabled()){
			//if(!bim_control_pt->bim_v_control.enable_encoder){
				//bim_start_theta(bim_control_pt);}
			//else{
				bim_controlloop(bim_control_pt);			
		}

		set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv1.vabc_ref[0], bim_control_pt->current_control->c_loop_inv1.vabc_ref[1], bim_control_pt->current_control->c_loop_inv1.vabc_ref[2], bim_control_pt->current_control->c_loop_inv1.inv);
		set_line_volts_three_phase(bim_control_pt->current_control->c_loop_inv2.vabc_ref[0], bim_control_pt->current_control->c_loop_inv2.vabc_ref[1], bim_control_pt->current_control->c_loop_inv2.vabc_ref[2], bim_control_pt->current_control->c_loop_inv2.inv);


	}
	else if(cmd_enable.enable_bp3_align){
		bp3_control_pt = &bp3_control_data;
		bm_start_theta(bp3_control_pt);
		double theta = get_encoder_pos();
    	bp3_control_pt->bp3_v_control.theta_rm_mes = 1.0*(theta);
		get_all_inverter_current_abc(bp3_control_pt->current_control);
        get_all_inverter_Vdc(bp3_control_pt->current_control);
		if(SENSE_V_ENABLE){
			bp3_control_pt->current_control->c_loop_inv1.inv->Vdc = bp3_control_pt->current_control->c_loop_inv1.Vdc_mes;
			bp3_control_pt->current_control->c_loop_inv2.inv->Vdc = bp3_control_pt->current_control->c_loop_inv2.Vdc_mes;
			bp3_control_pt->current_control->c_loop_inv3.inv->Vdc = bp3_control_pt->current_control->c_loop_inv3.Vdc_mes;
			bp3_control_pt->current_control->c_loop_inv4.inv->Vdc = bp3_control_pt->current_control->c_loop_inv4.Vdc_mes;
		}

		cmd_enable.enable_bp3_control = 0;
	}
	else if(cmd_enable.enable_bp3_control){
		cmd_enable.enable_bp3_align = 0;
		bp3_control_pt = &bp3_control_data;
		bp3_control_pt_1 = &bp3_control_data_1;
		double theta = get_encoder_pos();
    	bp3_control_pt->bp3_v_control.theta_rm_mes = 1.0*(theta);
		get_all_inverter_current_abc(bp3_control_pt->current_control);
        get_all_inverter_Vdc(bp3_control_pt->current_control);

		bp3_control_pt_1->bp3_v_control.theta_rm_mes = 1.0*(theta);
		get_all_inverter_current_abc(bp3_control_pt_1->current_control);
		get_all_inverter_Vdc(bp3_control_pt_1->current_control);
		if(pwm_is_enabled()){
			bp3_controlloop(bp3_control_pt);
			bp3_controlloop(bp3_control_pt_1);}
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv1.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv1.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv1.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv1.inv);
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv2.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv2.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv2.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv2.inv);
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv3.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv3.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv3.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv3.inv);
		set_line_volts_three_phase(bp3_control_pt->current_control->c_loop_inv4.vabc_ref[0], bp3_control_pt->current_control->c_loop_inv4.vabc_ref[1], bp3_control_pt->current_control->c_loop_inv4.vabc_ref[2], bp3_control_pt->current_control->c_loop_inv4.inv);

	}



	if (cmd_enable.enable_log){
		if(!BM_ENABLE){
		c_loop_data = &c_loop_control;
		c_loop_log (c_loop_data);
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
