#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/control_structure.h"
#include "usr/Machine_Control/BP3/bp3_outloop_control.h"
#include "usr/Machine_Control/task_cabinet.h"

#include "usr/Machine_Control/inverter.h"
#include "usr/Machine_Control/BP3/bp3_id.h"
#include "usr/Machine_Control//BP3/bp3_para_machine.h"
#include "usr/Machine_Control/currentloop_control.h"
#include "drv/pwm.h"
#include "drv/encoder.h"
#include "drv/fpga_timer.h"
#include "drv/eddy_current_sensor.h"
#include "usr/Machine_Control/definitions.h"
#include "usr/Machine_Control/analog_sensor.h"
#include "usr/Machine_Control/mb_sensor.h"
#include "usr/Machine_Control/controllers.h"
#include "usr/Machine_Control/transforms.h"
#include "sys/scheduler.h"
#include <math.h>
#include <stdbool.h>
//#include "drv/cpu_timer.h"
#include <stdint.h>


#define POSITION_RATIO_X (1.0)
#define POSITION_RATIO_y (1.2252)
#define GEO_CENTER_X (+8.0566e-6-6e-6)
#define GEO_CENTER_y (-1.7557e-4)
#define TS_v  (0.0001)


bp3_control bp3_control_data;

static double theta_OPL = 0.0;
static int flag_enc = 0;


void bp3_get_deltaxy_mes(bp3_control* data){
    data->bp3_lev_control.delta_mes[0] = POSITION_RATIO_X*(eddy_current_sensor_read_x_voltage(EDDY_CURRENT_SENSOR_3_BASE_ADDR) * M_PER_VOLT) + GEO_CENTER_X;
    data->bp3_lev_control.delta_mes[1] = POSITION_RATIO_y*(eddy_current_sensor_read_y_voltage(EDDY_CURRENT_SENSOR_3_BASE_ADDR) * M_PER_VOLT) + GEO_CENTER_y;
}



bp3_control *init_bp3(void){
    //init regulators
    encoder_set_pulses_per_rev_bits(ENCODER_BP3_PPR_BITS);
    para_bp3 *BP3_PARA;
    BP3_PARA = get_para_bp3();
    flag_enc = 0;
    bp3_control_data.bp3_lev_control.enable = 0;
    bp3_control_data.bp3_lev_control.para_levi_control.Ts = TS;
    bp3_control_data.bp3_lev_control.para_levi_control.ba = BP3_PARA->para_control.lev_ba;
    bp3_control_data.bp3_lev_control.para_levi_control.ka = BP3_PARA->para_control.lev_ka;

    bp3_control_data.bp3_lev_control.para_levi_control.para_PI.Ts = TS;
    bp3_control_data.bp3_lev_control.para_levi_control.para_PI.Ki = BP3_PARA->para_control.lev_pi_Ki;
    bp3_control_data.bp3_lev_control.para_levi_control.para_PI.Kp = BP3_PARA->para_control.lev_pi_Kp;

    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.Ts = TS;
    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.fs = BP3_PARA->para_control.lev_lpf_f;

    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.Ts = TS;
    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.fs = BP3_PARA->para_control.lev_delta_lpf_f;

    bp3_control_data.bp3_lev_control.para_levi_control.para_anti_wp.k = BP3_PARA->para_control.lev_antiwp_k;
    bp3_control_data.bp3_lev_control.para_levi_control.para_anti_wp.sat_high = BP3_PARA->para_control.lev_sat_high;
    bp3_control_data.bp3_lev_control.para_levi_control.para_anti_wp.sat_low = BP3_PARA->para_control.lev_sat_low;

    bp3_control_data.bp3_v_control.enable = 0;
    bp3_control_data.bp3_v_control.is_start = 0;
    bp3_control_data.bp3_v_control.para_velocity_control.wrm_max = BP3_PARA->para_machine.wrm_max;
    bp3_control_data.bp3_v_control.para_velocity_control.para_PI.Ts = TS;
    bp3_control_data.bp3_v_control.para_velocity_control.para_PI.Ki = BP3_PARA->para_control.v_pi_Ki;
    bp3_control_data.bp3_v_control.para_velocity_control.para_PI.Kp = BP3_PARA->para_control.v_pi_Kp;

    bp3_control_data.bp3_v_control.para_ob.Ts = TS;
    bp3_control_data.bp3_v_control.para_ob.Kj = 1.0/BP3_PARA->para_machine.J;
    bp3_control_data.bp3_v_control.para_ob.Kd = PI2*BP3_PARA->para_control.ob_theta_fd;
    bp3_control_data.bp3_v_control.para_ob.para_PI.Ts = TS;
    bp3_control_data.bp3_v_control.para_ob.para_PI.Kp = bp3_control_data.bp3_v_control.para_ob.Kd*BP3_PARA->para_machine.J*PI2*BP3_PARA->para_control.ob_theta_fp;
    bp3_control_data.bp3_v_control.para_ob.para_PI.Ki = bp3_control_data.bp3_v_control.para_ob.para_PI.Kp*PI2*BP3_PARA->para_control.ob_theta_fi;


    bp3_control_data.bp3_v_control.para_velocity_control.para_lpf.Ts = TS;
    bp3_control_data.bp3_v_control.para_velocity_control.para_lpf.fs = BP3_PARA->para_control.v_lpf_f;

    bp3_control_data.bp3_v_control.Idq0_ref[0] = BP3_PARA->para_machine.id_ref;
    bp3_control_data.bp3_v_control.wrm_ref = 0.0;
    bp3_control_data.bp3_v_control.wrm_ref_lpf = 0.0;
    bp3_control_data.bp3_v_control.Ts = TS;

    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_PI.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_velocity_control.para_PI.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_velocity_control.para_lpf.state_1));
    bp3_control_data.bp3_lev_control.delta_mes_lpf[0] = 0.0;
    bp3_control_data.bp3_lev_control.delta_mes_lpf[1] = 0.0;
    bp3_control_data.bp3_lev_control.diff_state[0] = 0.0;
    bp3_control_data.bp3_lev_control.diff_state[1] = 0.0;
    
    bp3_control_data.bp3_v_control.CFO_state =  0.0;
    bp3_control_data.bp3_v_control.time_pre =  0;
    bp3_control_data.bp3_v_control.theta_rm_mes =  0.0;
    bp3_control_data.bp3_v_control.step_pre = 0;

    bp3_control_data.bp3_v_control.para_ob.state1 = 0.0;
    bp3_control_data.bp3_v_control.para_ob.state2 = 0.0;
    bp3_control_data.bp3_v_control.para_ob.enable = 0;
    double theta = get_encoder_pos();
    bp3_control_data.bp3_v_control.theta_rm_est = 1.0*theta + PI2;
    //bp3_control_data.bp3_v_control.theta_rm_est = PI2;
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_ob.para_PI.state_1));

    bp3_control_data.current_control = init_currentloop();
    bp3_control_data.current_control->sel_config = InvFour;
    reset_regulator();
    bp3_control_data.is_init = 1;

    bp3_control_data.BP3_PARA = BP3_PARA;

    injection_ctx_clear(&inj_ctx_ctrl_bp3);

    bp3_control_data.bp3_lev_control.delta_ref[0] = 0.0;
    bp3_control_data.bp3_lev_control.delta_ref[1] = 0.0;

    reset_states_3phase(&(bp3_control_data.bp3_lev_control.Ixy0_ref[0]));

    bp3_control_data.bp3_v_control.wrm_ref = 0.0;
    reset_states_3phase(&(bp3_control_data.bp3_v_control.Idq0_ref[0]));
    bp3_control_data.bp3_v_control.Idq0_ref[0] = 1.0;
    bp3_get_deltaxy_mes(&bp3_control_data);
    bp3_control_data.bp3_lev_control.delta_ref[0] = bp3_control_data.bp3_lev_control.delta_mes[0]*0.95;
    bp3_control_data.bp3_lev_control.delta_ref[1] = bp3_control_data.bp3_lev_control.delta_mes[1]*0.95;

    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1[0] = bp3_control_data.bp3_lev_control.delta_mes[0]*0.95;
    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1[1] = bp3_control_data.bp3_lev_control.delta_mes[1]*0.95;
    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1[0] = bp3_control_data.bp3_lev_control.delta_ref[0];
    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1[1] = bp3_control_data.bp3_lev_control.delta_ref[1];

    bp3_control_data.bp3_lev_control.F_xy[0] = 0.0;
    bp3_control_data.bp3_lev_control.F_xy[1] = 0.0;

    bp3_control_data.bp3_v_control.enable_encoder = 0;

    cmd_enable.enable_openloop = 0;
	cmd_enable.enable_current_control = 0;
	cmd_enable.enable_bp3_control = 0;
	cmd_enable.enable_bp3_align = 0;
	cmd_enable.enable_log = 0;
	cmd_enable.enable_inject_tq_cctrl= 0;
    cmd_enable.enable_inject_s1_cctrl= 0;
    cmd_enable.enable_inject_Fxy= 0;
    cmd_enable.enable_inject_tq_vref= 0;
    cmd_enable.enable_inject_s1_vref= 0;



    return &(bp3_control_data);
}

bp3_control *reset_bp3(void){
    if (!bp3_control_data.is_init)
	{
		init_bp3();
	}
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_PI.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_velocity_control.para_PI.state_1));
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_velocity_control.para_lpf.state_1));
    bp3_control_data.bp3_lev_control.delta_mes_lpf[0] = 0.0;
    bp3_control_data.bp3_lev_control.delta_mes_lpf[1] = 0.0;
    bp3_control_data.bp3_lev_control.diff_state[0] = 0.0;
    bp3_control_data.bp3_lev_control.diff_state[1] = 0.0;
    
    bp3_control_data.bp3_v_control.CFO_state =  0.0;
    bp3_control_data.bp3_v_control.time_pre =  0;
    bp3_control_data.bp3_v_control.theta_rm_mes =  0.0;
    bp3_control_data.bp3_v_control.step_pre = 0;
    bp3_control_data.bp3_v_control.para_ob.enable = 0;
    bp3_control_data.bp3_v_control.para_ob.state1 = 0.0;
    bp3_control_data.bp3_v_control.para_ob.state2 = 0.0;
    bp3_control_data.bp3_v_control.theta_rm_est = 0.0;
    reset_states_3phase(&(bp3_control_data.bp3_v_control.para_ob.para_PI.state_1));

    reset_regulator();
    injection_ctx_clear(&inj_ctx_ctrl_bp3);

    bp3_control_data.bp3_lev_control.delta_ref[0] = 0.0;
    bp3_control_data.bp3_lev_control.delta_ref[1] = 0.0;

    reset_states_3phase(&(bp3_control_data.bp3_lev_control.Ixy0_ref[0]));

    bp3_control_data.bp3_v_control.wrm_ref = 0.0;
    bp3_control_data.bp3_v_control.wrm_ref_lpf = 0.0;
    reset_states_3phase(&(bp3_control_data.bp3_v_control.Idq0_ref[0]));
    bp3_control_data.bp3_v_control.Idq0_ref[0] = bp3_control_data.BP3_PARA->para_machine.id_ref;
    reset_states_3phase(&(bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1));
    bp3_get_deltaxy_mes(&bp3_control_data);
    bp3_control_data.bp3_lev_control.delta_ref[0] = bp3_control_data.bp3_lev_control.delta_mes[0]*0.95;
    bp3_control_data.bp3_lev_control.delta_ref[1] = bp3_control_data.bp3_lev_control.delta_mes[1]*0.95;

    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1[0] = bp3_control_data.bp3_lev_control.delta_mes[0]*0.95;
    bp3_control_data.bp3_lev_control.para_levi_control.para_lpf.state_1[1] = bp3_control_data.bp3_lev_control.delta_mes[1]*0.95;
    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1[0] = bp3_control_data.bp3_lev_control.delta_ref[0];
    bp3_control_data.bp3_lev_control.para_levi_control.para_delta_lpf.state_1[1] = bp3_control_data.bp3_lev_control.delta_ref[1];

    bp3_control_data.bp3_lev_control.F_xy[0] = 0.0;
    bp3_control_data.bp3_lev_control.F_xy[1] = 0.0;

    bp3_control_data.bp3_v_control.enable_encoder = 0;

    cmd_enable.enable_openloop = 0;
	cmd_enable.enable_current_control = 0;
	cmd_enable.enable_inject_tq_cctrl= 0;
    cmd_enable.enable_inject_s1_cctrl= 0;
    cmd_enable.enable_inject_Fxy= 0;
    cmd_enable.enable_inject_tq_vref= 0;
    cmd_enable.enable_inject_s1_vref= 0;


    return &(bp3_control_data);
}


bp3_control *deinit_bp3(void){


    reset_bp3();
    bp3_control_data.current_control= deinit_currentloop();
    bp3_control_data.is_init = 0;
	cmd_enable.enable_bp3_align = 0;
	cmd_enable.enable_bp3_control = 0;
	cmd_enable.enable_log = 0;
	bp3_control_data.bp3_v_control.is_start = 0;
    // init machine control parameters
    return &bp3_control_data;

}

// need to get eddy current sensor and encoder signals
static uint32_t time_begin;
static uint32_t time_end;
static int count;
void bm_start_theta(bp3_control* data){

    if (pwm_is_enabled()){
    	if (!data->bp3_v_control.enable_encoder){
    		if(!flag_enc){
    			time_begin = cpu_timer_now();
    			count = 0;
    			flag_enc = 1;
    		}

			double amp = 1;
			double freq = 1;
			double omega = PI2*freq;
			double v_OPL[3];
			theta_OPL += (data->bp3_v_control.Ts*omega);
			theta_OPL = fmod(theta_OPL, PI2);
			v_OPL[0] = amp * cos(theta_OPL);
			v_OPL[1] = amp * cos(theta_OPL - PI23);
			v_OPL[2] = amp * cos(theta_OPL - 2.0 * PI23);

			set_line_volts_three_phase(v_OPL[0], v_OPL[1], v_OPL[2], data->current_control->c_loop_inv1.inv);

			time_end = cpu_timer_now();
			double delta_time = cpu_timer_ticks_to_sec(time_end-time_begin);
			count ++;
			if(count>=10000){
				data->bp3_v_control.enable_encoder = 1;
				count = 0;
				}

    	}
    	else{
			set_line_volts_three_phase(2, -1, -1, data->current_control->c_loop_inv1.inv);

			time_begin = cpu_timer_now();
			time_end = time_begin;
			count ++;
			while(count<=20000){
				count ++;
				time_end = cpu_timer_now;
				data->bp3_v_control.theta_rm_mes_offset = get_encoder_pos();
			}
			data->bp3_v_control.theta_rm_mes_offset = get_encoder_pos();
			double theta = get_encoder_pos();
			data->bp3_v_control.theta_re_ref = 1.0*(theta - data->bp3_v_control.theta_rm_mes_offset)*data->BP3_PARA->para_machine.p;
			data->bp3_v_control.is_start = 1;}}
	   else{
			data->bp3_v_control.is_start = 0;
			data->bp3_v_control.enable_encoder = 1;
			count = 0;
			}
}

void bp3_velocity_regulation(bp3_control* data){
    double antiwp = 0.0;
    //get_pos_w_mes(data);
    //get_pos_w_mes(data->bp3_v_control.theta_rm_mes, &(data->bp3_v_control.theta_rm_mes_pre), &(data->bp3_v_control.wrm_mes));
    //data->bp3_v_control.theta_rm_mes_pre = data->bp3_v_control.theta_rm_mes;
    //func_lpf(&(data->bp3_v_control.wrm_ref), &(data->bp3_v_control.wrm_ref_lpf), 1, &(data->bp3_v_control.para_velocity_control.para_lpf));
    func_lpf(&(data->bp3_v_control.wrm_ref), &(data->bp3_v_control.wrm_ref_lpf), &(data->bp3_v_control.para_velocity_control.para_lpf), &(data->bp3_v_control.para_velocity_control.para_lpf.state_1[0]));
    data->bp3_v_control.wrm_ref_lpf += data->bp3_v_control.wrm_ref_inject;
    data->bp3_v_control.err_wrm = data->bp3_v_control.wrm_ref_lpf - data->bp3_v_control.wrm_mes;
    func_PI_normal(&(data->bp3_v_control.err_wrm), &(data->bp3_v_control.Te_ref), 1, &(data->bp3_v_control.para_velocity_control.para_PI), &(antiwp));
    /*
    double Ts = data->bp3_v_control.Ts;
    double state_p;
    double state_i;

    
        state_p = data->bp3_v_control.para_velocity_control.para_PI.Kp*data->bp3_v_control.err_wrm;
        double state;
        state = data->bp3_v_control.err_wrm;
        state_i = data->bp3_v_control.para_velocity_control.para_PI.Ki*Ts*state+ data->bp3_v_control.para_velocity_control.para_PI.state_1[0];
        data->bp3_v_control.para_velocity_control.para_PI.state_1[0] = state_i;
        data->bp3_v_control.Te_ref = state_i*1.0 + state_p;
    */


   //func_PI_normal(&(data->bp3_v_control.err_wrm), &(data->bp3_v_control.Te_ref), 1, &(data->bp3_v_control.para_velocity_control.para_PI), &antiwp);

}

void bp3_levitation_regulation(bp3_control* data){
    double id = data->bp3_v_control.Idq0_ref[0];
    update_para_bp3_activedamping(data->BP3_PARA, id);
    data->bp3_lev_control.para_levi_control.ba = data->BP3_PARA->para_control.lev_ba;
    data->bp3_lev_control.para_levi_control.ka = data->BP3_PARA->para_control.lev_ka;
    data->bp3_lev_control.para_levi_control.para_PI.Ki = data->BP3_PARA->para_control.lev_pi_Ki;
    data->bp3_lev_control.para_levi_control.para_PI.Kp = data->BP3_PARA->para_control.lev_pi_Kp;

    func_lpf(&(data->bp3_lev_control.delta_ref[0]), &(data->bp3_lev_control.delta_ref_lpf[0]), &(data->bp3_lev_control.para_levi_control.para_delta_lpf), &(data->bp3_lev_control.para_levi_control.para_delta_lpf.state_1[0]));
    func_lpf(&(data->bp3_lev_control.delta_ref[1]), &(data->bp3_lev_control.delta_ref_lpf[1]), &(data->bp3_lev_control.para_levi_control.para_delta_lpf), &(data->bp3_lev_control.para_levi_control.para_delta_lpf.state_1[1]));

    data->bp3_lev_control.err_delta[0] = data->bp3_lev_control.delta_ref_lpf[0] - data->bp3_lev_control.delta_mes[0];
    data->bp3_lev_control.err_delta[1] = data->bp3_lev_control.delta_ref_lpf[1] - data->bp3_lev_control.delta_mes[1];

    double pi_out[2];

    func_PI_normal(&(data->bp3_lev_control.err_delta), &(pi_out), 2, &(data->bp3_lev_control.para_levi_control.para_PI), &data->bp3_lev_control.out_antiwp);

    func_lpf(&(data->bp3_lev_control.delta_mes[0]), &(data->bp3_lev_control.delta_mes_lpf[0]), &(data->bp3_lev_control.para_levi_control.para_lpf), &(data->bp3_lev_control.para_levi_control.para_lpf.state_1[0]));
    func_lpf(&(data->bp3_lev_control.delta_mes[1]), &(data->bp3_lev_control.delta_mes_lpf[1]), &(data->bp3_lev_control.para_levi_control.para_lpf), &(data->bp3_lev_control.para_levi_control.para_lpf.state_1[1]));
    double fs = 1/data->bp3_lev_control.para_levi_control.Ts;
    double kf_inv = 1/data->BP3_PARA->para_machine.kf;
    double state_ka[2];
    double state_diff[2];
    double state_buf[2];
    double out[3];
    for(int i=0; i<2; i++){
        state_ka[i] = data->bp3_lev_control.para_levi_control.ka* data->bp3_lev_control.delta_mes[i];
    
        state_buf[i] = data->bp3_lev_control.delta_mes_lpf[i]*data->bp3_lev_control.para_levi_control.ba;
        state_diff[i] = (state_buf[i] - data->bp3_lev_control.diff_state[i])*fs;
        out[i] = (pi_out[i] - state_ka[i] - state_diff[i]);
        
        data->bp3_lev_control.diff_state[i] = state_buf[i];
    }
    out[2] = 0.0;

    out[1] = out[1] + 15;

    //data->bp3_lev_control.F_xy[0] = out[0];
    //data->bp3_lev_control.F_xy[1] = out[1];
    //out[0] = out[0] + data->bp3_lev_control.F_xy[0];
    //out[1] = out[1] + data->bp3_lev_control.F_xy[1];


/*
    double F_mag;
    double theta_F;
    F_mag = sqrt(out[0]*out[0]+out[1]*out[1]);
    if(out[0]==0 && out[1]>=0){
    	theta_F = PI/2.0;
    }else if(out[0]==0 && out[1]<0){
    	theta_F = -1.0*PI/2.0;

    }else if(out[0]>0){
    	theta_F = atan(out[1]/out[0]);
    }else{
    	theta_F = atan(out[1]/out[0])+PI;
    }
    out[0] = F_mag*cos(2*theta_F);
    out[1] = F_mag*sin(2*theta_F);*/


    out[0] =  out[0]+ data->bp3_lev_control.F_xy[0];
    out[1] =  out[1] +data->bp3_lev_control.F_xy[1];
    out[0] =  out[0]+ data->bp3_lev_control.F_xy_inject[0];
    out[1] =  out[1] +data->bp3_lev_control.F_xy_inject[1];

    //data->bp3_lev_control.F_xy_out[0] = out[0];
    //data->bp3_lev_control.F_xy_out[1] = out[1];
    data->bp3_lev_control.F_xy_out[0] = pi_out[0];
    data->bp3_lev_control.F_xy_out[1] = pi_out[1];


    for(int i = 0; i<2; i++){
    	if(data->BP3_PARA->para_machine.kf != 0){
    	            out[i] = out[i]*kf_inv;
    	        }else{
    	            out[i] = 0.0;
    	        }
    }
    data->bp3_lev_control.Ixy0_log[0] = out[0];
    data->bp3_lev_control.Ixy0_log[1] = out[1];
    double theta_rad;
       double out_xy[3];
       theta_rad = data->BP3_PARA->para_machine.kf_theta_rad;
       exp_jtheta(theta_rad, &(out[0]), &(out_xy[0]));

       out[0] = out_xy[0];
       out[1] = out_xy[1];
    double out_antiwp_xy[3];

    theta_rad = data->bp3_v_control.theta_re_ref*(0.0)+data->BP3_PARA->para_machine.theta_offset_xy;
    //func_Park(&out, &out_xy, theta_rad);

    func_anti_windup(&(data->bp3_lev_control.para_levi_control.para_anti_wp), out[0], &(out_xy[0]), &(out_antiwp_xy[0]));
    func_anti_windup(&(data->bp3_lev_control.para_levi_control.para_anti_wp), out[1], &(out_xy[1]), &(out_antiwp_xy[1]));

    
    out_antiwp_xy[2] = 0.0;
    out_xy[2] = 0.0;

    //func_Park_inverse(&(out_xy[0]), &(data->bp3_lev_control.Ixy0_ref[0]), theta_rad);
    exp_jtheta(theta_rad,&(out_xy[0]), &(data->bp3_lev_control.Ixy0_ref[0]));
    data->bp3_lev_control.Ixy0_ref[2] = 0.0;
}


void bp3_controlloop (bp3_control* data)
{
    if(!data->is_init || data == 0x00000000){
        data = init_bp3();
    }
    
    //theta_pre = data->bp3_v_control.theta_rm_mes;
    //update eddy current position
    bp3_get_deltaxy_mes(data);
    //update theta and we
    //data->theta_rm_mes_pre = data->bp3_v_control.theta_rm_mes;
    ////data->bp3_v_control.w_test = data->bp3_v_control.theta_rm_mes_pre;
    if(!data->bp3_v_control.is_start){
    	bm_start_theta(data);
    			}
    //double theta = get_encoder_pos();
    //data->bp3_v_control.theta_rm_mes = 1.0*(theta);
    data->bp3_v_control.theta_re_ref = 1.0*(data->bp3_v_control.theta_rm_mes - data->bp3_v_control.theta_rm_mes_offset)*data->BP3_PARA->para_machine.p;
    while(data->bp3_v_control.theta_re_ref<0||data->bp3_v_control.theta_re_ref>PI2){
            if(data->bp3_v_control.theta_re_ref<0){
                data->bp3_v_control.theta_re_ref+=PI2;
            }else{
                data->bp3_v_control.theta_re_ref-=PI2;
            }
        }
    if (data->bp3_v_control.para_ob.enable == 1){
        double tq;
        tq = 1.5*data->BP3_PARA->para_machine.p*data->BP3_PARA->para_machine.Lm/data->BP3_PARA->para_machine.Lr*data->current_control->tq.Idq0_ref[0]*data->BP3_PARA->para_machine.Lm*data->current_control->tq.Idq0_ref[1];
        func_observer_theta(data->bp3_v_control.theta_rm_mes, &(data->bp3_v_control.theta_rm_est), &(data->bp3_v_control.wrm_est), &(data->bp3_v_control.wrm_est_hf), &(data->bp3_v_control.para_ob), tq);
        data->bp3_v_control.wrm_mes = data->bp3_v_control.wrm_est;
    }else{
        data->bp3_v_control.wrm_mes = 0.0;
    }
    
    //data->bp3_v_control.w_test = data->theta_rm_mes_pre;
   //
   //get_all_inverter_current_abc(data->current_control);
   //get_all_inverter_Vdc(data->current_control);

   //protection
   // w
   if(data->bp3_v_control.wrm_mes>=data->BP3_PARA->para_machine.wrm_max || data->bp3_v_control.wrm_mes<=(data->BP3_PARA->para_machine.wrm_max*-1.0)){
        pwm_disable();
        data = reset_bp3();

        return;
    }

    int flag_t = 0;
    int flag_s = 0;
    flag_t = protection_overcurrent(data->current_control->c_loop_inv1.Iabc, data->BP3_PARA->para_machine.iabc_max);
    flag_s = protection_overcurrent(data->current_control->c_loop_inv2.Iabc, data->BP3_PARA->para_machine.iabc_max);
    if(flag_t || flag_s){
        data = reset_bp3();
        pwm_disable();
        return;
    }


   // velocity control
   if(data->bp3_v_control.enable){
       if(DEBUG_DFLUX){
           //data->bp3_v_control.Te_ref = 0.0;
           /*wrm_test = 0.0*PI2;
           theta_test = theta_test + wrm_test*TS;
           data->bp3_v_control.wrm_mes = wrm_test;
           data->bp3_v_control.theta_rm_mes = theta_test;*/

       }else{
           if(ID_VCTRL){
    	    bp3_injection_callback(data);
            }
           bp3_velocity_regulation(data);
       }
   }
   data->bp3_v_control.Idq0_ref[1] = data->bp3_v_control.Te_ref/data->BP3_PARA->para_machine.kt;
    /*if(cmd_enable.enable_inject_Fxy){
        double theta[2];
        theta[0] = 0.0;
        theta[1] = PI*0.5;

        bp3_injection_sin(data->bp3_lev_control.w_inject, data->bp3_lev_control.mag_inject, &theta[0], &(data->bp3_lev_control.F_xy[0]), 2);
    }*/
    // levitation 
    if(data->bp3_lev_control.enable){
        if(ID_LEVCTRL){
    	    bp3_injection_callback(data);
        }
        bp3_levitation_regulation(data);
    }else{

        double theta_rad = data->bp3_v_control.theta_re_ref*(0.0) + data->BP3_PARA->para_machine.kf_theta_rad;
        double Ixy_ref[3];
        func_Park(&(data->bp3_lev_control.Ixy0_ref[0]), &(Ixy_ref[0]), theta_rad);
        data->current_control->s1.Idq0_ref[0] = Ixy_ref[0];
        data->current_control->s1.Idq0_ref[1] = Ixy_ref[1];
        data->current_control->s1.Idq0_ref[2] = Ixy_ref[2];
    }
   // transfer commands to current loop
   data->current_control->tq.we = data->bp3_v_control.wre_ref;
   data->current_control->tq.theta_rad = data->bp3_v_control.theta_re_ref;
   data->current_control->tq.Idq0_ref[0] = data->bp3_v_control.Idq0_ref[0];
   data->current_control->tq.Idq0_ref[1] = data->bp3_v_control.Idq0_ref[1];
   data->current_control->tq.Idq0_ref[2] = data->bp3_v_control.Idq0_ref[2];

   data->current_control->s1.we = data->bp3_v_control.wre_ref*(-1.0);
   data->current_control->s1.theta_rad = data->bp3_v_control.theta_re_ref*(-1.0); //data->BP3_PARA->para_machine.kf_theta_rad;
   data->current_control->s1.Idq0_ref[0] = data->bp3_lev_control.Ixy0_ref[0];
   data->current_control->s1.Idq0_ref[1] = data->bp3_lev_control.Ixy0_ref[1];
   data->current_control->s1.Idq0_ref[2] = data->bp3_lev_control.Ixy0_ref[2];


   if(ID_CCTRL){
    	bp3_injection_callback(data);
        data->current_control->tq.Idq0_ref[0] += data->current_control->tq.Idq0_ref_inject[0];
        data->current_control->tq.Idq0_ref[1] += data->current_control->tq.Idq0_ref_inject[1];
        data->current_control->s1.Idq0_ref[0] += data->current_control->s1.Idq0_ref_inject[0];
        data->current_control->s1.Idq0_ref[1] += data->current_control->s1.Idq0_ref_inject[1];
        /*double theta[4];
        theta[0] = 0.0;
        theta[1] = PI*0.5;
        theta[2] = 0.0;
        theta[3] = PI*0.5;
        double out[4];
        bp3_injection_sin(data->bp3_lev_controlw_inject, data->bp3_lev_controlmag_inject, &theta[0], &(out[0]), 4);
        data->current_control->tq.Idq0_ref[0] = data->current_control->tq.Idq0_ref[0] + out[0];
        data->current_control->tq.Idq0_ref[1] = data->current_control->tq.Idq0_ref[1] + out[1];
        data->current_control->s1.Idq0_ref[0] = data->current_control->s1.Idq0_ref[0] + out[0];
        data->current_control->s1.Idq0_ref[1] = data->current_control->s1.Idq0_ref[1] + out[1];*/
        }
   if(ID_SYS){
	   bp3_injection_callback(data);
       }
   current_regulation (data->current_control);
    //theta_pre =data->bp3_v_control.theta_rm_mes;
}
