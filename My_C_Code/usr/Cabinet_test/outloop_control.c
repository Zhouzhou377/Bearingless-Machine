
#include "usr/Cabinet_test/outloop_control.h"
#include "usr/Cabinet_test/hardware_machine.h"
#include "usr/Cabinet_test/twinbearingless_control.h"
#include "drv/pwm.h"
#include "drv/encoder.h"
#include "drv/fpga_timer.h"
#include "drv/eddy_current_sensor.h"
#include "usr/Cabinet_test/definitions.h"
#include "usr/Cabinet_test/analog_sensor.h"
#include "usr/Cabinet_test/mb_sensor.h"
#include "usr/Cabinet_test/controllers.h"
#include "usr/Cabinet_test/transforms.h"
#include "usr/Cabinet_test/definitions.h"
#include "sys/scheduler.h"
#include <math.h>
#include <stdbool.h>
//#include "drv/cpu_timer.h"
#include <stdint.h>

#define ENCODER_BP3_PPR_BITS (10)
#define ENCODER_BP3_PPR      (1 << ENCODER_BP3_PPR_BITS)
#define M_PER_VOLT (4e-4)
#define POSITION_RATIO (1.0)
#define GEO_CENTER_X (0.0)
#define GEO_CENTER_y (0.0)
#define TS_v  (0.0001)

#define DEBUG_DFLUX (0)

bim_control bim_control_data;
volatile double theta_pre = 0.0;


void reset_states_3phase (double *state){
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void get_deltaxy_mes(bim_control* data){
    data->bim_lev_control.delta_mes[0] = POSITION_RATIO*(eddy_current_sensor_read_x_voltage() * M_PER_VOLT) + GEO_CENTER_X;
    data->bim_lev_control.delta_mes[1] = POSITION_RATIO*(eddy_current_sensor_read_y_voltage() * M_PER_VOLT) + GEO_CENTER_y;
}

double get_encoder_pos(void){
    uint32_t position;
    encoder_get_position(&position);

    double theta_now;

    position =  position&0x0000003FF;
    theta_now = PI2 * ( (double) position / (double) (ENCODER_BP3_PPR));

}

void get_pos_w_mes(bim_control* data){
	//uint32_t position = 0;
    //uint32_t now;

    /*uint32_t time_test_int = fpga_timer_now();
    double time_test = fpga_timer_ticks_to_sec(time_test_int);*/

	// Get delta position and speed
	//now = fpga_timer_now();
	//encoder_get_position(&position);
	//now = cpu_timer_now();
	//int32_t steps;
    //int32_t dsteps;
	//encoder_get_steps(&steps);
    //dsteps = steps - data->bim_v_control.step_pre;
    //int32_t dstep_abs;
/*
    if(dsteps <=0){
        dstep_abs = -1*dsteps;
    }else{
        dstep_abs = dsteps;
    }

    double dsteps_rad = PI2 * ( (double) dsteps / (double) (ENCODER_BP3_PPR));
    

    

	//double steps_rad = PI2 * ( (double) steps / (double) (ENCODER_BP3_PPR));
		// Add d axis offset
	//position += mo.d_offset;
	uint32_t dt_int;
     
    if(now<data->bim_v_control.time_pre){
        dt_int = 0xFFFFFFFF-data->bim_v_control.time_pre;
        dt_int = dt_int + now;
    }else{
        dt_int = now - data->bim_v_control.time_pre;
    }
	double dt;
	dt = fpga_timer_ticks_to_sec(dt_int);
	// dt = cpu_timer_ticks_to_sec(dt_int);
	
    /*uint32_t dpos;
    dpos = position - data->bim_v_control.pos_pre;*/
    //data->bim_v_control.pos_pre = position;
    /*while (position >= ENCODER_BP3_PPR) {
        position -= ENCODER_BP3_PPR;
    }*/
    
	// Convert to radians
    /*double theta_now;

    position =  position&0x0000003FF;
    theta_now = PI2 * ( (double) position / (double) (ENCODER_BP3_PPR));*/
    double dtheta;
    //dtheta = fmod(dtheta, PI2);
    //dtheta = theta_now - data->bim_v_control.theta_rm_mes;
    double w;


   // theta_now = fmod(theta_now, PI2);

    /*if(data->bim_v_control.wrm_mes>=0 && theta_now<data->bim_v_control.theta_rm_mes){
    	dtheta = theta_now + PI - data->bim_v_control.theta_rm_mes_pre;
    }else if(data->bim_v_control.wrm_mes<0 && theta_now>data->bim_v_control.theta_rm_mes){
    	dtheta = theta_now - PI - data->bim_v_control.theta_rm_mes_pre;
    }else{
    	dtheta = theta_now - data->bim_v_control.theta_rm_mes_pre;
    }*/
    double theta_now = data->bim_v_control.theta_rm_mes;
    dtheta = data->bim_v_control.theta_rm_mes - data->bim_v_control.theta_rm_mes_pre;
    double dtheta_abs;
    if(dtheta<0){
    	dtheta_abs = -1.0*dtheta;
    }else{
    	dtheta_abs = 1.0*dtheta;
    }
    if(dtheta_abs>=PI){
        if(theta_now>PI && data->bim_v_control.theta_rm_mes<=PI){
            dtheta = dtheta - PI;
        }else{
            dtheta = dtheta + PI;
        }
    	
    }

    w = dtheta/TS_v;
 
    /*double w_steps = dsteps_rad/TS_v;
        /*if(data->bim_v_control.wrm_mes!=0 && w!=0 && abs((w - data->bim_v_control.wrm_mes)/data->bim_v_control.wrm_mes)>=0.2){
        	w = data->bim_v_control.wrm_mes;
        }*/

    /*if(data->bim_v_control.theta_rm_mes == 0 && data->bim_v_control.time_pre == 0){
        data->bim_v_control.wrm_mes = 0.0;
    }else if(dt==0){
        ;
    }else if(dstep_abs>=0x8FFF0000){
    	;
    }
    else{
        data->bim_v_control.wrm_mes = w_steps;
    }*/
     //data->bim_v_control.w_test = theta_now - data->bim_v_control.theta_rm_mes;
    bim_control_data.bim_v_control.wrm_mes = w;
    //data->bim_v_control.theta_rm_mes_pre = data->bim_v_control.theta_rm_mes;
    //data->bim_v_control.theta_rm_mes = theta_now;
    //data->theta_rm_mes_pre[1] = theta_now;
    //data->bim_v_control.time_pre = now;
    //data->bim_v_control.step_pre = steps;
    
}

bim_control *init_bim(void){
    //init regulators
    encoder_set_pulses_per_rev_bits(ENCODER_BP3_PPR_BITS);
    para_bim *BIM_PARA;
    BIM_PARA = get_para_BIM();
    bim_control_data.bim_lev_control.enable = 0;
    bim_control_data.bim_lev_control.para_levi_control.Ts = TS;
    bim_control_data.bim_lev_control.para_levi_control.ba = BIM_PARA->para_control.lev_ba;
    bim_control_data.bim_lev_control.para_levi_control.ka = BIM_PARA->para_control.lev_ka;

    bim_control_data.bim_lev_control.para_levi_control.para_PI.Ts = TS;
    bim_control_data.bim_lev_control.para_levi_control.para_PI.Ki = BIM_PARA->para_control.lev_pi_Ki;
    bim_control_data.bim_lev_control.para_levi_control.para_PI.Kp = BIM_PARA->para_control.lev_pi_Kp;

    bim_control_data.bim_lev_control.para_levi_control.para_lpf.Ts = TS;
    bim_control_data.bim_lev_control.para_levi_control.para_lpf.fs = BIM_PARA->para_control.lev_lpf_f;

    bim_control_data.bim_lev_control.para_levi_control.para_anti_wp.k = BIM_PARA->para_control.lev_antiwp_k;
    bim_control_data.bim_lev_control.para_levi_control.para_anti_wp.sat_high = BIM_PARA->para_control.lev_sat_high;
    bim_control_data.bim_lev_control.para_levi_control.para_anti_wp.sat_low = BIM_PARA->para_control.lev_sat_low;

    bim_control_data.bim_v_control.enable = 0;
    bim_control_data.bim_v_control.para_velocity_control.wrm_max = BIM_PARA->para_machine.wrm_max;
    bim_control_data.bim_v_control.para_velocity_control.para_PI.Ts = TS;
    bim_control_data.bim_v_control.para_velocity_control.para_PI.Ki = BIM_PARA->para_control.v_pi_Ki;
    bim_control_data.bim_v_control.para_velocity_control.para_PI.Kp = BIM_PARA->para_control.v_pi_Kp;

    bim_control_data.bim_v_control.para_velocity_control.para_lpf.Ts = TS;
    bim_control_data.bim_v_control.para_velocity_control.para_lpf.fs = BIM_PARA->para_control.v_lpf_f;

    bim_control_data.bim_v_control.Idq0_ref[0] = BIM_PARA->para_machine.id_ref;
    bim_control_data.bim_v_control.Ts = TS;

    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.state_1));
    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.para_PI.state_1));
    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.para_lpf.state_1));
    reset_states_3phase(&(bim_control_data.bim_v_control.para_velocity_control.para_PI.state_1));
    reset_states_3phase(&(bim_control_data.bim_v_control.para_velocity_control.para_lpf.state_1));
    bim_control_data.bim_lev_control.delta_mes_lpf[0] = 0.0;
    bim_control_data.bim_lev_control.delta_mes_lpf[1] = 0.0;
    bim_control_data.bim_lev_control.diff_state[0] = 0.0;
    bim_control_data.bim_lev_control.diff_state[1] = 0.0;
    
    bim_control_data.bim_v_control.CFO_state =  0.0;
    bim_control_data.bim_v_control.time_pre =  0;
    bim_control_data.bim_v_control.theta_rm_mes =  0.0;
    bim_control_data.bim_v_control.step_pre = 0;

    bim_control_data.current_control = init_twinbearingless();
    bim_control_data.current_control->sel_config = InvFour;
    reset_regulator();
    bim_control_data.is_init = 1;

    bim_control_data.BIM_PARA = BIM_PARA;


    return &(bim_control_data);
}

bim_control *reset_bim(void){
    if (!bim_control_data.is_init)
	{
		init_bim();
	}
    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.state_1));
    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.para_PI.state_1));
    reset_states_3phase(&(bim_control_data.bim_lev_control.para_levi_control.para_lpf.state_1));
    reset_states_3phase(&(bim_control_data.bim_v_control.para_velocity_control.para_PI.state_1));
    reset_states_3phase(&(bim_control_data.bim_v_control.para_velocity_control.para_lpf.state_1));
    bim_control_data.bim_lev_control.delta_mes_lpf[0] = 0.0;
    bim_control_data.bim_lev_control.delta_mes_lpf[1] = 0.0;
    bim_control_data.bim_lev_control.diff_state[0] = 0.0;
    bim_control_data.bim_lev_control.diff_state[1] = 0.0;
    
    bim_control_data.bim_v_control.CFO_state =  0.0;
    bim_control_data.bim_v_control.time_pre =  0;
    bim_control_data.bim_v_control.theta_rm_mes =  0.0;
    bim_control_data.bim_v_control.step_pre = 0;
    reset_regulator();
    return &(bim_control_data);
}


bim_control *deinit_bim(void){

    bim_control_data.is_init = 0;
    reset_bim();
    bim_control_data.current_control= deinit_twinbearingless();
    // init machine control parameters
    return &bim_control_data;
}

// need to get eddy current sensor and encoder signals
void func_lpf(double *in, double *out, para_lpf *para_lpf, double *state){
    double t = 1/(PI2*para_lpf->fs);
    double a = 1/(t+para_lpf->Ts);
    double k1 = para_lpf->Ts*a;
    double k2 = t*a; 


    *out= *in*k1 + (*state)*k2;
    *state = *out;

}

void func_PI_normal(double *in, double *out, int Num_variable, para_PI_discrete_normal *para_PI, double *out_antiwp){
    double Ts = para_PI->Ts;
    double state_p;
    double state_i;

    for (int i = 0; i<Num_variable; i++){
        state_p = para_PI->Kp*(*(in+i));
        double state;
        state = *(in+i) + *(out_antiwp+i);
        state_i = para_PI->Ki*Ts*state+ para_PI->state_1[i];
        para_PI->state_1[i] = state_i;
        *(out+i) = state_i + state_p;
    }

}

void func_anti_windup(para_anti_windup *para_antiwp, double in, double *in_antiwp, double *out){
    double error;
    if (in>=para_antiwp->sat_high){
        *in_antiwp = para_antiwp->sat_high;
        error = *in_antiwp - in;
    }else if (in<=para_antiwp->sat_low){
        *in_antiwp = para_antiwp->sat_low;
        error = *in_antiwp - in;
    }else{
        *in_antiwp =  in;
        error = 0.0;
    }
    
    *out = error*para_antiwp->k;
}

void velocity_regulation(bim_control* data){
    double antiwp = 0.0;
    get_pos_w_mes(data);
    data->bim_v_control.theta_rm_mes_pre = data->bim_v_control.theta_rm_mes;
    //func_lpf(&(data->bim_v_control.wrm_ref), &(data->bim_v_control.wrm_ref_lpf), 1, &(data->bim_v_control.para_velocity_control.para_lpf));
    func_lpf(&(data->bim_v_control.wrm_ref), &(data->bim_v_control.wrm_ref_lpf), &(data->bim_v_control.para_velocity_control.para_lpf), &(data->bim_v_control.para_velocity_control.para_lpf.state_1[0]));

    data->bim_v_control.err_wrm = data->bim_v_control.wrm_ref_lpf - data->bim_v_control.wrm_mes;

    double Ts = data->bim_v_control.Ts;
    double state_p;
    double state_i;

    
        state_p = data->bim_v_control.para_velocity_control.para_PI.Kp*data->bim_v_control.err_wrm;
        double state;
        state = data->bim_v_control.err_wrm;
        state_i = data->bim_v_control.para_velocity_control.para_PI.Ki*Ts*state+ data->bim_v_control.para_velocity_control.para_PI.state_1[0];
        data->bim_v_control.para_velocity_control.para_PI.state_1[0] = state_i;
        data->bim_v_control.Te_ref = state_i + state_p;
    


   //func_PI_normal(&(data->bim_v_control.err_wrm), &(data->bim_v_control.Te_ref), 1, &(data->bim_v_control.para_velocity_control.para_PI), &antiwp);

}

void UFO(bim_control* data){
    double Te;
    
    data->bim_v_control.lamda_m_ref = data->BIM_PARA->para_machine.Lm*data->bim_v_control.Idq0_ref[0];
    if(data->bim_v_control.lamda_m_ref == 0.0){
        data->bim_v_control.Idq0_ref[1] = 0.0;
        
        data->bim_v_control.wsl_ref = 0.0;
    }else{
        double x = 1/data->bim_v_control.lamda_m_ref;
        Te = data->bim_v_control.Te_ref*0.66666666666666667/data->BIM_PARA->para_machine.p;
        Te = Te*data->BIM_PARA->para_machine.Lr/data->BIM_PARA->para_machine.Lm;
        data->bim_v_control.Idq0_ref[1] = Te*x;
        data->bim_v_control.wsl_ref = data->BIM_PARA->para_machine.Lm*data->bim_v_control.Idq0_ref[1];
        data->bim_v_control.wsl_ref = data->bim_v_control.wsl_ref*x/data->BIM_PARA->para_machine.tau_r;
    }
    data->bim_v_control.Idq0_ref[2] = 0.0;
}

void CFO(bim_control* data){
    double state_1;
    state_1 = data->bim_v_control.Ts*data->bim_v_control.wsl_ref + data->bim_v_control.CFO_state;
    data->bim_v_control.CFO_state = state_1;
    data->bim_v_control.theta_re_ref = data->bim_v_control.theta_rm_mes*data->BIM_PARA->para_machine.p;
    data->bim_v_control.theta_re_ref = data->bim_v_control.theta_re_ref + state_1;

    data->bim_v_control.theta_re_ref = fmod(data->bim_v_control.theta_re_ref, PI2);
    /*if (data->bim_v_control.theta_re_ref >= PI2){
        data->bim_v_control.theta_re_ref = data->bim_v_control.theta_re_ref-PI2;
    }*/

    data->bim_v_control.wre_ref = data->bim_v_control.wrm_mes*data->BIM_PARA->para_machine.p + data->bim_v_control.wsl_ref;
}

void levitation_regulation(bim_control* data){
    data->bim_lev_control.err_delta[0] = data->bim_lev_control.delta_ref[0] - data->bim_lev_control.delta_mes[0];
    data->bim_lev_control.err_delta[1] = data->bim_lev_control.delta_ref[1] - data->bim_lev_control.delta_mes[1];

    double pi_out[2];

    func_PI_normal(&(data->bim_lev_control.err_delta), &(pi_out), 2, &(data->bim_lev_control.para_levi_control.para_PI), &data->bim_lev_control.out_antiwp);

    func_lpf(&(data->bim_lev_control.delta_mes[0]), &(data->bim_lev_control.delta_mes_lpf[0]), &(data->bim_lev_control.para_levi_control.para_lpf), &(data->bim_lev_control.para_levi_control.para_lpf.state_1[0]));
    func_lpf(&(data->bim_lev_control.delta_mes[1]), &(data->bim_lev_control.delta_mes_lpf[1]), &(data->bim_lev_control.para_levi_control.para_lpf), &(data->bim_lev_control.para_levi_control.para_lpf.state_1[1]));
    double fs = 1/data->bim_lev_control.para_levi_control.Ts;
    double kf_inv = 1/data->BIM_PARA->para_machine.kf;
    double state_ka[2];
    double state_diff[2];
    double state_buf[2];
    double out[3];
    for(int i=0; i<2; i++){
        state_ka[i] = data->bim_lev_control.para_levi_control.ka* data->bim_lev_control.delta_mes[i];
    
        state_buf[i] = data->bim_lev_control.delta_mes_lpf[i]*data->bim_lev_control.para_levi_control.ba;
        state_diff[i] = (state_buf[i] - data->bim_lev_control.diff_state[i])*fs;


        out[i] = (pi_out[i] - state_ka[i] - state_diff[i])*kf_inv;
        data->bim_lev_control.diff_state[i] = state_buf[i];
    }
    out[2] = 0.0;


    double theta_rad;
    double out_xy[3];
    double out_antiwp_xy[3];

    theta_rad = data->bim_v_control.theta_re_ref*(-1.0);
    func_Park(&out, &out_xy, theta_rad);

    func_anti_windup(&(data->bim_lev_control.para_levi_control.para_anti_wp), out_xy[0], &(data->bim_lev_control.Ixy0_ref[0]), &(out_antiwp_xy[0]));
    func_anti_windup(&(data->bim_lev_control.para_levi_control.para_anti_wp), out_xy[1], &(data->bim_lev_control.Ixy0_ref[1]), &(out_antiwp_xy[1]));

    data->bim_lev_control.Ixy0_ref[2] = 0.0;
    out_antiwp_xy[2] = 0.0;

    func_Park_inverse(&(data->bim_lev_control.out_antiwp), &out_antiwp_xy, theta_rad);
}



int protection_overcurrent(double iabc[3], double i_max){
    int flag;
    flag = 0;
    for(int x = 0; x<3; x++){
        if(iabc[x]>=i_max || iabc[x]<=(i_max*-1.0)){
            flag = 1;
            break;
        }
    }
    return flag;
}


void bim_controlloop (bim_control* data)
{
    if(!data->is_init || data == 0x00000000){
        data = init_bim();
    }
    //theta_pre = data->bim_v_control.theta_rm_mes;
    //update eddy current position
    get_deltaxy_mes(data);
    //update theta and we
    //data->theta_rm_mes_pre = data->bim_v_control.theta_rm_mes;
    ////data->bim_v_control.w_test = data->bim_v_control.theta_rm_mes_pre;
    double theta = get_encoder_pos();
    data->bim_v_control.theta_rm_mes = theta;
    
    //data->bim_v_control.w_test = data->theta_rm_mes_pre;
   //
   get_all_inverter_current_abc(data->current_control);

   //protection
   // w
   /* if(data->bim_v_control.wrm_mes>=data->BIM_PARA->para_machine.wrm_max || data->bim_v_control.wrm_mes<=(data->BIM_PARA->para_machine.wrm_max*-1.0)){
        data = reset_bim();
        pwm_disable();
        return;
    }*/

    int flag_t = 0;
    int flag_s = 0;
    flag_t = protection_overcurrent(data->current_control->twin_inv1.Iabc, data->BIM_PARA->para_machine.iabc_max);
    flag_s = protection_overcurrent(data->current_control->twin_inv2.Iabc, data->BIM_PARA->para_machine.iabc_max);
    if(flag_t || flag_s){
        data = reset_bim();
        pwm_disable();
        return;
    }


   // velocity control
   if(data->bim_v_control.enable){
       if(DEBUG_DFLUX){
           //data->bim_v_control.Te_ref = 0.0;
       }else{
           velocity_regulation(data);
       }
       
        UFO(data);
        CFO(data);
   }
    

    // levitation 
    if(data->bim_lev_control.enable){
        levitation_regulation(data);
    }
   // transfer commands to current loop
   data->current_control->tq.we = data->bim_v_control.wre_ref;
   data->current_control->tq.theta_rad = data->bim_v_control.theta_re_ref;
   data->current_control->tq.Idq0_ref[0] = data->bim_v_control.Idq0_ref[0];
   data->current_control->tq.Idq0_ref[1] = data->bim_v_control.Idq0_ref[1];
   data->current_control->tq.Idq0_ref[2] = data->bim_v_control.Idq0_ref[2];

   data->current_control->s1.we = data->bim_v_control.wre_ref*(-1.0);
   data->current_control->s1.theta_rad = data->bim_v_control.theta_re_ref*(-1.0);
   data->current_control->s1.Idq0_ref[0] = data->bim_lev_control.Ixy0_ref[0];
   data->current_control->s1.Idq0_ref[1] = data->bim_lev_control.Ixy0_ref[1];
   data->current_control->s1.Idq0_ref[2] = data->bim_lev_control.Ixy0_ref[2];

   current_regulation (data->current_control);
    //theta_pre =data->bim_v_control.theta_rm_mes;
}
