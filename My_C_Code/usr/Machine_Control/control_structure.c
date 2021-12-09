#include "drv/encoder.h"
#include "usr/Machine_Control/control_structure.h"
#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/definitions.h"
#include <math.h>
#include <stdbool.h>
//#include "drv/cpu_timer.h"
#include <stdint.h>

#define TS_v  (0.0001)

void reset_states_3phase (double *state){
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

double get_encoder_pos(void){
    uint32_t position;
    encoder_get_position(&position);

    double theta_now;

    position =  position&0x0000003FF; //0x0000003FF
    theta_now = PI2 * ( (double) position / (double) (ENCODER_BP3_PPR));

}

void get_pos_w_mes(double theta_now, double *theta_pre, double *w){
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
    //double w;


   // theta_now = fmod(theta_now, PI2);

    /*if(data->bim_v_control.wrm_mes>=0 && theta_now<data->bim_v_control.theta_rm_mes){
    	dtheta = theta_now + PI - data->bim_v_control.theta_rm_mes_pre;
    }else if(data->bim_v_control.wrm_mes<0 && theta_now>data->bim_v_control.theta_rm_mes){
    	dtheta = theta_now - PI - data->bim_v_control.theta_rm_mes_pre;
    }else{
    	dtheta = theta_now - data->bim_v_control.theta_rm_mes_pre;
    }*/
    //double theta_now = data->bim_v_control.theta_rm_mes;
    dtheta = theta_now - theta_pre[0];
    double dtheta_abs;
    if(dtheta<0){
    	dtheta_abs = -1.0*dtheta;
    }else{
    	dtheta_abs = 1.0*dtheta;
    }
    if(dtheta_abs>=PI){
        if(theta_now>PI && (theta_pre[0])<=PI){
            dtheta = dtheta - PI;
        }else{
            dtheta = dtheta + PI;
        }

    }

    *w = dtheta/TS_v;
    theta_pre[1] = theta_pre[0];
    theta_pre[0] = theta_now;

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
    //bim_control_data.bim_v_control.wrm_mes = w;
    //data->bim_v_control.theta_rm_mes_pre = data->bim_v_control.theta_rm_mes;
    //data->bim_v_control.theta_rm_mes = theta_now;
    //data->theta_rm_mes_pre[1] = theta_now;
    //data->bim_v_control.time_pre = now;
    //data->bim_v_control.step_pre = steps;

}

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

void func_observer_theta(double theta_mes, double *theta_est, double *w_est, double *w_est_hf, para_observer *para_ob, double tq){
    double error;
    error = theta_mes - *theta_est;
    if (error<-1.0*PI){
        error = (error+PI2);
    }else if(error>PI){
        error = (error-PI2);
    }
    double out_PI;
    double antiwp = 0.0;
    func_PI_normal(&error, &out_PI, 1, &(para_ob->para_PI), &antiwp);
    out_PI += tq;
    double out_Kd;
    out_Kd = error*para_ob->Kd;

    *w_est = out_PI*para_ob->Ts*para_ob->Kj + para_ob->state1;
    *w_est_hf = *w_est + out_Kd;
    *theta_est = fmod((para_ob->Ts*(*w_est_hf) + para_ob->state2), PI2);
    para_ob->state1 = *w_est;
    para_ob->state2 = *theta_est;


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

