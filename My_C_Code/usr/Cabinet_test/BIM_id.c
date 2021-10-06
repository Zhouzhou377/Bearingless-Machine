#include "sys/injection.h"
#include "usr/Cabinet_test/outloop_control.h"
#include "usr/Cabinet_test/BIM_id.h"
#include "sys/util.h"
#include "drv/cpu_timer.h"
#include <math.h>

static double t = 0.0;
static uint32_t time = 0;

// Injection contexts for controller commands
inj_ctx_t inj_ctx_ctrl[8] = { 0 };
void BIM_injection_init (void){
	injection_ctx_init(&inj_ctx_ctrl[0], "vd_ref");
	injection_ctx_init(&inj_ctx_ctrl[1], "vq_ref");

	injection_ctx_init(&inj_ctx_ctrl[2], "vx_ref");
	injection_ctx_init(&inj_ctx_ctrl[3], "vy_ref");

	injection_ctx_init(&inj_ctx_ctrl[4], "id_ref");
	injection_ctx_init(&inj_ctx_ctrl[5], "iq_ref");

	injection_ctx_init(&inj_ctx_ctrl[6], "ix_ref");
	injection_ctx_init(&inj_ctx_ctrl[7], "iy_ref");


	 // Register all signal injection points
    for (int i = 0; i < ARRAY_SIZE(inj_ctx_ctrl); i++) {
        injection_ctx_register(&inj_ctx_ctrl[i]);
    }
	t = 0.0;
	time = 0;
}
void BIM_injection_callback (bim_control *data){
	injection_inj(&(data->current_control->tq.vdq0_ref[0]), &inj_ctx_ctrl[0], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.vdq0_ref[1]), &inj_ctx_ctrl[1], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.vdq0_ref[0]), &inj_ctx_ctrl[2], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.vdq0_ref[1]), &inj_ctx_ctrl[3], data->current_control->s1.PI_regulator->Ts);

	injection_inj(&(data->current_control->tq.Idq0_ref[0]), &inj_ctx_ctrl[4], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.Idq0_ref[1]), &inj_ctx_ctrl[5], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.Idq0_ref[0]), &inj_ctx_ctrl[6], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.Idq0_ref[1]), &inj_ctx_ctrl[7], data->current_control->s1.PI_regulator->Ts);

}

void BIM_injection_sin(double w, double mag, double *theta, double *out, int Num_var){
	uint32_t time_now =  cpu_timer_now();
	double t_delta;
	if (time == 0 && t == 0.0){
		t = 0.0;
		time = time_now;
		t_delta = 0.0;
	}else if(time_now>time){
		
		t_delta = cpu_timer_ticks_to_sec(time_now-time);
		time = time_now;
	}else{
		
		t_delta = cpu_timer_ticks_to_sec(time_now +(0xFFFF-time));
		time = time_now;
	}
	t = t + t_delta;
	int i=0;
	for(i=0; i<Num_var; i++){
		out[i] = mag*cos(w*t+theta[i]);
	}

}
