#include "sys/injection.h"
#include "usr/Machine_Control/BIM/bim_outloop_control.h"
#include "usr/Machine_Control/BIM/bim_id.h"
#include "sys/util.h"
#include "drv/cpu_timer.h"
#include <math.h>

static double t = 0.0;
static uint32_t time = 0;

// Injection contexts for controller commands
inj_ctx_t inj_ctx_ctrl[19] = { 0 };
void bim_injection_init (void){
	injection_ctx_init(&inj_ctx_ctrl[0], "vd_ref");
	injection_ctx_init(&inj_ctx_ctrl[1], "vq_ref");

	injection_ctx_init(&inj_ctx_ctrl[2], "vx_ref");
	injection_ctx_init(&inj_ctx_ctrl[3], "vy_ref");

	injection_ctx_init(&inj_ctx_ctrl[4], "id_ref");
	injection_ctx_init(&inj_ctx_ctrl[5], "iq_ref");

	injection_ctx_init(&inj_ctx_ctrl[6], "ix_ref");
	injection_ctx_init(&inj_ctx_ctrl[7], "iy_ref");

	injection_ctx_init(&inj_ctx_ctrl[8], "wrm_ref");

	injection_ctx_init(&inj_ctx_ctrl[9], "Fx_ref");
	injection_ctx_init(&inj_ctx_ctrl[10], "Fy_ref");

	injection_ctx_init(&inj_ctx_ctrl[11], "vd2_ref");
	injection_ctx_init(&inj_ctx_ctrl[12], "vq2_ref");

	injection_ctx_init(&inj_ctx_ctrl[13], "vx2_ref");
	injection_ctx_init(&inj_ctx_ctrl[14], "vy2_ref");

	injection_ctx_init(&inj_ctx_ctrl[15], "id2_ref");
	injection_ctx_init(&inj_ctx_ctrl[16], "iq2_ref");

	injection_ctx_init(&inj_ctx_ctrl[17], "ix2_ref");
	injection_ctx_init(&inj_ctx_ctrl[18], "iy2_ref");

	 // Register all signal injection points
    for (int i = 0; i < ARRAY_SIZE(inj_ctx_ctrl); i++) {
        injection_ctx_register(&inj_ctx_ctrl[i]);
    }
	t = 0.0;
	time = 0;
}
void bim_injection_callback (bim_control *data){
	injection_inj(&(data->current_control->tq.vdq0_ref_inject[0]), &inj_ctx_ctrl[0], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.vdq0_ref_inject[1]), &inj_ctx_ctrl[1], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.vdq0_ref_inject[0]), &inj_ctx_ctrl[2], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.vdq0_ref_inject[1]), &inj_ctx_ctrl[3], data->current_control->s1.PI_regulator->Ts);

	injection_inj(&(data->current_control->tq.Idq0_ref_inject[0]), &inj_ctx_ctrl[4], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.Idq0_ref_inject[1]), &inj_ctx_ctrl[5], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.Idq0_ref_inject[0]), &inj_ctx_ctrl[6], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.Idq0_ref_inject[1]), &inj_ctx_ctrl[7], data->current_control->s1.PI_regulator->Ts);

	injection_inj(&(data->bim_v_control.wrm_ref_inject), &inj_ctx_ctrl[8], data->bim_v_control.Ts);

	injection_inj(&(data->bim_lev_control.F_xy_inject[0]), &inj_ctx_ctrl[9], data->bim_v_control.Ts);
	injection_inj(&(data->bim_lev_control.F_xy_inject[1]), &inj_ctx_ctrl[10], data->bim_v_control.Ts);

	injection_inj(&(data->current_control->tq2.vdq0_ref_inject[0]), &inj_ctx_ctrl[11], data->current_control->tq2.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq2.vdq0_ref_inject[1]), &inj_ctx_ctrl[12], data->current_control->tq2.PI_regulator->Ts);

	injection_inj(&(data->current_control->s2.vdq0_ref_inject[0]), &inj_ctx_ctrl[13], data->current_control->s2.PI_regulator->Ts);
	injection_inj(&(data->current_control->s2.vdq0_ref_inject[1]), &inj_ctx_ctrl[14], data->current_control->s2.PI_regulator->Ts);

	injection_inj(&(data->current_control->tq2.Idq0_ref_inject[0]), &inj_ctx_ctrl[15], data->current_control->tq2.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq2.Idq0_ref_inject[1]), &inj_ctx_ctrl[16], data->current_control->tq2.PI_regulator->Ts);

	injection_inj(&(data->current_control->s2.Idq0_ref_inject[0]), &inj_ctx_ctrl[17], data->current_control->s2.PI_regulator->Ts);
	injection_inj(&(data->current_control->s2.Idq0_ref_inject[1]), &inj_ctx_ctrl[18], data->current_control->s2.PI_regulator->Ts);

}

void bim_injection_sin(double w, double mag, double *theta, double *out, int Num_var){
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
