#include "sys/injection.h"
#include "usr/Machine_Control/BIM/bim_outloop_control.h"
#include "usr/Machine_Control/BIM/bim_id.h"
#include "sys/util.h"
#include "drv/cpu_timer.h"
#include <math.h>


// Injection contexts for controller commands
inj_ctx_t inj_ctx_ctrl_bim[19] = { 0 };
void bim_injection_init (void){
	injection_ctx_init(&inj_ctx_ctrl_bim[0], "vd_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[1], "vq_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[2], "vx_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[3], "vy_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[4], "id_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[5], "iq_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[6], "ix_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[7], "iy_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[8], "wrm_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[9], "Fx_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[10], "Fy_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[11], "vd2_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[12], "vq2_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[13], "vx2_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[14], "vy2_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[15], "id2_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[16], "iq2_ref");

	injection_ctx_init(&inj_ctx_ctrl_bim[17], "ix2_ref");
	injection_ctx_init(&inj_ctx_ctrl_bim[18], "iy2_ref");

	 // Register all signal injection points
    for (int i = 0; i < ARRAY_SIZE(inj_ctx_ctrl_bim); i++) {
        injection_ctx_register(&inj_ctx_ctrl_bim[i]);
    }

}
void bim_injection_callback (bim_control *data){
	injection_inj(&(data->current_control->tq.vdq0_ref_inject[0]), &inj_ctx_ctrl_bim[0], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.vdq0_ref_inject[1]), &inj_ctx_ctrl_bim[1], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.vdq0_ref_inject[0]), &inj_ctx_ctrl_bim[2], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.vdq0_ref_inject[1]), &inj_ctx_ctrl_bim[3], data->current_control->s1.PI_regulator->Ts);

	injection_inj(&(data->current_control->tq.Idq0_ref_inject[0]), &inj_ctx_ctrl_bim[4], data->current_control->tq.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq.Idq0_ref_inject[1]), &inj_ctx_ctrl_bim[5], data->current_control->tq.PI_regulator->Ts);

	injection_inj(&(data->current_control->s1.Idq0_ref_inject[0]), &inj_ctx_ctrl_bim[6], data->current_control->s1.PI_regulator->Ts);
	injection_inj(&(data->current_control->s1.Idq0_ref_inject[1]), &inj_ctx_ctrl_bim[7], data->current_control->s1.PI_regulator->Ts);

	injection_inj(&(data->bim_v_control.wrm_ref_inject), &inj_ctx_ctrl_bim[8], data->bim_v_control.Ts);

	injection_inj(&(data->bim_lev_control.F_xy_inject[0]), &inj_ctx_ctrl_bim[9], data->bim_v_control.Ts);
	injection_inj(&(data->bim_lev_control.F_xy_inject[1]), &inj_ctx_ctrl_bim[10], data->bim_v_control.Ts);

	injection_inj(&(data->current_control->tq2.vdq0_ref_inject[0]), &inj_ctx_ctrl_bim[11], data->current_control->tq2.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq2.vdq0_ref_inject[1]), &inj_ctx_ctrl_bim[12], data->current_control->tq2.PI_regulator->Ts);

	injection_inj(&(data->current_control->s2.vdq0_ref_inject[0]), &inj_ctx_ctrl_bim[13], data->current_control->s2.PI_regulator->Ts);
	injection_inj(&(data->current_control->s2.vdq0_ref_inject[1]), &inj_ctx_ctrl_bim[14], data->current_control->s2.PI_regulator->Ts);

	injection_inj(&(data->current_control->tq2.Idq0_ref_inject[0]), &inj_ctx_ctrl_bim[15], data->current_control->tq2.PI_regulator->Ts);
	injection_inj(&(data->current_control->tq2.Idq0_ref_inject[1]), &inj_ctx_ctrl_bim[16], data->current_control->tq2.PI_regulator->Ts);

	injection_inj(&(data->current_control->s2.Idq0_ref_inject[0]), &inj_ctx_ctrl_bim[17], data->current_control->s2.PI_regulator->Ts);
	injection_inj(&(data->current_control->s2.Idq0_ref_inject[1]), &inj_ctx_ctrl_bim[18], data->current_control->s2.PI_regulator->Ts);

}


