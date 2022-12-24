#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/currentloop_control.h"
#include "usr/Machine_Control/BIM/bim_outloop_control.h"
#include "usr/Machine_Control/BIM/bim_id.h"
#include "usr/Machine_Control/BP3/bp3_outloop_control.h"
#include "usr/Machine_Control/BP3/bp3_id.h"
#include "usr/Machine_Control/para_machine_c_loop.h"
#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/analog_sensor.h"
#include "usr/Machine_Control/mb_sensor.h"
//#include "usr/Machine_Control/controllers.h"
#include "usr/Machine_Control/transforms.h"
#include "usr/Machine_Control/definitions.h"
#include <math.h>
#include <stdbool.h>

#define SINGLE_INV (0)

#define INV1 (3) //torque 1
#define INV2 (2) //suspension 1
#define INV3 (5) //torque 2
#define INV4 (6) //suspension 2

#define WD_TQ (2.0*PI*200.0)
#define WD_S1 (2.0*PI*500.0)
#define WD_S2 (2.0*PI*500.0)


para_PI_discrete PI_tq;
para_PI_discrete PI_s1;
para_PI_discrete PI_s2;
para_PI_discrete PI_tq2;
para_PR PR_tq;
para_PR PR_s1;
para_PR PR_s2;
para_PR PR_tq2;

currentloop_control c_loop_control;
void init_PI_para(double Ts, para_PI_discrete *PI_inv1, para_c_loop_machine_control_single *para_machine, double wd){
    PI_inv1->wd = wd;
    PI_inv1->Ts = Ts;
    PI_inv1->tau_p = para_machine->L/para_machine->R;
    PI_inv1->Ap = (double)exp(-PI_inv1->Ts/PI_inv1->tau_p);
    PI_inv1->Bp = 1 - PI_inv1->Ap;
    PI_inv1->tau_d = 1/PI_inv1->wd;
    PI_inv1->Ad = (double)exp(-PI_inv1->Ts/PI_inv1->tau_d);
    PI_inv1->Bd = 1 - PI_inv1->Ad;
    PI_inv1->state_1[0] = 0.0;
    PI_inv1->state_1[1] = 0.0;
    PI_inv1->state_1[2] = 0.0;
    PI_inv1->state_2[0] = 0.0;
    PI_inv1->state_2[1] = 0.0;
    PI_inv1->state_2[2] = 0.0;
    PI_inv1->state_3[0] = 0.0;
    PI_inv1->state_3[1] = 0.0;
    PI_inv1->state_3[2] = 0.0;

}

void init_PR_para(double Ts, para_PR *PR_inv1, para_c_loop_machine_control_single *para_machine, double wd){
    PR_inv1->wd = wd;
    PR_inv1->Ts = Ts;
    PR_inv1->Kr = PR_inv1->wd*para_machine->R*10.0;
    PR_inv1->Kp = PR_inv1->wd*para_machine->L;
    PR_inv1->state_1[0] = 0.0;
    PR_inv1->state_1[1] = 0.0;
    PR_inv1->state_1[2] = 0.0;
    PR_inv1->state_2[0] = 0.0;
    PR_inv1->state_2[1] = 0.0;
    PR_inv1->state_2[2] = 0.0;

}



currentloop_control *init_currentloop(void){

    c_loop_control.is_init = 1;
    c_loop_control.sel_config = InvFour;
    if (c_loop_control.sel_config == InvFour){
        c_loop_control.c_loop_inv1.Num_inv = INV1;
        c_loop_control.c_loop_inv2.Num_inv = INV2;
        c_loop_control.c_loop_inv3.Num_inv = INV3;
        c_loop_control.c_loop_inv4.Num_inv = INV4;

        int idx_inv;
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv1.Num_inv);
        c_loop_control.c_loop_inv1.inv = get_three_phase_inverter(idx_inv);
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv2.Num_inv);
        c_loop_control.c_loop_inv2.inv = get_three_phase_inverter(idx_inv);
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv3.Num_inv);
        c_loop_control.c_loop_inv3.inv = get_three_phase_inverter(idx_inv);
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv4.Num_inv);
        c_loop_control.c_loop_inv4.inv = get_three_phase_inverter(idx_inv);
        // init machine control parameters

        
        c_loop_control.para_machine = init_para_c_loop_machine_control();
        
        

        c_loop_control.tq.PI_regulator = &PI_tq;
        init_PI_para(TS, c_loop_control.tq.PI_regulator, &c_loop_control.para_machine->para_tq, WD_TQ);
        init_PR_para(TS, c_loop_control.tq.PR_regulator, &c_loop_control.para_machine->para_tq, WD_TQ);
        c_loop_control.s1.PI_regulator = &PI_s1;
        init_PI_para(TS, c_loop_control.s1.PI_regulator, &c_loop_control.para_machine->para_s1, WD_S1);
        init_PR_para(TS, c_loop_control.s1.PR_regulator, &c_loop_control.para_machine->para_s1, WD_S1);
        c_loop_control.s2.PI_regulator = &PI_s2;
        init_PI_para(TS, c_loop_control.s2.PI_regulator, &c_loop_control.para_machine->para_s2, WD_S2);
        init_PR_para(TS, c_loop_control.s2.PR_regulator, &c_loop_control.para_machine->para_s2, WD_S2);
        c_loop_control.tq2.PI_regulator = &PI_tq2;
        init_PI_para(TS, c_loop_control.tq2.PI_regulator, &c_loop_control.para_machine->para_tq2, WD_TQ);
        init_PR_para(TS, c_loop_control.tq2.PR_regulator, &c_loop_control.para_machine->para_tq2, WD_TQ);
    }
    else {
        c_loop_control.c_loop_inv1.Num_inv = INV1;
        c_loop_control.c_loop_inv2.Num_inv = INV2;
        c_loop_control.c_loop_inv3.Num_inv = INV3;
    

        int idx_inv;
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv1.Num_inv);
        c_loop_control.c_loop_inv1.inv = get_three_phase_inverter(idx_inv);
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv2.Num_inv);
        c_loop_control.c_loop_inv2.inv = get_three_phase_inverter(idx_inv);
        idx_inv = find_invIdex_invID (c_loop_control.c_loop_inv3.Num_inv);
        c_loop_control.c_loop_inv3.inv = get_three_phase_inverter(idx_inv);
        // init machine control parameters

        c_loop_control.para_machine = init_para_c_loop_machine_control();
        c_loop_control.tq.PR_regulator = &PR_tq;
        c_loop_control.s1.PR_regulator = &PR_s1;
        c_loop_control.s2.PR_regulator = &PR_s2;
        c_loop_control.tq.PI_regulator = &PI_tq;
        init_PI_para(TS, c_loop_control.tq.PI_regulator, &c_loop_control.para_machine->para_tq, WD_TQ);
        init_PR_para(TS, c_loop_control.tq.PR_regulator, &c_loop_control.para_machine->para_tq, WD_TQ);
        c_loop_control.s1.PI_regulator = &PI_s1;
        init_PI_para(TS, c_loop_control.s1.PI_regulator, &c_loop_control.para_machine->para_s1, WD_S1);
        init_PR_para(TS, c_loop_control.s1.PR_regulator, &c_loop_control.para_machine->para_s1, WD_S1);
        c_loop_control.s2.PI_regulator = &PI_s2;
        init_PI_para(TS, c_loop_control.s2.PI_regulator, &c_loop_control.para_machine->para_s2, WD_S2);
        init_PR_para(TS, c_loop_control.s2.PR_regulator, &c_loop_control.para_machine->para_s2, WD_S2);
    }
    return &c_loop_control;
}

void reset_regulator_single(c_loop_control_data *data){

    data->PI_regulator->state_1[0] = 0.0;
    data->PI_regulator->state_1[1] = 0.0;
    data->PI_regulator->state_1[2] = 0.0;
    data->PI_regulator->state_2[0] = 0.0;
    data->PI_regulator->state_2[1] = 0.0;
    data->PI_regulator->state_2[2] = 0.0;
    data->PI_regulator->state_3[0] = 0.0;
    data->PI_regulator->state_3[1] = 0.0;
    data->PI_regulator->state_3[2] = 0.0;

    data->PR_regulator->state_1[0] = 0.0;
    data->PR_regulator->state_1[1] = 0.0;
    data->PR_regulator->state_1[2] = 0.0;
    data->PR_regulator->state_2[0] = 0.0;
    data->PR_regulator->state_2[1] = 0.0;
    data->PR_regulator->state_2[2] = 0.0;
}

void reset_regulator(void){
	if (!c_loop_control.is_init)
	{
		init_currentloop();
	}
    if (c_loop_control.sel_config == InvFour){
        c_loop_control.tq.PI_regulator = &PI_tq;
        c_loop_control.s1.PI_regulator = &PI_s1;
        c_loop_control.s2.PI_regulator = &PI_s2;
        c_loop_control.tq2.PI_regulator = &PI_tq2;
        c_loop_control.tq.PR_regulator = &PR_tq;
        c_loop_control.s1.PR_regulator = &PR_s1;
        c_loop_control.s2.PR_regulator = &PR_s2;
        c_loop_control.tq2.PR_regulator = &PR_tq2;
        reset_regulator_single(&(c_loop_control.tq));
        reset_regulator_single(&(c_loop_control.s1));
        reset_regulator_single(&(c_loop_control.s2));
        reset_regulator_single(&(c_loop_control.tq2));
    }else{
        c_loop_control.tq.PI_regulator = &PI_tq;
        c_loop_control.s1.PI_regulator = &PI_s1;
        c_loop_control.s2.PI_regulator = &PI_s2;
        c_loop_control.tq.PR_regulator = &PR_tq;
        c_loop_control.s1.PR_regulator = &PR_s1;
        c_loop_control.s2.PR_regulator = &PR_s2;
        reset_regulator_single(&(c_loop_control.tq));
        reset_regulator_single(&(c_loop_control.s1));
        reset_regulator_single(&(c_loop_control.s2));
    }


}

currentloop_control *deinit_currentloop(void){

    c_loop_control.is_init = 0;
    reset_regulator();
    // init machine control parameters
    return &c_loop_control;
}


void get_inverter_current_abc(c_loop_threephase_data *c_loop){
    if (c_loop->inv->HW->sensor.enable){
        get_currents_three_phase_abc(c_loop->Iabc, c_loop->inv);
    }else{
        get_mb_currents_three_phase_abc(c_loop->Iabc, c_loop->inv);
    }
}


void get_all_inverter_current_abc(currentloop_control* data){
    if (c_loop_control.sel_config == InvFour){
        get_inverter_current_abc(&(data->c_loop_inv1));
        get_inverter_current_abc(&(data->c_loop_inv2));
        get_inverter_current_abc(&(data->c_loop_inv3));
        get_inverter_current_abc(&(data->c_loop_inv4));
    }else{
        get_inverter_current_abc(&(data->c_loop_inv1));
        get_inverter_current_abc(&(data->c_loop_inv2));
        get_inverter_current_abc(&(data->c_loop_inv3));
    }

}

void get_inverter_Vdc_single(c_loop_threephase_data *c_loop){
    if (c_loop->inv->HW->mb_Vsensor.enable){
        get_mb_voltage_abc(&(c_loop->Vdc_mes), c_loop->inv);
    }else{
        c_loop->Vdc_mes = 0.0;
    }
}

void get_all_inverter_Vdc(currentloop_control* data){
    if (c_loop_control.sel_config == InvFour){
        get_inverter_Vdc_single(&(data->c_loop_inv1));
        get_inverter_Vdc_single(&(data->c_loop_inv2));
        get_inverter_Vdc_single(&(data->c_loop_inv3));
        get_inverter_Vdc_single(&(data->c_loop_inv4));
    }else{
        get_inverter_Vdc_single(&(data->c_loop_inv1));
        get_inverter_Vdc_single(&(data->c_loop_inv2));
        get_inverter_Vdc_single(&(data->c_loop_inv3));
    }

}



void cal_invI_to_controlI_configseries(currentloop_control* data){
    
    //torque current calculation
    data->tq.Iabc[0] = 2.0*data->c_loop_inv1.Iabc[0] + data->c_loop_inv2.Iabc[0] + data->c_loop_inv3.Iabc[0]; 
    data->tq.Iabc[1] = 2.0*data->c_loop_inv1.Iabc[1] + data->c_loop_inv2.Iabc[1] + data->c_loop_inv3.Iabc[1];
    data->tq.Iabc[2] = 2.0*data->c_loop_inv1.Iabc[2] + data->c_loop_inv2.Iabc[2] + data->c_loop_inv3.Iabc[2];

    // suspension 1 
    data->s1.Iabc[0] = -0.5*data->c_loop_inv2.Iabc[0];
    data->s1.Iabc[1] = -0.5*data->c_loop_inv2.Iabc[1];
    data->s1.Iabc[2] = -0.5*data->c_loop_inv2.Iabc[2];

    // suspension 2 
    data->s2.Iabc[0] = -0.5*data->c_loop_inv3.Iabc[0];
    data->s2.Iabc[1] = -0.5*data->c_loop_inv3.Iabc[1];
    data->s2.Iabc[2] = -0.5*data->c_loop_inv3.Iabc[2];
}

void cal_invI_to_controlI_configInvFour(currentloop_control* data){
    
    //torque current calculation
    data->tq.Iabc[0] = data->c_loop_inv1.Iabc[0];
    data->tq.Iabc[1] = data->c_loop_inv1.Iabc[1];
    data->tq.Iabc[2] = data->c_loop_inv1.Iabc[2];

    //torque 2 current calculation
    data->tq2.Iabc[0] = data->c_loop_inv4.Iabc[0];
    data->tq2.Iabc[1] = data->c_loop_inv4.Iabc[1];
    data->tq2.Iabc[2] = data->c_loop_inv4.Iabc[2];

    // suspension 1 
    data->s1.Iabc[0] = 1.0*data->c_loop_inv2.Iabc[0] + 0.5*data->c_loop_inv1.Iabc[0];
    data->s1.Iabc[1] = 1.0*data->c_loop_inv2.Iabc[1] + 0.5*data->c_loop_inv1.Iabc[1];
    data->s1.Iabc[2] = 1.0*data->c_loop_inv2.Iabc[2] + 0.5*data->c_loop_inv1.Iabc[2];

    // suspension 2 
    data->s2.Iabc[0] = 1.0*data->c_loop_inv3.Iabc[0] + 0.5*data->c_loop_inv4.Iabc[0];
    data->s2.Iabc[1] = 1.0*data->c_loop_inv3.Iabc[1] + 0.5*data->c_loop_inv4.Iabc[1];
    data->s2.Iabc[2] = 1.0*data->c_loop_inv3.Iabc[2] + 0.5*data->c_loop_inv4.Iabc[2];
}



void cal_invI_to_controlI_config9(currentloop_control* data){
    
    //torque current calculation
    data->tq.Iabc[0] = 2.0*(data->c_loop_inv1.Iabc[0] + data->c_loop_inv2.Iabc[0]); 
    data->tq.Iabc[1] = 2.0*(data->c_loop_inv1.Iabc[1] + data->c_loop_inv2.Iabc[1]);
    data->tq.Iabc[2] = 2.0*(data->c_loop_inv1.Iabc[2] + data->c_loop_inv2.Iabc[2]);

    // suspension 1 
    data->s1.Iabc[0] = 0.5*(data->c_loop_inv1.Iabc[0] - data->c_loop_inv2.Iabc[0]);
    data->s1.Iabc[1] = 0.5*(data->c_loop_inv1.Iabc[1] - data->c_loop_inv2.Iabc[1]);
    data->s1.Iabc[2] = 0.5*(data->c_loop_inv1.Iabc[2] - data->c_loop_inv2.Iabc[2]);

    // suspension 2 
    data->s2.Iabc[0] = -0.5*data->c_loop_inv1.Iabc[0] - 0.5*data->c_loop_inv2.Iabc[0]-data->c_loop_inv3.Iabc[0];
    data->s2.Iabc[1] = -0.5*data->c_loop_inv1.Iabc[1] - 0.5*data->c_loop_inv2.Iabc[1]-data->c_loop_inv3.Iabc[1];
    data->s2.Iabc[2] = -0.5*data->c_loop_inv1.Iabc[2] - 0.5*data->c_loop_inv2.Iabc[2]-data->c_loop_inv3.Iabc[2];
}

void update_control_current(c_loop_control_data *data){
    data->Idq0_m1[0] = data->Idq0[0];
    data->Idq0_m1[1] = data->Idq0[1];
    data->Idq0_m1[2] = data->Idq0[2];

    abc_to_dq0(&(data->Iabc[0]), &(data->Idq0[0]), data->theta_rad);

}

void update_control_current_susp(c_loop_control_data *data){
    data->Idq0_m1[0] = data->Idq0[0];
    data->Idq0_m1[1] = data->Idq0[1];
    data->Idq0_m1[2] = data->Idq0[2];

    func_Clarke(&(data->Idq0[0]), &(data->Iabc[0]));

}


void update_states(double *states, double state_in[3]){
    states[0] = state_in[0];
    states[1] = state_in[1];
    states[2] = state_in[2];

}

void exp_jtheta(double theta, double *in_dq, double*out_dq){
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    out_dq[0] = cos_theta*in_dq[0] - sin_theta*in_dq[1];
    out_dq[1] = sin_theta*in_dq[0] + cos_theta*in_dq[1];
}

void regulator_PI_current_dq(c_loop_control_data *data_ctrl, para_c_loop_machine_control_single para_m, int tq){
	

	double K1 = para_m.R*data_ctrl->PI_regulator->Bd/data_ctrl->PI_regulator->Bp;
    if(tq == 1){
        K1 = K1*0.7;
    }
    
    double u1[2];
    double u2[2];
    data_ctrl->error[0] = data_ctrl->Idq0_ref[0] - data_ctrl->Idq0[0];
    data_ctrl->error[1] = data_ctrl->Idq0_ref[1] - data_ctrl->Idq0[1];
    u1[0] = data_ctrl->error[0] * K1;
    u1[1] = data_ctrl->error[1] * K1;
    u2[0] = u1[0] - data_ctrl->PI_regulator->state_1[0] + data_ctrl->PI_regulator->state_2[0]*data_ctrl->PI_regulator->Ad + data_ctrl->PI_regulator->state_3[0]*data_ctrl->PI_regulator->Bd;
    u2[1] = u1[1] - data_ctrl->PI_regulator->state_1[1] + data_ctrl->PI_regulator->state_2[1]*data_ctrl->PI_regulator->Ad + data_ctrl->PI_regulator->state_3[1]*data_ctrl->PI_regulator->Bd;

    double theta;
    theta = data_ctrl->we*2.0*data_ctrl->PI_regulator->Ts;
    exp_jtheta(theta, &u2, &(data_ctrl->vdq0_ref[0]));
    data_ctrl->vdq0_ref[2] = 0.0;

    // delay states
    // update state 1
    double state_in[3];
    theta = data_ctrl->we*(-1.0)*data_ctrl->PI_regulator->Ts;
    exp_jtheta(theta, &u1[0], &(state_in[0]));

    data_ctrl->PI_regulator->state_1[2] = 0.0;
    data_ctrl->PI_regulator->state_1[0]= state_in[0]*data_ctrl->PI_regulator->Ap;
    data_ctrl->PI_regulator->state_1[1] = state_in[1]*data_ctrl->PI_regulator->Ap;

    //update_states(&(data_ctrl->PI_regulator->state_1[0]), state_in);

    //update state 3
    data_ctrl->PI_regulator->state_3[0] = data_ctrl->PI_regulator->state_2[0];
    data_ctrl->PI_regulator->state_3[1] = data_ctrl->PI_regulator->state_2[1];

    //update state 2
    data_ctrl->PI_regulator->state_2[0] = u2[0];
    data_ctrl->PI_regulator->state_2[1] = u2[1];

    
}




void decouple(currentloop_control *data){
    if(data->sel_config == InvFour){
        data->s1.vdq0_decouple[0] = 0.0;
        data->s1.vdq0_decouple[1] = 0.0;
        data->s1.vdq0_decouple[2] = 0.0;

        data->s2.vdq0_decouple[0] = 0.0;
        data->s2.vdq0_decouple[1] = 0.0;
        data->s2.vdq0_decouple[2] = 0.0;

        double vabc[3];
        double vdecouple[3];
        dq0_to2_abc(&(vabc[0]), &(data->s1.vdq0_ref[0]), data->s1.theta_rad);
        abc_to_dq0(&(vabc[0]), &(vdecouple[0]), data->tq.theta_rad);

        data->tq.vdq0_decouple[0] = 0.5*vdecouple[0]; 
        data->tq.vdq0_decouple[1] = 0.5*vdecouple[1]; 
        data->tq.vdq0_decouple[2] = 0.5*vdecouple[2]; 

        dq0_to2_abc(&(vabc[0]), &(data->s2.vdq0_ref[0]), data->s2.theta_rad);
        abc_to_dq0(&(vabc[0]), &(vdecouple[0]), data->tq2.theta_rad);
     
        data->tq2.vdq0_decouple[0] = 0.5*vdecouple[0]; 
        data->tq2.vdq0_decouple[1] = 0.5*vdecouple[1]; 
        data->tq2.vdq0_decouple[2] = 0.5*vdecouple[2]; 
    }else if(data->sel_config == Inv_Series){
        data->tq.vdq0_decouple[0] = 0.0;
        data->tq.vdq0_decouple[1] = 0.0;
        data->tq.vdq0_decouple[2] = 0.0;

        double vabc[3];
        double vdecouple[3];
        dq0_to2_abc(&(vabc[0]), &(data->tq.vdq0_ref), data->tq.theta_rad);
        abc_to_dq0(&(vabc[0]), &(vdecouple[0]), data->s1.theta_rad);

        data->s1.vdq0_decouple[0] = 0.5*vdecouple[0]; 
        data->s1.vdq0_decouple[1] = 0.5*vdecouple[1]; 
        data->s1.vdq0_decouple[2] = 0.5*vdecouple[2]; 

        abc_to_dq0(&(vabc[0]), &(vdecouple[0]), data->s2.theta_rad);
        data->s2.vdq0_decouple[0] = 0.5*vdecouple[0]; 
        data->s2.vdq0_decouple[1] = 0.5*vdecouple[1]; 
        data->s2.vdq0_decouple[2] = 0.5*vdecouple[2]; 
    }
    else{
        
        data->s2.vdq0_decouple[0] = 0.0;
        data->s2.vdq0_decouple[1] = 0.0;
        data->s2.vdq0_decouple[2] = 0.0;

        double vs1_abc[3];
        double vs2_abc[3];
        double vtq_abc[3];
        dq0_to2_abc(&(vs1_abc[0]), &(data->s1.vdq0_ref), data->s1.theta_rad);
        dq0_to2_abc(&(vs2_abc[0]), &(data->s2.vdq0_ref), data->s2.theta_rad);
        dq0_to2_abc(&(vtq_abc[0]), &(data->tq.vdq0_ref), data->tq.theta_rad);

        // decoupling for vs1 -->v1
        double vdecouple_abc[3];

        vdecouple_abc[0] = 0.5*vtq_abc[0]-vs2_abc[0];
        vdecouple_abc[1] = 0.5*vtq_abc[1]-vs2_abc[1];
        vdecouple_abc[2] = 0.5*vtq_abc[2]-vs2_abc[2];

        abc_to_dq0(&(vdecouple_abc[0]), &(data->s1.vdq0_decouple[0]), data->s1.theta_rad);

        //decoupling for tq --> v2
        vdecouple_abc[0] = -vs1_abc[0]-vs2_abc[0];
        vdecouple_abc[1] = -vs1_abc[1]-vs2_abc[1];
        vdecouple_abc[2] = -vs1_abc[2]-vs2_abc[2];
        abc_to_dq0(&(vdecouple_abc[0]), &(data->tq.vdq0_decouple[0]), data->tq.theta_rad);
    }
}


void get_theta_we(currentloop_control *data){
    if(data->sel_config == InvFour){
        data->tq2.theta_rad += data->tq2.we*data->tq2.PI_regulator->Ts;
        data->tq2.theta_rad = fmod(data->tq2.theta_rad, 2.0 * PI); // Wrap to 2*pi
    }
    data->tq.theta_rad += data->tq.we*data->tq.PI_regulator->Ts;
    data->tq.theta_rad = fmod(data->tq.theta_rad, 2.0 * PI); // Wrap to 2*pi

    data->s1.theta_rad += data->s1.we*data->s1.PI_regulator->Ts;
    data->s1.theta_rad = fmod(data->s1.theta_rad, 2.0 * PI); // Wrap to 2*pi

    data->s2.theta_rad += data->s2.we*data->s2.PI_regulator->Ts;
    data->s2.theta_rad = fmod(data->s2.theta_rad, 2.0 * PI); // Wrap to 2*pi
}



void current_regulation (currentloop_control *data)
{
    if(!data->is_init || data == 0x00000000){
        data = init_currentloop();
    }

    if(!BM_ENABLE){
            //update sensed currents
        //update theta and we
        get_theta_we(data);
    }else{

    }
    

    if(data->sel_config == InvFour){
    	cal_invI_to_controlI_configInvFour(data);
    	update_control_current(&data->tq);
    	if(!SINGLE_INV){
    	update_control_current(&data->s1);
    	update_control_current(&data->s2);
    	update_control_current(&data->tq2);}

    	regulator_PI_current_dq(&data->tq, data->para_machine->para_tq, 1);
    	if(!SINGLE_INV){
    	regulator_PI_current_dq(&data->s1, data->para_machine->para_s1, 0);
    	regulator_PI_current_dq(&data->s2, data->para_machine->para_s2, 0);
    	regulator_PI_current_dq(&data->tq2, data->para_machine->para_tq2, 0);}

    	
        //if(!SINGLE_INV){
        decouple(data);
        //
        double vdq0[3];
        vdq0[0] = data->tq.vdq0_ref[0] + data->tq.vdq0_decouple[0]*1.0;
        vdq0[1] = data->tq.vdq0_ref[1] + data->tq.vdq0_decouple[1]*1.0;
        vdq0[2] = data->tq.vdq0_ref[2] + data->tq.vdq0_decouple[2]*1.0;
       
        dq0_to2_abc( &(data->c_loop_inv1.vabc_ref[0]), &(vdq0[0]), data->tq.theta_rad);

        vdq0[0] = data->s1.vdq0_ref[0] + data->s1.vdq0_decouple[0]*1.0;
        vdq0[1] = data->s1.vdq0_ref[1] + data->s1.vdq0_decouple[1]*1.0;
        vdq0[2] = data->s1.vdq0_ref[2] + data->s1.vdq0_decouple[2]*1.0;


        dq0_to2_abc(&(data->c_loop_inv2.vabc_ref[0]), &(vdq0[0]), data->s1.theta_rad);
        //dq0_to2_abc(data->c_loop_inv2.vabc_ref, vdq0, theta);

        vdq0[0] = data->s2.vdq0_ref[0] + data->s2.vdq0_decouple[0];
        vdq0[1] = data->s2.vdq0_ref[1] + data->s2.vdq0_decouple[1];
        vdq0[2] = data->s2.vdq0_ref[2] + data->s2.vdq0_decouple[2];

        dq0_to2_abc(&(data->c_loop_inv3.vabc_ref[0]), &vdq0[0], data->s2.theta_rad);

        vdq0[0] = data->tq2.vdq0_ref[0] + data->tq2.vdq0_decouple[0];
        vdq0[1] = data->tq2.vdq0_ref[1] + data->tq2.vdq0_decouple[1];
        vdq0[2] = data->tq2.vdq0_ref[2] + data->tq2.vdq0_decouple[2];

        dq0_to2_abc(&(data->c_loop_inv4.vabc_ref[0]), &vdq0[0], data->tq2.theta_rad);//}

    }else if(data->sel_config == Inv_Series){
    	//calculate torque and suspension currents
    	cal_invI_to_controlI_configseries(data);
    	update_control_current(&data->tq);
    	update_control_current(&data->s1);
    	update_control_current(&data->s2);

    	regulator_PI_current_dq(&data->tq, data->para_machine->para_tq, 0);
    	regulator_PI_current_dq(&data->s1, data->para_machine->para_s1, 0);
    	regulator_PI_current_dq(&data->s2, data->para_machine->para_s2, 0);


        data->tq.vdq0_ref[0] = data->tq.vdq0_ref[0] ;
        data->tq.vdq0_ref[1] = data->tq.vdq0_ref[1] ;
        data->tq.vdq0_ref[2] = data->tq.vdq0_ref[2] ;


        decouple(data);
        //
        double vdq0[3];
        vdq0[0] = data->tq.vdq0_ref[0] + data->tq.vdq0_decouple[0];
        vdq0[1] = data->tq.vdq0_ref[1] + data->tq.vdq0_decouple[1];
        vdq0[2] = data->tq.vdq0_ref[2] + data->tq.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv1.vabc_ref, vdq0, data->tq.theta_rad);

        vdq0[0] = -data->s1.vdq0_ref[0] + data->s1.vdq0_decouple[0];
        vdq0[1] = -data->s1.vdq0_ref[1] + data->s1.vdq0_decouple[1];
        vdq0[2] = -data->s1.vdq0_ref[2] + data->s1.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv2.vabc_ref, vdq0, data->s1.theta_rad);

        vdq0[0] = -data->s2.vdq0_ref[0] + data->s2.vdq0_decouple[0];
        vdq0[1] = -data->s2.vdq0_ref[1] + data->s2.vdq0_decouple[1];
        vdq0[2] = -data->s2.vdq0_ref[2] + data->s2.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv3.vabc_ref, vdq0, data->s2.theta_rad);
    } else{
    	//calculate torque and suspension currents
    	cal_invI_to_controlI_config9(data);
    	update_control_current(&data->tq);
    	update_control_current(&data->s1);
    	update_control_current(&data->s2);

    	regulator_PI_current_dq(&data->tq, data->para_machine->para_tq, 0);
    	regulator_PI_current_dq(&data->s1, data->para_machine->para_s1, 0);
    	regulator_PI_current_dq(&data->s2, data->para_machine->para_s2, 0);


        data->tq.vdq0_ref[0] = data->tq.vdq0_ref[0] ;
        data->tq.vdq0_ref[1] = data->tq.vdq0_ref[1] ;
        data->tq.vdq0_ref[2] = data->tq.vdq0_ref[2] ;


        decouple(data);
        //
        double vdq0[3];
        vdq0[0] = data->tq.vdq0_ref[0] + data->tq.vdq0_decouple[0];
        vdq0[1] = data->tq.vdq0_ref[1] + data->tq.vdq0_decouple[1];
        vdq0[2] = data->tq.vdq0_ref[2] + data->tq.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv1.vabc_ref, vdq0, data->tq.theta_rad);

        vdq0[0] = -data->s1.vdq0_ref[0] + data->s1.vdq0_decouple[0];
        vdq0[1] = -data->s1.vdq0_ref[1] + data->s1.vdq0_decouple[1];
        vdq0[2] = -data->s1.vdq0_ref[2] + data->s1.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv2.vabc_ref, vdq0, data->s1.theta_rad);

        vdq0[0] = -data->s2.vdq0_ref[0] + data->s2.vdq0_decouple[0];
        vdq0[1] = -data->s2.vdq0_ref[1] + data->s2.vdq0_decouple[1];
        vdq0[2] = -data->s2.vdq0_ref[2] + data->s2.vdq0_decouple[2];

        dq0_to2_abc(data->c_loop_inv3.vabc_ref, vdq0, data->s2.theta_rad);
    }
    
}
