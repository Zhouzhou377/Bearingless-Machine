#include "usr/Cabinet_test/current_control.h"
#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/transforms.h"
#include "usr/Cabinet_test/sensor_input.h"
#include "usr/Cabinet_test/mb_sensor.h"
//#include "usr/Cabinet_test/machine.h"
#include "usr/Cabinet_test/inverter.h"
//#include "usr/Cabinet_test/anti_windup.h"

#define SIL (0) //1 if running SIL, 0 if actual hardware
#define TAU_CURRENT_SENSOR (1.0/(2.0*3.1415*4000.0)) //4kHz default value
#define FILTER_CURRENTS (1)

Current_Controller_SinglePhase_t cc_single_phase[CABINET_NUM_1PHASE];
Current_Controller_ThreePhase_t cc_three_phase[CABINET_NUM_3PHASE];

Current_Controller_SinglePhase_t *get_single_phase_cc(int inverter){
    
    return &(cc_single_phase[inverter]);
}

Current_Controller_ThreePhase_t *get_three_phase_cc(int inverter){
   
    return &(cc_three_phase[inverter]);
}

void init_single_phase_cc(Current_Controller_SinglePhase_t *cc){

    //sets initial values for all primitive variables
    cc->enable = 0;
    cc->I = 0;
    cc->V_star = 0;
    cc->filter_currents = 0;

}

void init_three_phase_cc(Current_Controller_ThreePhase_t *cc){

    //sets initial values for all primitive variables
    cc->enable = 0;
    cc->Ixy[0] = 0;
    cc->Ixy[1] = 0;
    cc->Vxy_star[0] = 0;
    cc->Vxy_star[1] = 0;
    cc->Vabc_star[0] = 0;
    cc->Vabc_star[1] = 0;
    cc->Vabc_star[2] = 0;
    cc->filter_currents = 0;

}

/*void set_single_phase_cc_gains(double kp, double ki, double Ts, Current_Controller_SinglePhase_t *cc){

    populate_PI_Container(kp, ki, Ts, &(cc->cc_z));

}

void set_three_phase_cc_gains(double kp, double ki, double Ts, Current_Controller_ThreePhase_t *cc){

    populate_PI_Container(kp, ki, Ts, &(cc->cc_x));
    populate_PI_Container(kp, ki, Ts, &(cc->cc_y));

}

void default_three_phase_gains(double Ts, Current_Controller_ThreePhase_t *cc){
    set_three_phase_cc_gains(KP_I_RAD, KI_I_RAD, Ts, cc);
}

void default_single_phase_gains(double Ts, Current_Controller_SinglePhase_t *cc){
    set_single_phase_cc_gains(KP_I_AX, KI_I_AX, Ts, cc);
}
*/
void set_three_phase_current_filter_gains(double tau, double Ts, Current_Controller_ThreePhase_t *cc){
    populate_LowPass_Container(Ts, tau, &(cc->filter_Ix));
    populate_LowPass_Container(Ts, tau, &(cc->filter_Iy));
}

void set_single_phase_current_filter_gains(double tau, double Ts, Current_Controller_SinglePhase_t *cc){
    populate_LowPass_Container(Ts, tau, &(cc->filter_I));
}

void default_three_phase_filter_gains(double Ts, Current_Controller_ThreePhase_t *cc){
	set_three_phase_current_filter_gains(TAU_CURRENT_SENSOR, Ts, cc);
}
void default_single_phase_filter_gains(double Ts, Current_Controller_SinglePhase_t *cc){
	set_single_phase_current_filter_gains(TAU_CURRENT_SENSOR, Ts, cc);
}

void cc_setup(double Vdc, double Ts){

    //first set up inverters
    default_inverter_setup(Vdc);

    //next set controller initial values, assign inverters and set gains to default
    for (int i = 0; i < CABINET_NUM_1PHASE; i++) {
        Current_Controller_SinglePhase_t *cc = get_single_phase_cc(i);
        init_single_phase_cc(cc);
        cc->inverter = get_single_phase_inverter(i);
        //default_single_phase_gains(Ts, cc);
        default_single_phase_filter_gains(Ts, cc);
    }
    for (int i = 0; i < CABINET_NUM_3PHASE; i++) {
        Current_Controller_ThreePhase_t *cc = get_three_phase_cc(i);
        init_three_phase_cc(cc);
        cc->inverter = get_three_phase_inverter(i);
        //default_three_phase_gains(Ts, cc);
        default_three_phase_filter_gains(Ts, cc);
    }
}

void set_Vdc_all_inverters(double Vdc){

    for (int i = 0; i < CABINET_NUM_1PHASE; i++) {
        Current_Controller_SinglePhase_t *cc = get_single_phase_cc(i);
        cc->inverter->Vdc = Vdc;
    }
    for (int i = 0; i < CABINET_NUM_3PHASE; i++) {
        Current_Controller_ThreePhase_t *cc = get_three_phase_cc(i);
        cc->inverter->Vdc = Vdc;
    }
}


void cc_enable_single_phase(Current_Controller_SinglePhase_t *cc){
	init_single_phase_cc(cc);
    cc->enable = 1;
}

void cc_enable_three_phase(Current_Controller_ThreePhase_t *cc){
	init_three_phase_cc(cc);
    cc->enable = 1;
}

void cc_disable_single_phase(Current_Controller_SinglePhase_t *cc){

    //reset_PI_controller(&(cc->cc_z));
    set_pole_volts_single_phase(0,0, cc->inverter);
    cc->enable = 0;
    cc->I = 0.0;
}

void cc_disable_three_phase(Current_Controller_ThreePhase_t *cc){

	 //reset_PI_controller(&(cc->cc_x));
	 //reset_PI_controller(&(cc->cc_y));
    set_pole_volts_three_phase(0,0,0, cc->inverter);
    cc->enable = 0;
    cc->Ixy[0] = 0.0;
    cc->Ixy[1] = 0.0;

}

void disable_all_single_phase_cc(void){

    for (int i = 0; i < CABINET_NUM_1PHASE; i++) {
        cc_disable_single_phase( &(cc_single_phase[i]) );
    }
}

void disable_all_three_phase_cc(void){

    for (int i = 0; i < CABINET_NUM_3PHASE; i++) {
        cc_disable_three_phase( &(cc_three_phase[i]) );
    }
}

void disable_all_cc(void){

    //disable all single phase current controllers
    disable_all_single_phase_cc();

    //disable all three phase current controllers
    disable_all_three_phase_cc();
}

/*void set_current_commands_three_phase(double Ix_star, double Iy_star, Current_Controller_ThreePhase_t *cc){

    cc->cc_x.r_star = Ix_star;
    cc->cc_y.r_star = Iy_star;
    //LOG_I = Ix_star;

}

void set_current_commands_three_phase_abc(double Ia_star, double Ib_star, Current_Controller_ThreePhase_t *cc){


	double Ixy_star[2];
    double Iabc_star[3] = {Ia_star, Ib_star, -Ia_star-Ib_star};

    abc_to_xy(Ixy_star, Iabc_star);

    set_current_commands_three_phase(Ixy_star[0], Ixy_star[1], cc);
}

void set_current_commands_single_phase(double I_star, Current_Controller_SinglePhase_t *cc){
    cc->cc_z.r_star = I_star;
}

void update_voltage_commands_three_phase(Current_Controller_ThreePhase_t *cc){

    //compute control voltage commands in alpha beta frame and update context
	if (cc->filter_currents)
	{
		cc->Vxy_star[0] = PI_update(&(cc->cc_x), cc->Ixy_filtered[0]);
		cc->Vxy_star[1] = PI_update(&(cc->cc_y), cc->Ixy_filtered[1]);
	}
	else {
		cc->Vxy_star[0] = PI_update(&(cc->cc_x), cc->Ixy[0]);
		cc->Vxy_star[1] = PI_update(&(cc->cc_y), cc->Ixy[1]);
	}

    //update abc current commands in context
    xy_to_abc(cc->Vxy_star, cc->Vabc_star);
}

void update_voltage_commands_single_phase(Current_Controller_SinglePhase_t *cc){

	if (cc->filter_currents){
		cc->V_star = PI_update(&(cc->cc_z), cc->I_filtered);
	}
	else {
		cc->V_star = PI_update(&(cc->cc_z), cc->I);
	}
}

void output_voltage_three_phase(Current_Controller_ThreePhase_t *cc){

    set_line_volts_three_phase(cc->Vabc_star[0], cc->Vabc_star[1], cc->Vabc_star[2], cc->inverter);

}

void output_voltage_single_phase(Current_Controller_SinglePhase_t *cc){

    set_line_volts_single_phase(cc->V_star, cc->inverter);

}*/

/*void anti_windup_three_phase(Current_Controller_ThreePhase_t *cc){

    set_three_phase_current_clamp(&(cc->cc_x), &(cc->cc_y), cc->inverter->Vdc);

}

void anti_windup_single_phase(Current_Controller_SinglePhase_t *cc){

    set_single_phase_current_clamp(&(cc->cc_z), cc->inverter->Vdc);

}*/

void read_currents_three_phase(Current_Controller_ThreePhase_t *cc){

    input_read_currents_three_phase_xy(cc->Ixy, cc->inverter);
    cc->Ixy_filtered[0] = LowPass_update(&(cc->filter_Ix), cc->Ixy[0]);
    cc->Ixy_filtered[1] = LowPass_update(&(cc->filter_Iy), cc->Ixy[1]);

}

void read_currents_single_phase(Current_Controller_SinglePhase_t *cc){

    input_read_current_single_phase(&(cc->I), cc->inverter);
    cc->I_filtered = LowPass_update(&(cc->filter_I), cc->I);

}
/*
void current_control_callback_single_phase(Current_Controller_SinglePhase_t *cc){

    if (cc->enable == 1){
        if (SIL) {
            update_voltage_commands_single_phase(cc);
            anti_windup_single_phase(cc);
        } else {
            read_currents_single_phase(cc);
            update_voltage_commands_single_phase(cc);
            output_voltage_single_phase(cc);
            anti_windup_single_phase(cc);
        }
    }
}

void current_control_callback_three_phase(Current_Controller_ThreePhase_t *cc){

    if (cc->enable == 1){
        if (SIL){
            update_voltage_commands_three_phase(cc);
            anti_windup_three_phase(cc);
        } else {
            read_currents_three_phase(cc);
            update_voltage_commands_three_phase(cc);
            output_voltage_three_phase(cc);
            anti_windup_three_phase(cc);
        }
    }
}

void current_control_callback(void){

    //loop over each single phase inverter
    for (int i = 0; i < CABINET_NUM_1PHASE; i++) {
        Current_Controller_SinglePhase_t *cc = get_single_phase_cc(i); //grab current controller
        current_control_callback_single_phase(cc);
    }

    //loop over each single phase inverter
    for (int i = 0; i < CABINET_NUM_3PHASE; i++) {
        Current_Controller_ThreePhase_t *cc = get_three_phase_cc(i); //grab current controller
        current_control_callback_three_phase(cc);
    }
}
*/
