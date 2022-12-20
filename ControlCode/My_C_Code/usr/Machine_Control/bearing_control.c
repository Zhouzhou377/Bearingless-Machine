//#include "usr/cramb/force_to_currents.h"
#include "usr/Machine_Control/bearing_control.h"
#include "usr/Machine_Control/current_control.h"



#define DEFAULT_RAD_INVERTER (5)
#define DEFAULT_AX_INVERTER (4)

//static command makes variables only available to this file
//then we could use the same variable names in other files without
//name clashes
/*static CRAMB_ctxt bearing = {.linear = 0, .Ixyz_star = {0,0,0}};
CRAMB_ctxt *get_CRAMB_ctxt(void){
    return &bearing;
}
void set_axial_inverter(int inverter, CRAMB_ctxt *CRAMB){
    CRAMB->cc_ax = get_single_phase_cc(inverter - 1);
}
void set_radial_inverter(int inverter, CRAMB_ctxt *CRAMB){
    CRAMB->cc_rad = get_three_phase_cc(inverter - 5);
}*/

void cabinet_setup(double Vdc, double Ts){

    //mc_setup(Ts, get_mc());
    cc_setup(Vdc, Ts);
}

/*
void CRAMB_setup(CRAMB_ctxt *CRAMB){
    //SET MOTION CONTROLLER AND CURRENT CONTROLLER POINTERS
    CRAMB->mc = get_mc();
    set_axial_inverter(DEFAULT_AX_INVERTER, CRAMB);
    set_radial_inverter(DEFAULT_RAD_INVERTER, CRAMB);
}
void CRAMB_enable(CRAMB_ctxt *CRAMB){
    cc_enable_single_phase(CRAMB->cc_ax);
    cc_enable_three_phase(CRAMB->cc_rad);
    mc_enable(CRAMB->mc);
}
void CRAMB_disable(CRAMB_ctxt *CRAMB){
    mc_disable(CRAMB->mc);
    cc_disable_single_phase(CRAMB->cc_ax);
    cc_disable_three_phase(CRAMB->cc_rad);
}
void CRAMB_Callback(CRAMB_ctxt *CRAMB){
	motion_control_callback(CRAMB->mc);
    //only update current commands if motion controller is enabled
    if (CRAMB->mc->enable == 1) {
        force_to_currents(CRAMB->mc->Fxyz_star, CRAMB->Ixyz_star, CRAMB->linear);
        set_current_commands_three_phase(CRAMB->Ixyz_star[0], CRAMB->Ixyz_star[1], CRAMB->cc_rad);
        set_current_commands_single_phase(CRAMB->Ixyz_star[2], CRAMB->cc_ax);
    }
    current_control_callback();
}
*/
