#ifndef BEARING_CONTROL_H
#define BEARING_CONTROL_H

//#include "usr/cramb/current_control.h"
//#include "usr/cramb/motion_control.h"

/*typedef struct CRAMB_ctxt{

    //store key variables and state information
    int linear;
    double Ixyz_star[3];

    Motion_Controller_t *mc;
    Current_Controller_SinglePhase_t *cc_ax;
    Current_Controller_ThreePhase_t *cc_rad;

} CRAMB_ctxt;*/

//CRAMB_ctxt *get_CRAMB_ctxt(void);

//void set_axial_inverter(int inverter, CRAMB_ctxt *CRAMB);
//void set_radial_inverter(int inverter, CRAMB_ctxt *CRAMB);

void cabinet_setup(double Vdc, double Ts);
//void CRAMB_setup(CRAMB_ctxt *CRAMB);

//void CRAMB_enable(CRAMB_ctxt *CRAMB);
//void CRAMB_disable(CRAMB_ctxt *CRAMB);

//void CRAMB_Callback(CRAMB_ctxt *CRAMB);

#endif // BEARING_CONTROL_H
