#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H

#include "usr/Cabinet_test/inverter.h"
#include "usr/Cabinet_test/controllers.h"

typedef struct Current_Controller_SinglePhase_t {

    int enable;
    double I;
    double I_filtered;
    double V_star;
    InverterSinglePhase_t *inverter;
    PI_Container cc_z;
    LowPass_Container filter_I;
    int filter_currents;

} Current_Controller_SinglePhase_t;

typedef struct Current_Controller_ThreePhase_t {

    int enable;
    double Ixy[2];
    double Ixy_filtered[2];
    double Vxy_star[2];
    double Vabc_star[3];
    InverterThreePhase_t *inverter;
    PI_Container cc_x;
    PI_Container cc_y;
    LowPass_Container filter_Ix;
    LowPass_Container filter_Iy;
    int filter_currents;

} Current_Controller_ThreePhase_t;

Current_Controller_SinglePhase_t *get_single_phase_cc(int inverter);
Current_Controller_ThreePhase_t *get_three_phase_cc(int inverter);

void init_single_phase_cc(Current_Controller_SinglePhase_t *cc);
void init_three_phase_cc(Current_Controller_ThreePhase_t *cc);
void set_single_phase_cc_gains(double kp, double ki, double Ts, Current_Controller_SinglePhase_t *cc);
void set_three_phase_cc_gains(double kp, double ki, double Ts, Current_Controller_ThreePhase_t *cc);
void default_three_phase_gains(double Ts, Current_Controller_ThreePhase_t *cc);
void default_single_phase_gains(double Ts, Current_Controller_SinglePhase_t *cc);

//COMMANDS TO CALL TO CONTROL
void cc_setup(double Vdc, double Ts);

void cc_enable_single_phase(Current_Controller_SinglePhase_t *cc);
void cc_enable_three_phase(Current_Controller_ThreePhase_t *cc);
void cc_disable_single_phase(Current_Controller_SinglePhase_t *cc);
void cc_disable_three_phase(Current_Controller_ThreePhase_t *cc);
//


void disable_all_single_phase_cc(void);
void disable_all_three_phase_cc(void);
void disable_all_cc(void);

void set_Vdc_all_inverters(double Vdc);

/*void set_current_commands_three_phase(double Ix_star, double Iy_star, Current_Controller_ThreePhase_t *cc);
void set_current_commands_three_phase_abc(double Ia_star, double Ib_star, Current_Controller_ThreePhase_t *cc);
void set_current_commands_single_phase(double I_star, Current_Controller_SinglePhase_t *cc);
void update_voltage_commmands_three_phase(Current_Controller_ThreePhase_t *cc);
void update_voltage_commands_single_phase(Current_Controller_SinglePhase_t *cc);
void output_voltage_three_phase(Current_Controller_ThreePhase_t *cc);
void output_voltage_single_phase(Current_Controller_SinglePhase_t *cc);
void anti_windup_three_phase(Current_Controller_ThreePhase_t *cc);
void anti_windup_single_phase(Current_Controller_SinglePhase_t *cc);*/
void read_currents_three_phase(Current_Controller_ThreePhase_t *cc);
void read_currents_single_phase(Current_Controller_SinglePhase_t *cc);
void set_three_phase_current_filter_gains(double tau, double Ts, Current_Controller_ThreePhase_t *cc);
void set_single_phase_current_filter_gains(double tau, double Ts, Current_Controller_SinglePhase_t *cc);
void default_three_phase_filter_gains(double Ts, Current_Controller_ThreePhase_t *cc);
void default_single_phase_filter_gains(double Ts, Current_Controller_SinglePhase_t *cc);

/*void current_control_callback_single_phase(Current_Controller_SinglePhase_t *cc);
void current_control_callback_three_phase(Current_Controller_ThreePhase_t *cc);
void current_control_callback(void);*/

#endif // CURRENT_CONTROL_H

