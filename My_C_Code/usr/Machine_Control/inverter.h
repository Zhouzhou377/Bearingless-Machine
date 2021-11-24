#ifndef INVERTER_H
#define INVERTER_H

#include "usr/Machine_Control/hardware.h"

typedef struct InverterSinglePhase_t {

    double Vdc;
    HW_single_phase_t *HW;

} InverterSinglePhase_t;

typedef struct InverterThreePhase_t {

    double Vdc;
    HW_three_phase_t *HW;

} InverterThreePhase_t;

void set_line_volts_single_phase(double V, InverterSinglePhase_t *inv);
void set_pole_volts_single_phase(double Va, double Vb, InverterSinglePhase_t *inv);
void set_line_volts_three_phase(double Va, double Vb, double Vc, InverterThreePhase_t *inv);
void set_pole_volts_three_phase(double Va, double Vb, double Vc, InverterThreePhase_t *inv);

#endif // INVERTER_H
