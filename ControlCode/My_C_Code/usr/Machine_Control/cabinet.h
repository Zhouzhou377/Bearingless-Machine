#ifndef CABINET_H
#define CABINET_H

#include "usr/Machine_Control/inverter.h"

#define CABINET_NUM_1PHASE (2)
#define CABINET_NUM_3PHASE (4)
#define CABINET_NUM_INVERTERS (CABINET_NUM_1PHASE + CABINET_NUM_3PHASE)
#define NUM_POS_SENSORS (3)

InverterSinglePhase_t *get_single_phase_inverter(int inverter);
InverterThreePhase_t *get_three_phase_inverter(int inverter);
void default_inverter_setup(double Vdc);


//extern command isn't actually allocating memory, it
//just tells compiler variable exists in global scope so that
//we can use it in other files
//POSITION SENSORS
extern const analog_sensor_t pos_sensors[NUM_POS_SENSORS];
int find_invIdex_invID (int inverter);
#endif // CABINET_H
