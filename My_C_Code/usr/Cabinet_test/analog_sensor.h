#ifndef SENSOR_INPUT_H
#define SENSOR_INPUT_H

#include "drv/analog.h"
#include "usr/Cabinet_test/hardware.h"
#include "usr/Cabinet_test/inverter.h"

double get_adc(analog_channel_e adcCh);
double get_analog_sensor(analog_sensor_t sense);
void get_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv);
void get_currents_three_phase_dq0(double *Idq0, InverterThreePhase_t *inv);
void input_read_current_single_phase(double *Iz, InverterSinglePhase_t *inv);


#endif // SENSOR_INPUT_H
