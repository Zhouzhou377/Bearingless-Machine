//#ifndef MB_SENSOR_H
//#define MB_SENSOR_H


#include "drv/motherboard.h"

#include "usr/Cabinet_test/hardware.h"
#include "usr/Cabinet_test/inverter.h"
#include <stdint.h>

double get_mb_current_adc(mb_channel_e mbCh);
double get_mb_current(mb_sensor sensor);
void get_mb_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv);




//#endif // MB_CURRENT_SENSOR_H
