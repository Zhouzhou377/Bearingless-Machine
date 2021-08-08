//#ifndef MB_SENSOR_H
//#define MB_SENSOR_H


#include "drv/motherboard.h"

#include "usr/Cabinet_test/hardware.h"
#include "usr/Cabinet_test/inverter.h"


double read_mb_current_adc(mb_channel_e mbCh);
double read_mb_current_sensor(mb_sensor sensor);
void input_read_mb_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv);




//#endif // MB_CURRENT_SENSOR_H
