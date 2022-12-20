#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/analog_sensor.h"
#include "usr/Machine_Control/hardware.h"
#include "usr/Machine_Control/inverter.h"
#include "drv/analog.h"
#include "usr/Machine_Control/transforms.h"
//#include "SIL_specific.h"



double get_adc(analog_channel_e adcCh){
    float adc;
    analog_getf(adcCh, &adc);
    return (double) adc;
}

double get_analog_sensor(analog_sensor_t sense){

    double adc = get_adc(sense.adcCh);
    double out = sense.adcGain*adc + sense.adcOffset;
    return out;
}

void get_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv){

	Iabc[0] = get_analog_sensor(inv->HW->sensor.Ia);
	Iabc[1] = get_analog_sensor(inv->HW->sensor.Ib);
	Iabc[2] = get_analog_sensor(inv->HW->sensor.Ic);
}

void get_currents_three_phase_dq0(double *Idq0, double theta_rad, InverterThreePhase_t *inv){

    double Iabc[3];
    get_currents_three_phase_abc(Iabc, inv);
    abc_to_dq0(Iabc, Idq0, theta_rad); 
}

