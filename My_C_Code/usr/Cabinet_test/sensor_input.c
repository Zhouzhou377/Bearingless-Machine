#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/hardware.h"
#include "usr/Cabinet_test/inverter.h"
#include "drv/analog.h"
#include "usr/Cabinet_test/transforms.h"
//#include "SIL_specific.h"

double LOG_Ia = 0.0;
double LOG_Ib = 0.0;
double LOG_Ic = 0.0;

double read_adc(analog_channel_e adcCh){
    float adc;
    analog_getf(adcCh, &adc);
    return (double) adc;
}

double read_analog_sensor(analog_sensor_t sense){

    double adc = read_adc(sense.adcCh);
    double out = sense.adcGain*adc + sense.adcOffset;
    return out;
}

void input_read_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv){

	Iabc[0] = read_analog_sensor(inv->HW->sensor.Ia);
	Iabc[1] = read_analog_sensor(inv->HW->sensor.Ib);
	Iabc[2] = read_analog_sensor(inv->HW->sensor.Ic);
}

void input_read_currents_three_phase_xy(double *Ixy, InverterThreePhase_t *inv){

    double Iabc[3];
    input_read_currents_three_phase_abc(Iabc, inv);
    LOG_Ia = Iabc[0];
    LOG_Ib = Iabc[1];
    LOG_Ic = Iabc[2];
    func_Clarke(Ixy, Iabc); //convert to alpha-beta frame
}

void input_read_current_single_phase(double *Iz, InverterSinglePhase_t *inv){

	*Iz = read_analog_sensor(inv->HW->sensor.Iz);
	//LOG_I = *Iz;
}
