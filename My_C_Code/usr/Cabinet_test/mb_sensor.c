#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/hardware.h"
#include "usr/Cabinet_test/inverter.h"
#include "drv/motherboard.h"
#include "usr/Cabinet_test/mb_sensor.h"

#include <stdint.h>
#include <stdio.h>
//#include "SIL_specific.h"

#define MB_CURRENtSENSOR_ZERO_HEX 0x8E38   // 2.5V vias
#define MB_CURRENtSENSOR_FULLADC 0xFFFF



double mb_int32todouble(int32_t data_raw){
    double data;
    int32_t data_raw_16;
    data_raw_16 = data_raw & 0x0000FFFF;
    data = (double)(data_raw_16 - MB_CURRENtSENSOR_ZERO_HEX)/(double)MB_CURRENtSENSOR_FULLADC;
    data = data*4.5;
    return data;
}

double read_mb_current_adc(mb_channel_e mbCh){
    float mb_adc;
    int32_t *data_raw;
    if(motherboard_get_data(mbCh, data_raw)){
        mb_adc = mb_int32todouble(data_raw[0]);
    	//mb_adc = data_raw;
        return (double) mb_adc;
    }
    return (double) 0;
}

double read_mb_current_sensor(mb_sensor sensor){

    double adc = read_mb_current_adc(sensor.mbCh);
    double out = sensor.adcGain*adc + sensor.adcOffset;
    return out;
}

void input_read_mb_currents_three_phase_abc(double *Iabc, InverterThreePhase_t *inv){
    mb_sensor sensor;
    sensor = inv->HW->mb_sensor.Ia;
	//Iabc[0] = read_mb_current_sensor(sensor);
    Iabc[0] = read_mb_current_adc(sensor.mbCh);
    sensor = inv->HW->mb_sensor.Ib;
    Iabc[1] = read_mb_current_adc(sensor.mbCh);
	//Iabc[1] = read_mb_current_sensor(sensor);
    sensor = inv->HW->mb_sensor.Ic;
    Iabc[2] = read_mb_current_adc(sensor.mbCh);
	//Iabc[2] = read_mb_current_sensor(sensor);
}


