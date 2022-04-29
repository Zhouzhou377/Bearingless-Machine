#ifndef HARDWARE_H
#define HARDWARE_H
#include "drv/motherboard.h"
#include "drv/analog.h"
//#include "usr/Cabinet_test/mb_sensor.h"
//#include "SIL_specific.h"

typedef struct analog_sensor_t {

    analog_channel_e adcCh;
    double adcGain;
    double adcOffset;

} analog_sensor_t;

typedef struct mb_sensor {

    mb_channel_e mbCh;
    double adcGain;
    double adcOffset;

} mb_sensor;

//Inverter channel info for current control
typedef struct hw_three_phase_pwm_t {

	int pwmIdxA;
	int pwmIdxB;
	int pwmIdxC;

} hw_three_phase_pwm_t;

typedef struct hw_three_phase_sensor_t {

    bool enable;
    analog_sensor_t Ia;
    analog_sensor_t Ib;
    analog_sensor_t Ic;

} hw_three_phase_sensor_t;

//zzw
typedef struct mb_three_phase_sensor {
    bool enable;
    mb_sensor mb_Ia;
    mb_sensor mb_Ib;
    mb_sensor mb_Ic;

} mb_three_phase_sensor;
//

typedef struct HW_three_phase_t {

    hw_three_phase_pwm_t pwm;
    hw_three_phase_sensor_t sensor;
    mb_three_phase_sensor mb_csensor;
} HW_three_phase_t;

typedef struct hw_single_phase_pwm_t {

	int pwmIdxA;
	int pwmIdxB;

} hw_single_phase_pwm_t;

typedef struct hw_single_phase_sensor_t {

    analog_sensor_t Iz;

} hw_single_phase_sensor_t;

typedef struct HW_single_phase_t {

    hw_single_phase_pwm_t pwm;
    hw_single_phase_sensor_t sensor;

} HW_single_phase_t;


#endif // HARDWARE_H
