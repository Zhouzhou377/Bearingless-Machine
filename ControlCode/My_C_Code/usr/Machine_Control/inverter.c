#include "usr/Machine_Control/inverter.h"
#include "drv/pwm.h"
//#include "SIL_specific.h"


// the following functions are for outputting voltages manually.
void set_line_volts_single_phase(double V, InverterSinglePhase_t *inv){

	//sets line to neutral voltages assuming neutral point is at Vdc/2

	double da, db;

	da = 0.5 + (V/2.0)/(inv->Vdc);
	db = 0.5 - (V/2.0)/(inv->Vdc);

	pwm_set_duty(inv->HW->pwm.pwmIdxA, da);
	pwm_set_duty(inv->HW->pwm.pwmIdxB, db);
}

void set_pole_volts_single_phase(double Va, double Vb, InverterSinglePhase_t *inv){

	//sets the voltage of each leg with respect to the dc bus negative terminal

	double da, db;

	da = Va/(inv->Vdc);
	db = Vb/(inv->Vdc);

	pwm_set_duty(inv->HW->pwm.pwmIdxA, da);
	pwm_set_duty(inv->HW->pwm.pwmIdxB, db);
}

void set_line_volts_three_phase(double Va, double Vb, double Vc, InverterThreePhase_t *inv){

	//uses sine-triangle PWM
	//sets line to neutral voltages assuming neutral point is at Vdc/2

	double da, db, dc;

	da = 0.5 + Va/(inv->Vdc);
	db = 0.5 + Vb/(inv->Vdc);
	dc = 0.5 + Vc/(inv->Vdc);

	pwm_set_duty(inv->HW->pwm.pwmIdxA, da);
	pwm_set_duty(inv->HW->pwm.pwmIdxB, db);
	pwm_set_duty(inv->HW->pwm.pwmIdxC, dc);
}

void set_pole_volts_three_phase(double Va, double Vb, double Vc, InverterThreePhase_t *inv){

	//sets each of the three pole voltages manually
	double da, db, dc;

	da = Va/(inv->Vdc);
	db = Vb/(inv->Vdc);
	dc = Vc/(inv->Vdc);

	pwm_set_duty(inv->HW->pwm.pwmIdxA, da);
	pwm_set_duty(inv->HW->pwm.pwmIdxB, db);
	pwm_set_duty(inv->HW->pwm.pwmIdxC, dc);
}
