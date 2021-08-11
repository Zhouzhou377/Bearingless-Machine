#include "usr/Cabinet_test/transforms.h"
#include "usr/Cabinet_test/definitions.h"
#include <math.h>

void func_Clarke(double *afbe0, double *abc){

	afbe0[0] = 2/3*(abc[0] - 0.5*abc[1] - 0.5*abc[2]);
	afbe0[1] = 2/3*(abc[1] - abc[2]);
	afbe0[2] = 1/3*(abc[0] + abc[1] + abc[2]);

}

void func_Clarke_inverse(double *afbe0, double *abc){

	abc[0] = afbe0[0] + afbe0[2];
	abc[1] = -1/2*afbe0[0] + SQRT3/2*afbe0[1] + afbe0[2];
	abc[2] = -1/2*afbe0[0] - SQRT3/2*afbe0[1] + afbe0[2];

}

void func_Park(double *afbe0, double *dq0, double theta_rad){
	double cos_theta = cos(theta_rad);
	double sin_theta = sin(theta_rad);
	dq0[0] = cos_theta*afbe0[0] + sin_theta*afbe0[1];
	dq0[1] = -sin_theta*afbe0[0] + cos_theta*afbe0[1];
	dq0[2] = afbe0[2];

}

void func_Park_inverse(double *afbe0, double *dq0, double theta_rad){
	double cos_theta = cos(theta_rad);
	double sin_theta = sin(theta_rad);
	afbe0[0] = cos_theta*dq0[0] - sin_theta*dq0[1];
	afbe0[1] = sin_theta*dq0[0] + cos_theta*dq0[1];
	afbe0[2] = afbe0[2];

}

void abc_to_dq0(double *abc, double *dq0, double theta_rad){

	double afbe0[3];
	func_Clarke(&afbe0, abc);
	func_Park(&afbe0, dq0, theta_rad);

}

void dq0_to2_abc(double *abc, double *dq0, double theta_rad){

	double afbe0[3];
	func_Park_inverse(&afbe0, dq0, theta_rad);
	func_Clarke_inverse(&afbe0, abc);

}
