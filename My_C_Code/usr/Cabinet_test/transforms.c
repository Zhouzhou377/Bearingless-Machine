#include "usr/Cabinet_test/transforms.h"
#include "usr/Cabinet_test/definitions.h"

void abc_to_xy(double *xy, double *abc){

	xy[0] = abc[0] - 0.5*(abc[1] + abc[2]);
	xy[1] = (SQRT3/2.0)*(abc[1] - abc[2]);

}

void xy_to_abc(double *xy, double *abc){

    double a = 1.0/3.0;
	double b = SQRT3/3.0;

	abc[0] = 2.0*a*xy[0];
	abc[1] = -a*xy[0] + b*xy[1];
	abc[2] = -a*xy[0] - b*xy[1];

}
