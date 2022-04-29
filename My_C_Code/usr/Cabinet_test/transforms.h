#ifndef TRANSFORMS_H
#define TRANSFORMS_H

void func_Clarke(double *afbe0, double *abc);
void func_Clarke_inverse(double *afbe0, double *abc);
void func_Park(double *afbe0, double *dq0, double theta_rad);
void func_Park_inverse(double *afbe0, double *dq0, double theta_rad);
void abc_to_dq0(double *abc, double *dq0, double theta_rad);
void dq0_to2_abc(double *abc, double *dq0, double theta_rad);
#endif // TRANSFORMS_H
