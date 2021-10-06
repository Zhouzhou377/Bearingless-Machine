#ifndef BIM_ID_H
#define BIM_ID_H
#include "sys/injection.h"
#include "usr/Cabinet_test/outloop_control.h"
extern inj_ctx_t inj_ctx_ctrl[];
void BIM_injection_callback (bim_control *data);
void BIM_injection_init (void);
void BIM_injection_sin(double w, double mag, double *theta, double *out, int Num_var);

#endif
