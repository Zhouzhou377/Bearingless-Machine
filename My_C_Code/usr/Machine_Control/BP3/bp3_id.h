#ifndef bp3_ID_H
#define bp3_ID_H
#include "sys/injection.h"
#include "usr/Machine_Control/BP3/bp3_outloop_control.h"
extern inj_ctx_t inj_ctx_ctrl_bp3[];
void bp3_injection_callback (bp3_control *data);
void bp3_injection_init (void);
void bp3_injection_sin(double w, double mag, double *theta, double *out, int Num_var);

#endif
