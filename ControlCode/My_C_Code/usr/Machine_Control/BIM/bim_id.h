#ifndef BIM_ID_H
#define BIM_ID_H
#include "sys/injection.h"
#include "usr/Machine_Control/BIM/bim_outloop_control.h"
extern inj_ctx_t inj_ctx_ctrl_bim[];
void bim_injection_callback (bim_control *data);
void bim_injection_init (void);
void bim_injection_sin(double w, double mag, double *theta, double *out, int Num_var);

#endif
