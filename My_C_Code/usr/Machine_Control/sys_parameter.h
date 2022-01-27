#ifndef SYS_PARAMETER_H
#define SYS_PARAMETER_H

#define BP3_ENABLE (0)
#define BIM_ENABLE (1)
#define BM_ENABLE (1)

#define ID_SYS (1)
#define ID_CCTRL (0)
#define ID_VCTRL (0)
#define ID_LEVCTRL (0)
//#define ID_STIFFNESS(1)

#define DEBUG_DFLUX (0)

#define ENCODER_BP3_PPR_BITS (10)
#define ENCODER_BP3_PPR      (1 << ENCODER_BP3_PPR_BITS)
#define M_PER_VOLT (4e-4)

#define PWM_DEADTIME_NS (200)
#endif // SYS_PARAMETER_H
