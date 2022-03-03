#ifndef TASK_CABINET_H
#define TASK_CABINET_H

#include <stdint.h>
#include <stdbool.h>
#include "sys/scheduler.h"
#include "usr/Machine_Control/openloop_vsi.h"
#define TASK_CABINET_UPDATES_PER_SEC		(10000)
#define TASK_CABINET_INTERVAL_USEC		(USEC_IN_SEC / TASK_CABINET_UPDATES_PER_SEC)

typedef struct cmd_signal {

    bool enable_openloop;
    bool enable_current_control;
    bool enable_log;
    bool enable_testloop;
    bool enable_bim_control;
    bool enable_bim_VFcontrol;
    bool enable_bp3_control;
    bool enable_bp3_align;
    bool enable_inject_tq_cctrl;
    bool enable_inject_s1_cctrl;
    bool enable_inject_Fxy;
    bool enable_inject_tq_vref;
    bool enable_inject_s1_vref;

}cmd_signal;

extern cmd_signal cmd_enable;

void task_cabinet_init(void);
void task_cabinet_deinit(void);
void task_cabinet_callback(void *arg);
bool task_cabinet_is_inited(void);
void task_cabinet_stats_print(void);
void task_cabinet_stats_reset(void);
void task_loop_test(void);

#endif // TASK_CABINET_H
