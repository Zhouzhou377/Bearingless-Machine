#ifndef TASK_CABINET_H
#define TASK_CABINET_H

#include <stdint.h>
#include <stdbool.h>
#include "sys/scheduler.h"
#include "usr/Cabinet_test/OpenloopVSI.h"
#define TASK_CABINET_UPDATES_PER_SEC		(10000)
#define TASK_CABINET_INTERVAL_USEC		(USEC_IN_SEC / TASK_CABINET_UPDATES_PER_SEC)

typedef struct cmd_signal {

    bool enable_openloop;
    bool enable_currentcontrol;
    bool enable_log;

} cmd_signal;

extern cmd_signal cmd_enable;

void task_cabinet_init(void);
void task_cabinet_deinit(void);
void task_cabinet_callback(void *arg);
bool task_cabinet_is_inited(void);
void task_cabinet_stats_print(void);
void task_cabinet_stats_reset(void);

#endif // TASK_CABINET_H
