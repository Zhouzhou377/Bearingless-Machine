#ifndef TASK_CABINET_H
#define TASK_CABINET_H

#include <stdint.h>
#include "sys/scheduler.h"

#define TASK_CABINET_UPDATES_PER_SEC		(10000)
#define TASK_CABINET_INTERVAL_USEC		(USEC_IN_SEC / TASK_CABINET_UPDATES_PER_SEC)

void task_cabinet_init(void);
void task_cabinet_deinit(void);
void task_cabinet_callback(void *arg);
uint8_t task_cabinet_is_inited(void);
void task_cabinet_stats_print(void);
void task_cabinet_stats_reset(void);

#endif // TASK_CABINET_H
