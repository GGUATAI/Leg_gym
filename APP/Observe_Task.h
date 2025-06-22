#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "main.h"
#include "cmsis_os.h"

extern uint8_t observe_flag;

extern void Observe_Init(void);
extern void Observe_Task(void *argument);

#endif
