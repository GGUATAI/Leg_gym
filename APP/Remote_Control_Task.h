#ifndef  __REMOTE_CONTROL_TASK_H
#define __REMOTE_CONTROL_TASK_H

#include "main.h"
#include "cmsis_os.h"

#include "sbus.h"

extern void Remote_Control_Init(void);
extern void Remote_Control_Task(void *argument);
void sbus_frame_parse(remoter_t *remoter, uint8_t *buf);

#endif
