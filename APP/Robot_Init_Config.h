#ifndef __ROBOT_INIT_CONFIG_H
#define __ROBOT_INIT_CONFIG_H

#include "main.h"
#include "cmsis_os.h"

extern void V_OS_Task_Init(void);
extern void V_MCU_Init(void);
extern void V_Device_Init(void);

#endif
