#ifndef  __CHASSIS_L_TASK_H
#define __CHASSIS_L_TASK_H

#include "main.h"
#include "cmsis_os.h"

#include "Chassis_Leg_Task.h"

extern float LQR_K_L[12];

extern uint8_t chassis_l_init_flag;

extern void Chassis_L_Init(void);
extern void Chassis_L_Task(void *argument);

#endif
