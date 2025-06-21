#ifndef __CHASSIS_R_TASK_H
#define __CHASSIS_R_TASK_H

#include "main.h"
#include "cmsis_os.h"

#include "Chassis_Leg_Task.h"

extern float LQR_K_R[12];

extern uint8_t chassis_r_init_flag;

extern void Chassis_R_Init(void);
extern void Chassis_R_Task(void *argument);

#endif
