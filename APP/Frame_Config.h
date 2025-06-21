#ifndef _FRAME_CONFIG_H
#define _FRAME_CONFIG_H

#define REMOTE_CONTROL_TASK_PERIOD 2
#define INS_TASK_PERIOD 1
#define CHASSIS_R_TASK_PERIOD 1
#define CHASSIS_L_TASK_PERIOD 1
#define OBSERVE_TASK_PERIOD 2
#define MESSAGE_CENTER_TASK_PERIOD 1
#define GIMBAL_TASK_PERIOD 5
#define CHASSIS_L_TIME 1
#define CHASSIS_R_TIME 1

//小板凳实验
#define PARA \
    {-26.02, -2.26, -5.63, -6.02, 9.17, 0.94, 17.13, 1.29, 3.73, 3.78, 23.10, 1.21}

#define REDUCTION_RATIO (268.0f / 17.0f) // 减速比
#define WHEEL_R (0.20f / 2)               // 轮子半径

#define MAX_F0 64.0f           // 最大前馈力
#define MAX_TORQUE_DM4310 15.0f // 最大扭矩
#define MAX_TORQUE_DJI3508 5.8f // 最大扭矩

#define DATA_VIEW_SEND_LEN 15

#endif
