#ifndef __CHASSIS_LEG_TASK_H
#define __CHASSIS_LEG_TASK_H

#include "main.h"
#include "cmsis_os.h"

#include "dm_motor.h"
#include "pid.h"
#include "VMC_calc.h"

#include "digital_pid.h"

typedef struct
{
    Joint_Motor_t joint_motor[6];
    Wheel_Motor_t wheel_motor[2];

    float v_set;    // 期望速度，单位是m/s
    float x_set;    // 期望位置，单位是m
	float wz_set;	// 期望角速度，单位是rad/s

    float turn_set; // 期望yaw轴弧度
    float roll_set; // 期望roll轴弧度
    float leg_set;  // 期望腿长，单位是m
    float last_leg_set;

    float v_filter; // 滤波后的车体速度，单位是m/s
    float x_filter; // 滤波后的车体位置，单位是m

    float pitch_r;
    float pitch_gyro_r;
    float pitch_l;
    float pitch_gyro_l;
    float roll;
    float total_yaw;
    float theta_err; // 两腿夹角误差

    float turn_T;  // yaw轴补偿
    float roll_f0; // roll轴补偿
    float leg_tp;  // 防劈叉补偿

    uint8_t start_flag;          // 启动标志
    uint8_t body_offground_flag; // 车体离地标志
    uint8_t leg_length_change_flag;     // 腿长改变标志
	uint8_t leg_control_flag;

    uint8_t jump_flag;  // 跳跃标志
    uint8_t jump_flag2; // 跳跃标志

    uint8_t prejump_flag; // 预跳跃标志
    uint8_t recover_flag; // 一种情况下的倒地自起标志

	uint32_t nomotion_start_cnt;
	uint32_t standup_start_cnt;
} chassis_leg_t;

extern float Poly_Coefficient_Speed[12][4];
extern float Poly_Coefficient_Position[12][4];

extern float offset_theta_set;
extern float offset_pitch_set;
extern float offset_x_filter;
extern float offset_v_set;

extern float leg_init_set;
extern float leg_max_set;
extern float leg_min_set;

extern vmc_leg_t chassis_right_leg;
extern vmc_leg_t chassis_left_leg;
extern chassis_leg_t chassis_move;

extern const float MG;// 质量*重力加速度*高度
extern float roll_f0_forward;

extern uint8_t right_off_ground_flag;
extern uint8_t left_off_ground_flag;

extern digital_PID_t leg_l_length_pid; // 左腿的腿长pd
extern digital_PID_t leg_r_length_pid; // 右腿的腿长pd
extern digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
extern digital_PID_t leg_roll_speed_pid; // 横滚角补偿pd
extern digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
extern digital_PID_t leg_turn_pid; // 转向pd

// 保持腿长
extern float leg_length_pid_kp;
extern float leg_length_pid_ki;
extern float leg_length_pid_kd;
extern float leg_length_pid_max_out;

extern const uint8_t ps_flag;

extern void Leg_Saturate(float *in, float min, float max);
extern void Leg_Pensation_Init( pid_t *Tp, pid_t *turn);

#endif
