#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "main.h"
#include "cmsis_os.h"

typedef struct __attribute__((packed))
{
    uint8_t header; // 0x5A
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
    float motor_pitch;
    float motor_yaw;
	float body_height;
    uint16_t checksum;
} pc_tx_communicate_t;

typedef struct __attribute__((packed))
{
    uint8_t header; // 0xA5
	bool shoot;
	bool height_change;
	uint8_t nav_state;
	uint8_t reserved;
    float pitch;
    float yaw;
    float line_speed;
	float angular_speed;
	float target_body_height;
    uint16_t checksum;
} pc_rx_communicate_t;

extern float gimbal_motor_tar_pos[2];
extern float gimbal_motor_tar_pos_vision[2];
extern pc_tx_communicate_t pc_tx_communicate;
extern pc_rx_communicate_t pc_rx_communicate;

extern uint8_t tim_pwm_test_flag;

extern void Gimbal_Init(void);
extern void Gimbal_Task(void *argument);

#endif
