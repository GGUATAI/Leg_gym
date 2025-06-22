#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "stm32h7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_can.h"

#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f

typedef struct
{
    uint16_t id;
    uint16_t state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
	float last_tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_callback_t;

typedef struct
{
    uint16_t mode;
    motor_callback_t para;
} Joint_Motor_t;

typedef struct
{
    uint16_t mode;
    float wheel_T;

    motor_callback_t para;
} Wheel_Motor_t;

extern void DM8009p_Decode(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
extern void DM4310_Decode(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
extern void DJI3508_Decode(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);

extern void DM_Motor_Set_Zeropoint(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

extern void DM_Motor_ENABLE(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);
extern void DM_Motor_DISABLE(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id);

// �ؽڵ��
extern void DM_MIT_Control(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
extern void DM_Position_Control(hcan_t *hcan, uint16_t motor_id, float pos, float vel);
extern void DM_Speed_Control(hcan_t *hcan, uint16_t motor_id, float _vel);

// ��챵��MITģʽ��������
extern void mit_ctrl2(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);

extern void DJI3508_Control(hcan_t *hcan, float torq1, float torq2);

extern void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode);
extern void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode);

extern float Hex_To_Float(uint32_t *Byte, int num); // ʮ�����Ƶ�������
extern uint32_t FloatTohex(float HEX);              // ��������ʮ������ת��

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* __DM_MOTOR_H__ */
