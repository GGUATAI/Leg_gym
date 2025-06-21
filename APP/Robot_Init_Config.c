#include "Robot_Init_Config.h"
#include "INS_Task.h"
#include "Observe_Task.h"
#include "Remote_Control_Task.h"
#include "Message_Center_Task.h"
#include "Chassis_Leg_Task.h"
#include "Chassis_L_Task.h"
#include "Chassis_R_Task.h"
#include "Gimbal_Task.h"

#include "bsp_can.h"
#include "bsp_PWM.h"

#include "BMI088driver.h"
#include "sbus.h"
#include "ws2812.h"
#include "GPRMC.h"

void V_OS_Task_Init( void )
{
	INS_Init( );
	Observe_Init( );
	Remote_Control_Init( );
	Gimbal_Init( );
	Chassis_L_Init( );
	Chassis_R_Init( );
	Message_Center_Init( );
}

void V_MCU_Init( void )
{
	DWT_Init( 480 );
	BMI088_Init( &hspi2, 0 );

	FDCAN1_Config( );
	FDCAN2_Config( );
	FDCAN3_Config( );
}

static const float roll_target = 0.0302489772f;

void V_Device_Init( void )
{
	ws2812_instance = WS2812_Init( ws2812_config );
	SBUS_Init( &huart5 );
	GPRMC_Init( &huart1 );
	Leg_Pensation_Init( &leg_Tp_pid, &leg_turn_pid);   // 补偿pid初始化
	HAL_TIM_PWM_Start_IT( &htim1, TIM_CHANNEL_3 );
	HAL_TIM_PWM_Start_IT( &htim1, TIM_CHANNEL_1 );//1Hz mid360 100msHigh 900msLow
	HAL_TIM_PWM_Start_IT( &htim2, TIM_CHANNEL_3 );//10Hz haser
	HAL_TIM_PWM_Start_IT( &htim2, TIM_CHANNEL_1 );

	chassis_move.leg_set = leg_init_set;
	chassis_move.x_set = 0.0f;
	chassis_move.v_set = 0.0f;
	chassis_move.turn_set = chassis_move.total_yaw;
	chassis_move.roll_set = roll_target;
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

