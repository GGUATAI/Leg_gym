#include "gimbal_task.h"

#include "Chassis_Leg_Task.h"
#include "INS_task.h"

#include "GPRMC.h"

#include "usbd_cdc_if.h"
#include "tim.h"
#include "stm32h7xx_hal_tim.h"
#include "usart.h"

float gimbal_motor_tar_pos[2] = {0};
float gimbal_motor_tar_pos_vision[2] = {0};
pc_tx_communicate_t pc_tx_communicate;
pc_rx_communicate_t pc_rx_communicate;
uint8_t rx_buf[sizeof(pc_rx_communicate)] = {0};

osThreadId_t gimbal_task_handel;

void Gimbal_Init(void)
{
	osThreadAttr_t attr = {
			.name = "Gimbal_Task" ,
			.stack_size = 128 * 8 ,
			.priority = (osPriority_t) osPriorityRealtime7 , };
	gimbal_task_handel = osThreadNew(Gimbal_Task, NULL, &attr);
}

static uint32_t GPRMC_DWT = 0;
static float GPRMC_dt = 0.0f; 
static uint8_t GPRMC_cnt = 0;

uint32_t gimbal_diff;

void Gimbal_Task(void *argument)
{
    /* USER CODE BEGIN gimbal_task */
    while (INS.ins_flag == 0)
    {
        osDelay(2);
    }
	
	joint_motor_init( &chassis_move.joint_motor[4], 4, MIT_MODE );
	joint_motor_init( &chassis_move.joint_motor[5], 5, MIT_MODE ); 

//	DM_Motor_Set_Zeropoint(&hfdcan1, chassis_move.joint_motor[4] . para . id, chassis_move . joint_motor[4] . mode);
//	DM_Motor_Set_Zeropoint(&hfdcan1, chassis_move . joint_motor[5] . para . id, chassis_move . joint_motor[5] . mode);
	for( int j = 0; j < 10; j++ )
	{
		DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[4].para.id, chassis_move.joint_motor[4].mode );
	}
	osDelay( 200 );
	for( int j = 0; j < 10; j++ )
	{
		DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[5].para.id, chassis_move.joint_motor[5].mode );
	}
	
    gimbal_motor_tar_pos[0] = 0;
    gimbal_motor_tar_pos[1] = 0;
    pc_tx_communicate.header = 0x5A;
    gimbal_motor_tar_pos[0] = chassis_move.joint_motor[4].para.pos;
    gimbal_motor_tar_pos[1] = chassis_move.joint_motor[5].para.pos;
	
	uint32_t time = osKernelGetTickCount();
    /* Infinite loop */
    for (;;)
    {
        pc_tx_communicate.imu_roll = INS.Roll;
        pc_tx_communicate.imu_pitch = INS.Pitch;
        pc_tx_communicate.imu_yaw = INS.Yaw;
        pc_tx_communicate.motor_pitch = chassis_move.joint_motor[5].para.pos;
        pc_tx_communicate.motor_yaw = chassis_move.joint_motor[4].para.pos;
		pc_tx_communicate.body_height =( chassis_right_leg.height + chassis_left_leg.height ) / 2.0f;
		
        Append_CRC16_Check_Sum((uint8_t *)&pc_tx_communicate, sizeof(pc_tx_communicate));
        CDC_Transmit_HS((uint8_t *)&pc_tx_communicate, sizeof(pc_tx_communicate));

        if (Verify_CRC16_Check_Sum(rx_buf, sizeof(pc_rx_communicate)))
        {
            memcpy(&pc_rx_communicate, rx_buf, sizeof(pc_rx_communicate));
        }
		
		if (chassis_move.start_flag)
		{
			DM_MIT_Control (&hfdcan1, chassis_move . joint_motor[4] . para . id, gimbal_motor_tar_pos[0], 0.1f, 4.0f, 0.5f,
			          0.0f);
			DM_MIT_Control (&hfdcan1, chassis_move . joint_motor[5] . para . id, gimbal_motor_tar_pos[1], 0.1f, 4.0f, 0.5f,
			          0.0f);
		}
		else 
		{
			DM_MIT_Control (&hfdcan1, chassis_move . joint_motor[4] . para . id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			DM_MIT_Control (&hfdcan1, chassis_move . joint_motor[5] . para . id, 0.0f, 0.0f, 0.0f, 0.0f,
			          0.0f);
		}
		
		vofa_send_indefinite_len_data(data_view,DATA_VIEW_SEND_LEN);
		if(tim_pwm_test_flag  == 1)
		{
				GPRMC_Tx_Test();
				GPRMC_dt = DWT_GetDeltaT(&GPRMC_DWT);
				tim_pwm_test_flag = 0;
				GPRMC_cnt = 0;
		}
		gimbal_diff = osKernelGetTickCount() - time;
		time = osKernelGetTickCount();
		osDelayUntil(time + GIMBAL_TASK_PERIOD);
    }
    /* USER CODE END gimbal_task */
}

uint8_t tim_pwm_test_flag = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
 if ((htim == &htim1) && (htim->Channel == 0x01U))
 {
	tim_pwm_test_flag = 1;
	GPRMC_dt = DWT_GetDeltaT(&GPRMC_DWT);
	return;
 }
}
