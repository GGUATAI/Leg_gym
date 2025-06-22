/**
  *********************************************************************
  * @file      Remote_Control_Task.c/h
  * @brief     该任务是读取并处理天地飞传来的遥控数据，
  *           将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "Remote_Control_Task.h"

#include "Chassis_Leg_Task.h"
#include "Chassis_R_Task.h"
#include "Chassis_L_Task.h"
#include "Observe_Task.h"
#include "INS_Task.h"
#include "Gimbal_Task.h"

#include "gpio.h"

#include "VMC_calc.h"
#include "LPF.h"
#include "filter.h"
#include "pid.h"

ramp_init_config_t chassis_ramp_config_wheel_speed = {
		.decrease_value = 0.15f,
		.increase_value = 0.025f,
		.frame_period = 0.001f,
		.max_value = 0.0f,
		.min_value = 0.0f,
		.ramp_state = SLOPE_FIRST_REAL, };

ramp_init_config_t chassis_ramp_config_offset_speed = {
		.decrease_value = 0.15f,
		.increase_value = 0.015f,
		.frame_period = 0.001f,
		.max_value = 0.0f,
		.min_value = 0.0f,
		.ramp_state = SLOPE_FIRST_REAL, };		

ramp_function_source_t *chassis_wheel_ramp_speed;
ramp_function_source_t *chassis_offset_ramp_speed;

osThreadId_t remote_control_task_handel;

float map_range( float input, float in_min, float in_max, float out_min, float out_max )
{
	return ( input - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}

void Remote_Control_Init( void )
{
	const osThreadAttr_t attr = {
			.name = "Remote_Control_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityHigh7, };
	remote_control_task_handel = osThreadNew( Remote_Control_Task, NULL, &attr );
}

uint32_t remote_control_diff;
int8_t block_v_flag;
float block_v = 0.20f;

void Remote_Control_Task( void *argument )
{
	static uint32_t RC_DWT = 0;
	static float dt = 0;
	dt = DWT_GetDeltaT( &RC_DWT );

	chassis_wheel_ramp_speed = ramp_init( &chassis_ramp_config_wheel_speed );
	chassis_offset_ramp_speed = ramp_init( &chassis_ramp_config_offset_speed );
	
	uint32_t time = osKernelGetTickCount( );
	while( 1 )
	{
		sbus_frame_parse( &remoter, remoter_buff );
		
		dt = DWT_GetDeltaT( &RC_DWT );
		if( ( remoter.toggle.SF == RC_SW_UP ) && ( remoter.online == 1 ) )
		{
			chassis_move.start_flag = 1;
		}
		else
		{
			chassis_move.start_flag = 0;
			chassis_move.x_filter = 0.0f;
			chassis_move.x_set = chassis_move.x_filter;
			chassis_move.leg_set = leg_init_set;
			chassis_move.turn_set = chassis_move.total_yaw;
			gimbal_motor_tar_pos[0] = chassis_move.joint_motor[4].para.pos;
			gimbal_motor_tar_pos[1] = chassis_move.joint_motor[5].para.pos;
		}

		if(chassis_move.leg_control_flag == 1)
		{
			if(pc_rx_communicate.height_change == 1)
			{
				chassis_move.leg_set = pc_rx_communicate.target_body_height;
				chassis_move.leg_length_change_flag = 1;
			}
			else
			{
				chassis_move.leg_length_change_flag = 0;
			}

			if( chassis_move.leg_set > leg_max_set )
			{
				chassis_move.leg_set = leg_max_set;
			}
			else if( chassis_move.leg_set < leg_min_set )
			{
				chassis_move.leg_set = leg_min_set;
			}
		}

		switch( remoter.toggle.SA )
		{
			if(chassis_move.leg_control_flag == 0)
				case RC_SW_UP:
					if(chassis_move.leg_control_flag == 0)
					{
						if( remoter.rc.ch[RC_CH_RY] )
						{
							chassis_move.leg_set += ( float )remoter.rc.ch[RC_CH_RY] * 0.000005f; // 调腿长
							chassis_move.leg_length_change_flag = 1;
						}
						else
						{
							chassis_move.leg_length_change_flag = 0;
						}
					}

				if( chassis_move.leg_set > leg_max_set )
				{
					chassis_move.leg_set = leg_max_set;
				}
				else if( chassis_move.leg_set < leg_min_set )
				{
					chassis_move.leg_set = leg_min_set;
				}

				if( remoter.toggle.SB == RC_SW_MID )
				{
					chassis_move.leg_set = leg_init_set;
				}
				break;
				case RC_SW_MID:
					if(chassis_move.leg_control_flag == 0)
					{
						chassis_move.v_set = ramp_calc( chassis_wheel_ramp_speed, ( ( float )remoter.rc.ch[RC_CH_RY] *
																					0.0025f ) );
					}
					else if(chassis_move.leg_control_flag == 1)
					{
						chassis_move.v_set = ramp_calc( chassis_wheel_ramp_speed, pc_rx_communicate.line_speed );
						if(user_abs(chassis_move.v_set) < 0.025f )
						{
							chassis_move.v_set = 0.0f;
						}

						if(chassis_move.v_set < -0.7f)
						{
							chassis_move.v_set = -0.7f;
						}
						else if(chassis_move.v_set > 0.7f)
						{
							chassis_move.v_set = 0.7f;
						}
					}

				if( user_abs( chassis_move.v_set ) > 0.6f)
				{
					if(block_v_flag == 0)
					{
					if( chassis_move.v_set > 0.0f )
					{
						block_v_flag = 1;
					}
					else if( chassis_move.v_set < 0.0f )
					{
						block_v_flag = -1;
					}
				    }
				}

				if( ( block_v_flag != 0 ) && ( chassis_move.v_set == 0 ) )
				{
					if( block_v_flag == -1 )
					{
						if( chassis_move.nomotion_start_cnt > 225 )
						{

							offset_v_set = ramp_calc( chassis_offset_ramp_speed, -block_v );
							chassis_move.x_filter = 0.0f;
						}
					}
					else if( block_v_flag == 1 )
					{
						if( chassis_move.nomotion_start_cnt > 225 )
						{
							offset_v_set = ramp_calc( chassis_offset_ramp_speed, block_v );
							chassis_move.x_filter = 0.0f;
						}
					}
					if( chassis_move.nomotion_start_cnt > 525 )
					{
						chassis_offset_ramp_speed->out = 0.0f;
						chassis_offset_ramp_speed->target = 0.0f;
						chassis_offset_ramp_speed->plan_value = 0.0f;
						chassis_offset_ramp_speed->real_value = 0.0f;
						offset_v_set = 0.0f;
						block_v_flag = 0;
					}
				}
				else
				{
					offset_v_set = 0.0f;
				}

				chassis_move.x_set = 0.0f;

				if(chassis_move.leg_control_flag == 0)
				{
					chassis_move.wz_set = -( float )remoter.rc.ch[RC_CH_RX] * 0.00125f;
				}
				else if(chassis_move.leg_control_flag == 1)
				{
					chassis_move.wz_set = -pc_rx_communicate.angular_speed * 0.8f + chassis_move.wz_set * 0.2f;
				}

				if(user_abs(chassis_move.wz_set) < 0.025f)
				{
					chassis_move.wz_set = 0.0f;
				}

				if(chassis_move.wz_set < -0.6f)
				{
					chassis_move.wz_set = -0.6f;
				}
				else if(chassis_move.wz_set > 0.6f)
				{
					chassis_move.wz_set = 0.6f;
				}

				if( chassis_move.wz_set != 0 )
				{
					chassis_move.turn_set = chassis_move.total_yaw + chassis_move.wz_set;
				}
				break;
			case RC_SW_DOWN:
				switch( remoter.toggle.SB )
				{
					case RC_SW_UP:
						
						break;
					case RC_SW_MID:
						DM_Motor_DISABLE( &hfdcan1, chassis_move.joint_motor[2].para.id, chassis_move.joint_motor[2]
								.mode );
						DM_Motor_DISABLE( &hfdcan1, chassis_move.joint_motor[3].para.id, chassis_move.joint_motor[3]
								.mode );
						DM_Motor_DISABLE( &hfdcan1, chassis_move.joint_motor[0].para.id, chassis_move.joint_motor[0]
								.mode );
						DM_Motor_DISABLE( &hfdcan1, chassis_move.joint_motor[1].para.id, chassis_move.joint_motor[1]
								.mode );
						break;
					case RC_SW_DOWN:
						DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[2].para.id, chassis_move.joint_motor[2]
								.mode );
						DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[3].para.id, chassis_move.joint_motor[3]
								.mode );
						DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[0].para.id, chassis_move.joint_motor[0]
								.mode );
						DM_Motor_ENABLE( &hfdcan1, chassis_move.joint_motor[1].para.id, chassis_move.joint_motor[1]
								.mode );
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		if(remoter.toggle.SA != RC_SW_DOWN)
		{
			switch( remoter.toggle.SB )
			{
				case RC_SW_UP:
					chassis_move.leg_control_flag = 0;
					break;
				case RC_SW_MID:
					chassis_move.leg_control_flag = 1;
					break;
				case RC_SW_DOWN:
					chassis_move.leg_control_flag = 1;
					break;
				default:
					break;
			}
		}
		switch( remoter.toggle.SC )
		{
			case RC_SW_DOWN:
				remoter.rc.ch[RC_CH_LX] += 4;
				gimbal_motor_tar_pos[0] -= ( float )remoter.rc.ch[RC_CH_LX] * 0.00001f;
				if( gimbal_motor_tar_pos[0] > PI / 2 )
				{
					gimbal_motor_tar_pos[0] = PI / 2;
				}
				else if( gimbal_motor_tar_pos[0] < -PI / 2 )
				{
					gimbal_motor_tar_pos[0] = -PI / 2;
				}

				gimbal_motor_tar_pos[1] = map_range( ( float )remoter.rc.ch[RC_CH_LY], -671, 671, -0.6, 1 );
				switch( remoter.toggle.SD )
				{
					case RC_SW_UP:
						break;
//            case RC_SW_MID:
//                Power_OUT1_ON;
//                break;
//            case RC_SW_DOWN:
//                Power_OUT1_OFF;
//                break;
					default:
						break;
				}
				break;
			case RC_SW_MID:
				gimbal_motor_tar_pos[0] = map_range( pc_rx_communicate.yaw, -1, 1, -PI / 2, PI / 2 );
				gimbal_motor_tar_pos[1] = map_range( pc_rx_communicate.pitch, -1, 1, -0.6, 0.6 );
//            if (pc_rx_communicate.shoot)
//                Power_OUT1_ON;
//            else
//                Power_OUT1_OFF;
				break;
		}

		remote_control_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + REMOTE_CONTROL_TASK_PERIOD );
	}
}
