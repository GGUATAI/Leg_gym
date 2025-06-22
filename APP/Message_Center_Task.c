#include "Message_Center_Task.h"

#include "Observe_Task.h"
#include "Chassis_Leg_Task.h"
#include "Chassis_L_Task.h"
#include "Chassis_R_Task.h"

#include "dm_motor.h"

#include "tim.h"

osThreadId_t message_center_task_handel;

void Message_Center_Init( void )
{
	const osThreadAttr_t attr = {
			.name = "Message_Center_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityRealtime7,
	};
	message_center_task_handel = osThreadNew( Message_Center_Task, NULL, &attr );
}

uint32_t message_center_diff;

void Message_Center_Task( void *argument )
{
	uint32_t left_beat;
	uint32_t right_beat;

	uint32_t time = osKernelGetTickCount( );

	left_beat = 0;
	right_beat = 0;

	while( 1 )
	{
		if( ( chassis_l_init_flag == 1 ) && ( chassis_r_init_flag == 1 ) )
		{
			left_beat++;
			right_beat++;

			if( chassis_move.start_flag == 1 )
			{
				if( ( left_beat % 2 ) == 0 )
				{
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[3].para
							.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_left_leg
											.torque_set[1] ); // chassis_left_leg.torque_set[1]

					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[2].para
							.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_left_leg
											.torque_set[0] ); // chassis_left_leg.torque_set[0]
				}
				else if( ( right_beat % 1 ) == 0 )
				{
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
									chassis_right_leg.torque_set[1] ); // chassis_right_leg.torque_set[1]

					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
									chassis_right_leg.torque_set[0] ); // chassis_right_leg.torque_set[0]
				}
				DJI3508_Control( &hfdcan3, chassis_move.wheel_motor[0].wheel_T,
								 chassis_move.wheel_motor[1].wheel_T ); // 右边边轮毂电机
			}
			else if( chassis_move.start_flag == 0 )
			{
				if( ( left_beat % 2 ) == 0 )
				{
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
									0.0f ); // chassis_left_leg.torque_set[1]
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[2].para
							.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f ); // chassis_left_leg.torque_set[0]
				}
				else if( ( right_beat % 1 ) == 0 )
				{
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
									0.0f ); // chassis_right_leg.torque_set[1]
					DM_MIT_Control( &hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
									0.0f ); // chassis_right_leg.torque_set[0]
				}
				DJI3508_Control( &hfdcan3, 0, 0 ); // 右边边轮毂电机
			}
			// 这里可以添加消息中心的处理逻辑，比如接收和发送消息等
		}

		chassis_move.turn_T = leg_turn_pid.Kp * ( chassis_move.turn_set - chassis_move.total_yaw ) -
							  leg_turn_pid.Kd * INS.Gyro[2]; // 这样计算更稳一点
		Leg_Saturate( &chassis_move.turn_T, -1.0f * leg_turn_pid.Output_Max, leg_turn_pid.Output_Max );

		if( chassis_move.standup_start_cnt > 300 )
		{
			chassis_move.roll_f0 = roll_f0_forward + leg_roll_position_pid.Kp * ( chassis_move.roll_set - chassis_move.roll )
//					PID_Position(&leg_roll_position_pid,chassis_move.roll,chassis_move.roll_set)
								    - leg_roll_position_pid.Kd * INS.Gyro[1];
			Leg_Saturate( &chassis_move.roll_f0,
						  -1.0f * leg_roll_position_pid.Output_Max, leg_roll_position_pid.Output_Max );
		}
		else
		{
			chassis_move.roll_f0 =
					leg_roll_position_pid.Kp * ( chassis_move.roll_set - chassis_move.roll ) -
					leg_roll_position_pid.Kd * INS.Gyro[1];
			chassis_move.roll_f0 = float_constrain( chassis_move.roll_f0, ( -0.25f ), ( 0.25f ) );
		}

		chassis_move.leg_tp = PID_Position( &leg_Tp_pid, chassis_move.theta_err, 0.0f ); // 防劈叉pid计算
		Leg_Saturate( &chassis_move.leg_tp, -1.0f * leg_Tp_pid.Output_Max, leg_Tp_pid.Output_Max );

		data_view[0] = chassis_move.roll_f0;
		data_view[1] = chassis_move.roll;
		data_view[2] = chassis_move.roll_set;
		data_view[3] = chassis_move.leg_tp;
		data_view[4] = leg_Tp_pid.p_out;
		data_view[5] = leg_Tp_pid.d_out;
		

		if( chassis_left_leg.off_ground_flag == 1 && chassis_right_leg.off_ground_flag == 1 )
		{
			chassis_move.body_offground_flag = 1;
			chassis_move.x_filter = 0.0f;
			chassis_move.x_set = 0.0f;
			chassis_move.turn_set = chassis_move.total_yaw;
		}
		else
		{
			chassis_move.body_offground_flag = 0;
		}

		if(ps_flag == 1)
		{
			if( ( chassis_move.nomotion_start_cnt > 150 ) && ( chassis_move.nomotion_start_cnt < 600 ) )
			{
				offset_pitch_set = 0.03f;
				offset_x_filter = -0.333f;
				offset_v_set = 0.0f;
			}
			else
			{
				offset_pitch_set = 0.09f;
				offset_x_filter = -2.25f;
				offset_v_set = 0.0f;
			}
		}
		else if(ps_flag == 2)
		{
			if( ( chassis_move.nomotion_start_cnt > 150 ) )
			{
				offset_pitch_set = 0.03f;
				offset_x_filter = -0.333f;
				offset_v_set = 0.0f;
			}
			else
			{
				offset_pitch_set = 0.09f;
				offset_x_filter = -2.25f;
				offset_v_set = 0.0f;
			}
		}

		if((chassis_right_leg.off_ground_flag == 1) && (chassis_left_leg.off_ground_flag == 1))
		{
			offset_pitch_set = 0.0f;
		}
		else
		{
			offset_pitch_set = -0.05f;
		}

		message_center_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + MESSAGE_CENTER_TASK_PERIOD );
	}
}
