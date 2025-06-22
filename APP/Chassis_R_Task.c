/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
    *						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
    *						 右下角的DM4310发送id为8、接收id为4，
    *						 右边DM轮毂电机发送id为1、接收id为0。
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "Chassis_R_Task.h"

#include "INS_Task.h"
#include "Observe_Task.h"

#include "bsp_can.h"

#include "VMC_calc.h"
#include "LPF.h"
#include "filter.h"
#include "pid.h"

osThreadId_t chassis_r_task_handel;

float LQR_K_R[12] = PARA;

static moving_average_filter_t right_F_N_maf;
static lowpass_filter1p_info_t right_F_N_lpf;
static ramp_function_source_t *right_F_0_ramp;

static void Chassis_R_Base_Init( chassis_leg_t *chassis, vmc_leg_t *vmc );

static void Chassis_R_Feedback_Update( chassis_leg_t *chassis, vmc_leg_t *vmc, INS_t *ins );

static void Right_Control( void );

void Chassis_R_Init( void )
{
	osThreadAttr_t attr = {
			.name = "Chassis_R_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityRealtime6, };
	chassis_r_task_handel = osThreadNew( Chassis_R_Task, NULL, &attr );
}

uint32_t chassis_r_diff;
uint8_t chassis_r_init_flag;

void Chassis_R_Task( void *argument )
{
	while( INS.ins_flag == 0 )
	{ // 等待加速度收敛
		osDelay( 2 );
	}

	Chassis_R_Base_Init( &chassis_move, &chassis_right_leg ); // 初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
	chassis_r_init_flag = 1;

	uint32_t time = osKernelGetTickCount( );
	while( 1 )
	{
		Right_Control( );

		chassis_r_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + CHASSIS_R_TASK_PERIOD );
	}
}

ramp_init_config_t chassis_r_length_pid_ramp_config =
		{
				.decrease_value = 0.01f,
				.increase_value = 0.01f,
				.frame_period = 0.001f,
				.max_value = 0.0f,
				.min_value = 0.0f,
				.ramp_state = SLOPE_FIRST_REAL,
		};

ramp_init_config_t chassis_r_F0_forward_config = {
		.decrease_value = 1.25f,
		.increase_value = 1.25f,
		.frame_period = 0.001f,
		.max_value = 0.0f,
		.min_value = 0.0f,
		.ramp_state = SLOPE_FIRST_REAL,
};

static void Chassis_R_Base_Init( chassis_leg_t *chassis, vmc_leg_t *vmc )
{
	joint_motor_init( &chassis->joint_motor[0], 0, MIT_MODE ); // 发送id为0
	joint_motor_init( &chassis->joint_motor[1], 1, MIT_MODE ); // 发送id为1

	DM_Motor_Set_Zeropoint( &hfdcan1, chassis->joint_motor[0].para.id, MIT_MODE );
	DM_Motor_Set_Zeropoint( &hfdcan1, chassis->joint_motor[1].para.id, MIT_MODE );

	VMC_Init( vmc ); // 给杆长赋值

	Average_Init( &right_F_N_maf, 20 );
	LowPass_Filter1p_Init( &right_F_N_lpf, 0.225f );

	right_F_0_ramp = ramp_init( &chassis_r_F0_forward_config );

	leg_r_length_pid.Kp = leg_length_pid_kp;
	leg_r_length_pid.Ki = leg_length_pid_ki;
	leg_r_length_pid.Kd = leg_length_pid_kd;
	leg_r_length_pid.Output_Max = leg_length_pid_max_out;
	leg_r_length_pid.output_LPF_RC = 0.125f;
	leg_r_length_pid.dead_band = 0.0001f;
	leg_l_length_pid.derivative_LPF_RC = 0.15f;
	leg_r_length_pid.ramp_target = ramp_init( &chassis_r_length_pid_ramp_config );
	leg_r_length_pid.improve = PID_DERIVATIVE_ON_MEASUREMENT | PID_DERIVATIVE_FILTER;//PID_RAMP_TARGET | PID_OUTPUT_FILTER;

	for( int j = 0; j < 10; j++ )
	{
		DM_Motor_ENABLE( &hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode );
	}
	osDelay( 200 );
	for( int j = 0; j < 10; j++ )
	{
		DM_Motor_ENABLE( &hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode );
	}
	osDelay( 200 );
}

static void Chassis_R_Feedback_Update( chassis_leg_t *chassis, vmc_leg_t *vmc, INS_t *ins )
{
	vmc->phi1 = PI / 2.0f + chassis_move.joint_motor[0].para.pos + 1.88106048f;
	vmc->phi4 = PI / 2.0f + chassis_move.joint_motor[1].para.pos - 1.88106048f;

	vmc->pitch = (ins->Pitch) * 0.8f + vmc->pitch * 0.2f;
	vmc->d_pitch = (ins->Gyro[0]) * 0.8f + vmc->pitch * 0.2f;

	chassis->pitch_l = vmc->pitch;
	chassis->pitch_gyro_l = vmc->d_pitch;
}

static void Right_Control( void )
{
	static float err[6] = { 0.0f };
	static uint32_t right_dwt = 0;
	static float dt = 0;

	Chassis_R_Feedback_Update( &chassis_move, &chassis_right_leg, &INS );

	if( chassis_right_leg.pitch > 0.34f || chassis_right_leg.pitch < -0.34f )
	{
		chassis_move.recover_flag = 1;
	}
	else
	{
		chassis_move.recover_flag = 0;
	}

	dt = DWT_GetDeltaT( &right_dwt );

	VMC_Calc_Base_Data( &chassis_right_leg, &INS, dt );

	chassis_right_leg.v = -chassis_move.v_filter;
	chassis_right_leg.x = -chassis_move.x_filter;

	chassis_right_leg.v_tar = -chassis_move.v_set;
	chassis_right_leg.x_tar = -chassis_move.x_set;

	if( ps_flag == 1 )
	{
		if( ( chassis_move.nomotion_start_cnt > 150 ) && ( chassis_move.nomotion_start_cnt < 600 ) )
		{
			for( int i = 0; i < 12; i++ )
			{
				LQR_K_R[i] = LQR_K_Calc( &Poly_Coefficient_Position[i][0], chassis_right_leg.L0 );
			}
		}
		else
		{
			for( int i = 0; i < 12; i++ )
			{
				LQR_K_R[i] = LQR_K_Calc( &Poly_Coefficient_Speed[i][0], chassis_right_leg.L0 );
			}
		}
	}
	else if( ps_flag == 2 )
	{
		if( ( chassis_move.nomotion_start_cnt > 150 ) )
		{
			for( int i = 0; i < 12; i++ )
			{
				LQR_K_R[i] = LQR_K_Calc( &Poly_Coefficient_Position[i][0], chassis_right_leg.L0 );
			}
		}
		else
		{
			for( int i = 0; i < 12; i++ )
			{
				LQR_K_R[i] = LQR_K_Calc( &Poly_Coefficient_Speed[i][0], chassis_right_leg.L0 );
			}
		}
	}
	else
	{
		for( int i = 0; i < 12; i++ )
		{
			LQR_K_R[i] = LQR_K_Calc( &Poly_Coefficient_Speed[i][0], chassis_right_leg.L0 );
		}
	}

	err[0] = offset_theta_set - chassis_right_leg.theta;
	err[1] = 0.0f - chassis_right_leg.d_theta;
	err[2] = chassis_right_leg.x_tar - chassis_right_leg.x - ( -offset_x_filter );
	err[3] = chassis_right_leg.v_tar - chassis_right_leg.v + ( -offset_v_set );
	err[4] = offset_pitch_set - chassis_right_leg.pitch;
	err[5] = 0.0f - chassis_right_leg.d_pitch;

	chassis_move.wheel_motor[0].wheel_T = 0.0f;

	for( int i = 0; i < 6; i++ )
	{
		chassis_move.wheel_motor[0].wheel_T += LQR_K_R[i] * err[i];
	}
	// 转向环
	chassis_move.wheel_motor[0].wheel_T += chassis_move.turn_T;

	chassis_right_leg.Tp = 0.0f;

	for( int i = 6; i < 12; i++ )
	{
		chassis_right_leg.Tp -= LQR_K_R[i] * err[i - 6];
	}

	chassis_right_leg.Tp += chassis_move.leg_tp;
	// 防劈腿
	chassis_right_leg.F_0_forward = MG / arm_cos_f32( chassis_right_leg.theta );
//	chassis_right_leg.F_0_forward = ramp_calc( right_F_0_ramp, chassis_right_leg.F_0_forward );
	// 支持力
	chassis_right_leg.F0 = MG / arm_cos_f32( chassis_right_leg.theta ) +
						   PID_Position( &leg_r_length_pid, chassis_right_leg.L0, chassis_move.leg_set );

	if( chassis_move.standup_start_cnt < 250 )
	{
		chassis_right_leg.F0 = float_constrain( chassis_right_leg.F0, ( -10.0f ), ( 10.0f ) );
	}

	chassis_right_leg.T1 = chassis_move.joint_motor[2].para.tor;
	chassis_right_leg.T2 = chassis_move.joint_motor[3].para.tor;

	Leg_Force_Calc( &chassis_right_leg );

	chassis_right_leg.F_N = 140.05f + LowPass_Filter1p_Update( &right_F_N_lpf, chassis_right_leg.F_N );

	VMC_FN_Ground_Detection_R( &chassis_right_leg, &right_F_N_maf, 20.0f );

	data_view[8] = chassis_right_leg.F0;

	data_view[9] = chassis_right_leg.F_N;

	if( chassis_move.recover_flag == 0 && chassis_right_leg.F_N < 20.0f && chassis_move.leg_length_change_flag == 0 )
	{ // 离地了
		chassis_right_leg.off_ground_flag = 1;
		chassis_move.wheel_motor[0].wheel_T = 0.0f;
		chassis_right_leg.Tp = 0.0f;
		chassis_right_leg.Tp -= LQR_K_R[6] * err[0];
		chassis_right_leg.Tp -= LQR_K_R[7] * err[1];
		chassis_right_leg.Tp += chassis_move.leg_tp;
	}
	else
	{
	chassis_right_leg.off_ground_flag = 0;
	chassis_right_leg.F0 -= chassis_move.roll_f0;
	}

	Leg_Saturate( &chassis_right_leg.F0, -1.0f * MAX_F0, MAX_F0 );

	chassis_right_leg.j11 = ( chassis_right_leg.l1 * arm_sin_f32( chassis_right_leg.phi0 - chassis_right_leg.phi3 ) *
							  arm_sin_f32( chassis_right_leg.phi1 - chassis_right_leg.phi2 ) ) /
							arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 );
	chassis_right_leg.j12 = ( chassis_right_leg.l1 * arm_cos_f32( chassis_right_leg.phi0 - chassis_right_leg.phi3 ) *
							  arm_sin_f32( chassis_right_leg.phi1 - chassis_right_leg.phi2 ) ) /
							( chassis_right_leg.L0 * arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 ) );
	chassis_right_leg.j21 = ( chassis_right_leg.l4 * arm_sin_f32( chassis_right_leg.phi0 - chassis_right_leg.phi2 ) *
							  arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi4 ) ) /
							arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 );
	chassis_right_leg.j22 = ( chassis_right_leg.l4 * arm_cos_f32( chassis_right_leg.phi0 - chassis_right_leg.phi2 ) *
							  arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi4 ) ) /
							( chassis_right_leg.L0 * arm_sin_f32( chassis_right_leg.phi3 - chassis_right_leg.phi2 ) );

	if( chassis_move.standup_start_cnt < 250 )
	{
		chassis_right_leg.Tp = float_constrain( chassis_right_leg.Tp, ( -1.0f ), ( 1.0f ) );
	}

	chassis_right_leg.torque_set[0] =
			chassis_right_leg.j11 * chassis_right_leg.F0 + chassis_right_leg.j12 * chassis_right_leg.Tp;
	chassis_right_leg.torque_set[1] =
			chassis_right_leg.j21 * chassis_right_leg.F0 + chassis_right_leg.j22 * chassis_right_leg.Tp;

	//	Leg_Saturate (&chassis_move.wheel_motor[0].wheel_T, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);
	Leg_Saturate( &chassis_right_leg.torque_set[0], -1.0f * MAX_TORQUE_DM4310, MAX_TORQUE_DM4310 );
	Leg_Saturate( &chassis_right_leg.torque_set[1], -1.0f * MAX_TORQUE_DM4310, MAX_TORQUE_DM4310 );
}

// TODO:跳跃：分阶段，缩腿，给支持力
