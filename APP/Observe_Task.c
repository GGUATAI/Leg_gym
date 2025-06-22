/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
    * 					 原理来源于https://zhuanlan.zhihu.com/p/689921165
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "Observe_Task.h"

#include "INS_Task.h"
#include "Chassis_Leg_Task.h"

#include "kalman_filter.h"
#include "VMC_calc.h"
#include "LPF.h"
#include "filter.h"
#include "pid.h"

osThreadId_t observe_task_handel;

moving_average_filter_t v_wheel_moving_filter;

lowpass_filter1p_info_t v_l_lpf_filter;

lowpass_filter1p_info_t v_r_lpf_filter;

lowpass_filter1p_info_t v_wheel_lpf_filter;

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体

static float vaEstimateKF_F[4] = {
		1.0f, 0.002f,
		0.0f, 1.0f }; // 状态转移矩阵，控制周期为0.001s

static float vaEstimateKF_P[4] = {
		1.0f, 0.0f,
		0.0f, 1.0f }; // 后验估计协方差初始值

static float vaEstimateKF_Q[4] = {
		0.05f, 0.0f,
		0.0f, 0.01f }; // Q预测矩阵初始值 0--速度噪声 3--加速度噪声  增益

static float vaEstimateKF_R[4] = {
		1200.0f, 0.0f,
		0.0f, 2000.0f }; // R量测噪声矩阵初始值 0--速度噪声 3--加速度噪声  惩罚

float vaEstimateKF_K[4];

static const float vaEstimateKF_H[4] = {
		1.0f, 0.0f,
		0.0f, 1.0f }; // 设置矩阵H为常量

static void xvEstimateKF_Init( KalmanFilter_t *EstimateKF );
static void xvEstimateKF_Update( KalmanFilter_t *EstimateKF, float acc, float vel );		
		
void Observe_Init( void )
{
	osThreadAttr_t attr = {
			.name = "Observe_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityRealtime7, };
	observe_task_handel = osThreadNew( Observe_Task, NULL, &attr );

	Average_Init( &v_wheel_moving_filter, 10 );

	LowPass_Filter1p_Init( &v_l_lpf_filter, 0.125f );

	LowPass_Filter1p_Init( &v_r_lpf_filter, 0.125f );

	LowPass_Filter1p_Init( &v_wheel_lpf_filter, 0.175f );
}

uint32_t observe_diff;
uint8_t observe_flag;

static float v_l_temp;
static float v_r_temp;
static float wheel_v_temp;
static float fusion_v_data[2];

float Q_1 = 1.0f;
float Q_3 = 0.01f;
float R_1 = 1.0f;
float R_3 = 2000.0f;
float leg_length_kp = 100.0f;
float leg_length_kd = 15.0f;
float temp_a = 1.0f;

void Observe_Task( void *argument )
{
	while( INS.ins_flag == 0 )
	{ // 等待加速度收敛
		osDelay( 2 );
	}
	static float wheel_z_e_right, wheel_z_e_left = 0.0f;
	static float v_right_b_axis, v_left_b_axis = 0.0f;
	static float aver_v_observe = 0.0f;
	static float wheel_v = 0.0f;

	xvEstimateKF_Init( &vaEstimateKF );
	float last_vel_acc = 0.0f;

	static uint32_t OB_DWT = 0;
	static float dt = 0;
	dt = DWT_GetDeltaT( &OB_DWT );

	uint32_t time = osKernelGetTickCount( );
	while( 1 )
	{
		dt = DWT_GetDeltaT( &OB_DWT );

		chassis_move.total_yaw = INS.YawTotalAngle;
		chassis_move.roll = INS.Roll;
		chassis_move.theta_err = 0.0f - ( chassis_right_leg.theta + chassis_left_leg.theta );

		vaEstimateKF_F[1] = dt;
		
//		vaEstimateKF_Q[0] = Q_1;
//		vaEstimateKF_Q[3] = Q_3;
//		vaEstimateKF_R[0] = R_1;
//		vaEstimateKF_R[3] = R_3;

//		memcpy( vaEstimateKF.Q_data, vaEstimateKF_Q, sizeof( vaEstimateKF_Q ) );
//		memcpy( vaEstimateKF.R_data, vaEstimateKF_R, sizeof( vaEstimateKF_R ) );

//		leg_r_length_pid.Kp = leg_length_kp;
//		leg_l_length_pid.Kp = leg_length_kp;
//		leg_r_length_pid.Kd = leg_length_kd;
//		leg_l_length_pid.Kd = leg_length_kd;
		
		wheel_z_e_right = -chassis_move.wheel_motor[0].para.vel * 2 * PI + INS.Gyro[0] +
				   chassis_right_leg.d_alpha; // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		v_right_b_axis = wheel_z_e_right * WHEEL_R +
						 chassis_right_leg.L0 * chassis_right_leg.d_theta * arm_cos_f32( chassis_right_leg.theta ) +
						 chassis_right_leg.d_L0 * arm_sin_f32( chassis_right_leg.theta ); // 机体b系的速度

		wheel_z_e_left = -chassis_move.wheel_motor[1].para.vel * 2 * PI + INS.Gyro[0] +
				  chassis_left_leg.d_alpha; // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		v_left_b_axis = wheel_z_e_left * WHEEL_R +
						chassis_left_leg.L0 * chassis_left_leg.d_theta * arm_cos_f32( chassis_left_leg.theta ) +
						chassis_left_leg.d_L0 * arm_sin_f32( chassis_left_leg.theta ); // 机体b系的速度

		aver_v_observe = ( v_right_b_axis - v_left_b_axis ) / 2.0f; // 取平均

		v_r_temp = LowPass_Filter1p_Update( &v_r_lpf_filter, chassis_move.wheel_motor[0].para
				.vel );

		v_l_temp = LowPass_Filter1p_Update( &v_l_lpf_filter, chassis_move.wheel_motor[1].para
				.vel );

		wheel_v_temp = ( -v_r_temp + v_l_temp ) * WHEEL_R * 2 * PI /
					   2.0f;

		wheel_v_temp = LowPass_Filter1p_Update( &v_wheel_lpf_filter, wheel_v_temp );

		Average_Add( &v_wheel_moving_filter, wheel_v_temp );

		wheel_v = v_wheel_moving_filter.aver_value * 0.95f + aver_v_observe * 0.05f;
		
		xvEstimateKF_Update( &vaEstimateKF, -INS.MotionAccel_b[1], wheel_v );
		
	    data_view[10] = wheel_v;
		data_view[11] = fusion_v_data[0];
		data_view[12] = fusion_v_data[1];
		data_view[13] = chassis_move.v_set;
		data_view[14] = chassis_move.leg_set;

		if( observe_flag == 0 )
		{
			if( fabs( fusion_v_data[0] - last_vel_acc ) / dt < 0.0001f )
			{
				observe_flag = 1;
				chassis_move.x_filter = 0.0f;
			}
			else
			{
				last_vel_acc = fusion_v_data[0];
			}
		}
		else if( observe_flag == 1 )
		{ 
			;
		}

		if( chassis_move.start_flag == 1 )
		{
			chassis_move.standup_start_cnt++;

			chassis_move.v_filter = fusion_v_data[0];//wheel_v;

			if( chassis_move.v_set != 0.0f )
			{
				chassis_move.nomotion_start_cnt = 0;
				chassis_move.x_filter = 0.0f;
			}
			else
			{
				chassis_move.nomotion_start_cnt++;
				if( chassis_move.nomotion_start_cnt > 150 )
				{
					chassis_move.x_filter +=
							wheel_v * dt * temp_a;//( chassis_move.v_filter - chassis_move.v_set ) * dt;
				}
			}
		}
		else
		{
			chassis_move.nomotion_start_cnt = 0;
			chassis_move.standup_start_cnt = 0;
			chassis_move.v_filter = 0.0f;
			chassis_move.x_filter = 0.0f;
		}

		observe_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + OBSERVE_TASK_PERIOD );
	}
}

void xvEstimateKF_Init( KalmanFilter_t *EstimateKF )
{
	Kalman_Filter_Init( EstimateKF, 2, 0, 2 ); // 状态向量2维 没有控制量 测量向量2维

	memcpy( EstimateKF->F_data, vaEstimateKF_F, sizeof( vaEstimateKF_F ) );
	memcpy( EstimateKF->P_data, vaEstimateKF_P, sizeof( vaEstimateKF_P ) );
	memcpy( EstimateKF->Q_data, vaEstimateKF_Q, sizeof( vaEstimateKF_Q ) );
	memcpy( EstimateKF->R_data, vaEstimateKF_R, sizeof( vaEstimateKF_R ) );
	memcpy( EstimateKF->H_data, vaEstimateKF_H, sizeof( vaEstimateKF_H ) );
}

void xvEstimateKF_Update( KalmanFilter_t *EstimateKF, float acc, float vel )
{
	// 卡尔曼滤波器测量值更新
	EstimateKF->MeasuredVector[0] = vel; // 测量速度
	EstimateKF->MeasuredVector[1] = acc; // 测量加速度

	// 卡尔曼滤波器更新函数
	Kalman_Filter_Update( EstimateKF );

	// 提取估计值
	for( uint8_t i = 0; i < 2; i++ )
	{
		fusion_v_data[i] = EstimateKF->FilteredValue[i];
	}
}
