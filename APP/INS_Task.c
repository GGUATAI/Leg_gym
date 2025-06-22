/**
  *********************************************************************
  * @file      INS_Task.c/h
  * @brief    
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "INS_Task.h"
#include "Remote_Control_Task.h"
#include "Observe_Task.h"

#include "QuaternionEKF.h"
#include "mahony_filter.h"
#include "LPF.h"

#include "bsp_PWM.h"

osThreadId_t INS_task_handel;

#ifdef Get_Bias
float bias[4] = {0, 0, 0};
uint32_t sum = 0;
#endif

INS_t INS;

lowpass_filter1p_info_t  ins_acc_lpf[3];
lowpass_filter1p_info_t  ins_gyro_lpf[3];
lowpass_filter1p_info_t  ins_angle_lpf[3];

mahony_filter_t mahony;
Axis3f Gyro, Accel;
float gravity[3] = { 0, 0, 9.81f };

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

void INS_Init( void )
{
	osThreadAttr_t attr = {
			.name = "INS_Task",
			.stack_size = 128 * 8,
			.priority = ( osPriority_t )osPriorityRealtime, };
	INS_task_handel = osThreadNew( INS_Task, NULL, &attr );

	Mahony_Init( &mahony, 1.0f, 0.0f, 0.001f );
	INS.AccelLPF = 0.0125f;

	LowPass_Filter1p_Init(&ins_acc_lpf[0],0.125f);
	LowPass_Filter1p_Init(&ins_acc_lpf[1],0.125f);
	LowPass_Filter1p_Init(&ins_acc_lpf[2],0.125f);
	LowPass_Filter1p_Init(&ins_gyro_lpf[0],0.125f);
	LowPass_Filter1p_Init(&ins_gyro_lpf[1],0.125f);
	LowPass_Filter1p_Init(&ins_gyro_lpf[2],0.125f);
	LowPass_Filter1p_Init(&ins_angle_lpf[0],0.225f);
	LowPass_Filter1p_Init(&ins_angle_lpf[1],0.225f);
	LowPass_Filter1p_Init(&ins_angle_lpf[2],0.225f);
}

uint32_t INS_diff;

static void BodyFrame_To_EarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrame_To_BodyFrame(const float *vecEF, float *vecBF, float *q);

void INS_Task( void *argument )
{
	uint32_t time = osKernelGetTickCount( );

	while( 1 )
	{
		ins_dt = DWT_GetDeltaT( &INS_DWT_Count );

		mahony.dt = ins_dt;

		BMI088_Read( &BMI088 );

#ifdef Get_Bias
		bias[0] += BMI088.Gyro[0];
		bias[1] += BMI088.Gyro[1];
		bias[2] += BMI088.Gyro[2];
		sum++;
#else
		BMI088.Gyro[0] -= 794.821716f / 291947;
		BMI088.Gyro[1] -= 675.006165f / 291947;
		BMI088.Gyro[2] -= 796.894104f / 291947;
		// 浣滃紛 鍙�浠ヨ�﹜aw寰堢ǔ瀹� 鍘绘帀姣旇緝灏忕殑鍊�
		if( fabsf( BMI088.Gyro[2] ) < 0.005f )
		{
			BMI088.Gyro[2] = 0;
		}
#endif

		INS.Accel[X] = BMI088.Accel[X];
		INS.Accel[X] = LowPass_Filter1p_Update(&ins_acc_lpf[X],INS.Accel[X]);
		INS.Accel[Y] = BMI088.Accel[Y];
		INS.Accel[Y] = LowPass_Filter1p_Update(&ins_acc_lpf[Y],INS.Accel[Y]);
		INS.Accel[Z] = BMI088.Accel[Z];
		INS.Accel[Z] = LowPass_Filter1p_Update(&ins_acc_lpf[Z],INS.Accel[Z]);
		Accel.x = INS.Accel[X];
		Accel.y = INS.Accel[Y];
		Accel.z = INS.Accel[Z];

		INS.Gyro[X] = BMI088.Gyro[X];
		INS.Gyro[X] = LowPass_Filter1p_Update(&ins_gyro_lpf[X],INS.Gyro[X]);
		INS.Gyro[Y] = BMI088.Gyro[Y];
		INS.Gyro[Y] = LowPass_Filter1p_Update(&ins_gyro_lpf[Y],INS.Gyro[Y]);
		INS.Gyro[Z] = BMI088.Gyro[Z];
		INS.Gyro[Z] = LowPass_Filter1p_Update(&ins_gyro_lpf[Z],INS.Gyro[Z]);
		Gyro.x = INS.Gyro[X];
		Gyro.y = INS.Gyro[Y];
		Gyro.z = INS.Gyro[Z];

		Mahony_Input( &mahony, Gyro, Accel );
		Mahony_Update( &mahony );
		Mahony_Output( &mahony );
		RotationMatrix_Update( &mahony );

		INS.q[0] = mahony.q0;
		INS.q[1] = mahony.q1;
		INS.q[2] = mahony.q2;
		INS.q[3] = mahony.q3;

		// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
		float gravity_b[3];
		EarthFrame_To_BodyFrame( gravity, gravity_b, INS.q );
		for( uint8_t i = 0; i < 3; i++ ) // 同锟斤拷锟斤拷一锟斤拷锟斤拷通锟剿诧拷
		{
			INS.MotionAccel_b[i] = ( INS.Accel[i] - gravity_b[i] ) * ins_dt / ( INS.AccelLPF + ins_dt ) +
								   INS.MotionAccel_b[i] * INS.AccelLPF / ( INS.AccelLPF + ins_dt );
			//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
			//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
		}
		BodyFrame_To_EarthFrame( INS.MotionAccel_b, INS.MotionAccel_n, INS.q ); // 转锟斤拷锟截碉拷锟斤拷系n

		// 锟斤拷锟斤拷锟斤拷锟斤拷
		if( fabsf( INS.MotionAccel_n[0] ) < 0.02f )
		{
			INS.MotionAccel_n[0] = 0.0f; // x轴
		}
		if( fabsf( INS.MotionAccel_n[1] ) < 0.02f )
		{
			INS.MotionAccel_n[1] = 0.0f; // y轴
		}
		if( fabsf( INS.MotionAccel_n[2] ) < 0.06f )
		{
			INS.MotionAccel_n[2] = 0.0f; // z轴
			stop_time++;
		}
		if( stop_time > 10 )
		{ // 锟斤拷止10ms
			stop_time = 0;
			INS.v_n = 0.0f;
		}

		if( ins_time > 3000.0f )
		{
			INS.v_n = INS.v_n + INS.MotionAccel_n[1] * 0.001f;
			INS.x_n = INS.x_n + INS.v_n * 0.001f;
			INS.ins_flag = 1; // 四元数收敛
			INS.Pitch = mahony.roll;
			INS.Pitch = LowPass_Filter1p_Update(&ins_angle_lpf[0],INS.Pitch);
			INS.Roll = mahony.pitch;
			INS.Roll = LowPass_Filter1p_Update(&ins_angle_lpf[1],INS.Roll);
			INS.Yaw = mahony.yaw;
			INS.Yaw = LowPass_Filter1p_Update(&ins_angle_lpf[2],INS.Yaw);

			// INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

			if( INS.Yaw - INS.YawAngleLast > 3.1415926f )
			{
				INS.YawRoundCount--;
			}
			else if( INS.Yaw - INS.YawAngleLast < -3.1415926f )
			{
				INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.2831852f * INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
			ins_time++;
		}
		INS_diff = osKernelGetTickCount( ) - time;
		time = osKernelGetTickCount( );
		osDelayUntil( time + INS_TASK_PERIOD );
	}
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrame_To_EarthFrame( const float *vecBF, float *vecEF, float *q )
{
	vecEF[0] = 2.0f * ( ( 0.5f - q[2] * q[2] - q[3] * q[3] ) * vecBF[0] +
						( q[1] * q[2] - q[0] * q[3] ) * vecBF[1] +
						( q[1] * q[3] + q[0] * q[2] ) * vecBF[2] );

	vecEF[1] = 2.0f * ( ( q[1] * q[2] + q[0] * q[3] ) * vecBF[0] +
						( 0.5f - q[1] * q[1] - q[3] * q[3] ) * vecBF[1] +
						( q[2] * q[3] - q[0] * q[1] ) * vecBF[2] );

	vecEF[2] = 2.0f * ( ( q[1] * q[3] - q[0] * q[2] ) * vecBF[0] +
						( q[2] * q[3] + q[0] * q[1] ) * vecBF[1] +
						( 0.5f - q[1] * q[1] - q[2] * q[2] ) * vecBF[2] );
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrame_To_BodyFrame( const float *vecEF, float *vecBF, float *q )
{
	vecBF[0] = 2.0f * ( ( 0.5f - q[2] * q[2] - q[3] * q[3] ) * vecEF[0] +
						( q[1] * q[2] + q[0] * q[3] ) * vecEF[1] +
						( q[1] * q[3] - q[0] * q[2] ) * vecEF[2] );

	vecBF[1] = 2.0f * ( ( q[1] * q[2] - q[0] * q[3] ) * vecEF[0] +
						( 0.5f - q[1] * q[1] - q[3] * q[3] ) * vecEF[1] +
						( q[2] * q[3] + q[0] * q[1] ) * vecEF[2] );

	vecBF[2] = 2.0f * ( ( q[1] * q[3] + q[0] * q[2] ) * vecEF[0] +
						( q[2] * q[3] - q[0] * q[1] ) * vecEF[1] +
						( 0.5f - q[1] * q[1] - q[2] * q[2] ) * vecEF[2] );
}
