#ifndef _MAHONY_FILTER_H
#define _MAHONY_FILTER_H

#include "../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h"
#include "../../../Core/Inc/main.h"
#include "../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os.h"

/*************************************
完成时间：2023年09月02日
功能介绍：实现mahony姿态角解算算法的模块封装
知乎账号：龙胆也
B站账号：华南小虎队
***************************************/

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
    float x;
    float y;
    float z;
} Axis3f;

///*-----------------------MahonyAHRS滤波器----------------------*/
///*-----------------------------------------------------------*/
////比例项用于控制传感器的“可信度”，积分项用于消除静态误差。
////Kp越大，意味着通过加速度计得到误差后补偿越显著，即是越信任加速度计。
////反之 Kp越小时，加速度计对陀螺仪的补偿作用越弱，也就越信任陀螺仪。
////而积分项则用于消除角速度测量值中的有偏噪声，故对于经过零篇矫正的角速度测量值，一般选取很小的Ki。
////最后将补偿值补偿给角速度测量值，带入四元数微分方程中即可更新当前四元数。

// 定义 MAHONY_FILTER_t 结构体，用于封装 Mahony 滤波器的数据和函数
typedef struct MAHONY_FILTER_t
{
    // 输入参数
    float Kp, Ki;     // 比例和积分增益
    float dt;         // 采样时间间隔
    Axis3f gyro, acc; // 陀螺仪和加速度计数据

    // 过程参数
    float exInt, eyInt, ezInt; // 积分误差累计
    float q0, q1, q2, q3;      // 四元数
    float rMat[3][3];          // 旋转矩阵

    // 输出参数
    float pitch, roll, yaw; // 姿态角：俯仰角，滚转角，偏航角

    // 函数指针
    void (*Mahony_Init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*Mahony_Input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*Mahony_Update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*Mahony_Output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_Update)(struct MAHONY_FILTER_t *mahony_filter);
}mahony_filter_t;
///*-----------------------MahonyAHRS滤波器----------------------*/
///*-----------------------------------------------------------*/

// 函数声明
void Mahony_Init(mahony_filter_t *mahony_filter, float Kp, float Ki, float dt); // 初始化函数
void Mahony_Input(mahony_filter_t *mahony_filter, Axis3f gyro, Axis3f acc);     // 输入数据函数
void Mahony_Update(mahony_filter_t *mahony_filter);                             // 更新滤波器函数
void Mahony_Output(mahony_filter_t *mahony_filter);                             // 输出姿态角函数
void RotationMatrix_Update(mahony_filter_t *mahony_filter);                     // 更新旋转矩阵函数

#endif
