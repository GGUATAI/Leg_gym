/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef _PID_H
#define _PID_H

#include "stm32h7xx.h"
#include "main.h"
#include "cmsis_os.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次

} pid_t;

extern void PID_init(pid_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc(pid_t *pid, float ref, float set);
extern void PID_clear(pid_t *pid);
#endif
