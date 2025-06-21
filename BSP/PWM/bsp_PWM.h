/**
 ******************************************************************************
 * @file	 bsp_PWM.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/3/1
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __BSP_IMU_PWM_H
#define __BSP_IMU_PWM_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#endif
