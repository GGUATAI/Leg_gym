#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"

typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN3_Config(void);
extern void FDCAN2_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

#endif
