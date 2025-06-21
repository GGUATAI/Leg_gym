#ifndef _GPRMC_H
#define _GPRMC_H

#include "main.h"
#include "cmsis_os.h"

#include "usart.h"

extern void GPRMC_Init(UART_HandleTypeDef *GPRMC_usart_handle);
extern void GPRMC_Tx_Test(void);

#endif
