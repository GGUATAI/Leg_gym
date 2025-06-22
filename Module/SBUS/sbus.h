#ifndef _SBUS_H
#define _SBUS_H

#include "main.h"
#include "cmsis_os.h"

#include "usart.h"

#define BUFF_RC_RECEIVE_SIZE 25
#define BUFF_REMOTER_SIZE 52
#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA 120

typedef enum
{
	RC_SW_UP,
	RC_SW_MID,
	RC_SW_DOWN
} toggle_t;

typedef enum
{
	RC_CH_LY,
	RC_CH_LX,
	RC_CH_RY,
	RC_CH_RX
} rc_ch_t;

typedef struct
{
	uint16_t online;
	struct
	{
		/*
		 *
		 * ch[0] : 左摇杆上下 3535-1024-1695
		 * ch[1] : 左摇杆左右
		 * ch[2] : 右摇杆上下
		 * ch[3] : 右摇杆左右
		 * */
		int16_t ch[10];
	} rc;
	struct
	{
		uint8_t SA;
		uint8_t SB;
		uint8_t SC;
		uint8_t SD;
		uint8_t SF;
		uint8_t SH;
	} toggle;
} remoter_t;

extern uint8_t remoter_buff[BUFF_REMOTER_SIZE];
extern remoter_t remoter;

extern void SBUS_Init(UART_HandleTypeDef *sbus_usart_handle);
extern void sbus_frame_parse(remoter_t *remoter, uint8_t *buf);

#endif // DM_BALANCEV1_HOME_SBUS_H
