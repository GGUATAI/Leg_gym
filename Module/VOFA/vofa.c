/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参）
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		2024/9/10		向上位机vofa+发送数据
 */
#include "vofa.h"
#include "bsp_usart.h"
#define MAX_BUFFER_SIZE 128
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;
float data_view[20]={0};
/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
	HAL_UART_Transmit(&huart7, buf, len, 0xff);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data)
{
	send_buf[cnt++] = byte0(data);
	send_buf[cnt++] = byte1(data);
	send_buf[cnt++] = byte2(data);
	send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void)
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;

	/* 将数据和帧尾打包发送 */
	vofa_transmit((uint8_t *)send_buf, cnt);
	cnt = 0;// 每次发送完帧尾都需要清零
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL
* @retval     void
* @details:   demo示例
***********************************************************************
**/
void vofa_demo(float data1,float data2,float data3,float data4)
{

	// Call the function to store the data in the buffer
	vofa_send_data(0, data1);
	vofa_send_data(1, data2);
	vofa_send_data(2, data3);
	vofa_send_data(3, data4);
	// Call the function to send the frame tail
	vofa_sendframetail();
}

/**
 * @brief vofa发送不动长数据
 * @param buf 数组指针
 * @param len 数组长度
 */

uint8_t buf_end[4] = {0x00, 0x00, 0x80, 0x7f};
void vofa_send_indefinite_len_data(float *buf, uint8_t len)
{
	cnt = len * 4 ;

	memcpy(send_buf , buf, cnt);
	memcpy(send_buf + cnt, buf_end, 4);
	vofa_transmit((uint8_t *)send_buf, cnt +4);

//	memset(send_buf , 0, cnt);
}







