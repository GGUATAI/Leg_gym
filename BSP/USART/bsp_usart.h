#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"

#define DEVICE_USART_CNT 4     // DM_H7至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
	USART_TRANSFER_NONE=0,
	USART_TRANSFER_BLOCKING,
	USART_TRANSFER_IT,
	USART_TRANSFER_DMA,
} usart_transfer_e;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
	uint8_t current_size;
	uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
	uint16_t recv_buff_size;                // 模块接收一包数据的大小
	UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
	usart_module_callback module_callback; // 解析收到的数据的回调函数
	uint16_t beat;
	uint8_t lost_flag;
} usart_t;

/* usart 初始化配置结构体 */
typedef struct
{
	uint16_t recv_buff_size;                // 模块接收一包数据的大小
	UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
	usart_module_callback module_callback; // 解析收到的数据的回调函数
} usart_init_config_t;

usart_t *USART_Register(usart_init_config_t *init_config);

void USART_Service_Init(usart_t *_instance);

void USART_Send(usart_t *_instance, uint8_t *send_buf, uint16_t send_size,usart_transfer_e mode);

uint8_t USART_Error_Lost(usart_t *_instance);

uint8_t USART_Is_Ready(usart_t *_instance);
