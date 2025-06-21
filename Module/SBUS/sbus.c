#include "sbus.h"
#include "bsp_usart.h"

uint8_t remoter_buff[BUFF_REMOTER_SIZE];
remoter_t remoter;

usart_t *sbus_usart_instance;

static void SBUS_Rx_Callback(void)
{
	memcpy(remoter_buff, sbus_usart_instance->recv_buff, BUFF_RC_RECEIVE_SIZE);
}

void SBUS_Init(UART_HandleTypeDef *sbus_usart_handle)
{
	usart_init_config_t config;
	config.module_callback = SBUS_Rx_Callback;
	config.usart_handle = sbus_usart_handle;
	config.recv_buff_size = sizeof(remoter_t) * 4;

	sbus_usart_instance = USART_Register(&config);
}

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
	USART_Error_Lost(sbus_usart_instance);

	if(sbus_usart_instance->lost_flag == 1)
	{
		memset(remoter,0,sizeof(remoter_t ));
		remoter->online = 0;
		return;
	}

    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
    {
        return;
    }

    if ((buf[23] == 0x0C) || (buf[23] == 0x0F))
    {
        remoter->online = 0;
    }
    else
    {
        remoter->online = 1;
    }

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF) - REMOTE_RC_OFFSET;
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF) - REMOTE_RC_OFFSET;
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF) - REMOTE_RC_OFFSET;
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF) - REMOTE_RC_OFFSET;
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

//    for (uint8_t i = 0; i < 4; i++)
//    {
//        if (remoter->rc.ch[i] < 5 || remoter->rc.ch[i] > -5)
//        {
//            remoter->rc.ch[i] = 0;
//        }
//    }

    switch (remoter->rc.ch[4])
    {
    case 0x161:
        remoter->toggle.SA = RC_SW_UP;
        break;
    case 0x0400:
        remoter->toggle.SA = RC_SW_MID;
        break;
    case 0x069F:
        remoter->toggle.SA = RC_SW_DOWN;
        break;
    }
    switch (remoter->rc.ch[5])
    {
    case 0x161:
        remoter->toggle.SB = RC_SW_UP;
        break;
    case 0x0400:
        remoter->toggle.SB = RC_SW_MID;
        break;
    case 0x069F:
        remoter->toggle.SB = RC_SW_DOWN;
        break;
    }
    switch (remoter->rc.ch[6])
    {
    case 0x161:
        remoter->toggle.SC = RC_SW_UP;
        break;
    case 0x0400:
        remoter->toggle.SC = RC_SW_MID;
        break;
    case 0x069F:
        remoter->toggle.SC = RC_SW_DOWN;
        break;
    }
    switch (remoter->rc.ch[7])
    {
    case 0x161:
        remoter->toggle.SD = RC_SW_UP;
        break;
    case 0x0400:
        remoter->toggle.SD = RC_SW_MID;
        break;
    case 0x069F:
        remoter->toggle.SD = RC_SW_DOWN;
        break;
    }
    switch (remoter->rc.ch[8])
    {
    case 0x161:
        remoter->toggle.SF = RC_SW_UP;
        break;
    case 0x069F:
        remoter->toggle.SF = RC_SW_DOWN;
        break;
    }
    switch (remoter->rc.ch[9])
    {
    case 0x161:
        remoter->toggle.SH = RC_SW_UP;
        break;
    case 0x069F:
        remoter->toggle.SH = RC_SW_DOWN;
        break;
    }

    //	remoter -> sbus_recever_time = HAL_GetTick ();
}
