#include "GPRMC.h"

#include "bsp_usart.h"

volatile uint16_t varl = 0;
volatile uint16_t var_Exp = 0;
volatile uint16_t global_time;
char snum[7];
volatile uint16_t shorttt = 0;

char gprmcStr[7] = "$GPRMC,";
int chckNum = 0;
char chckNumChar[2];

int ss = 0;
int mm = 0;
int hh = 0;

unsigned char result;
int i;

char value_1[100]="";
char value_2[100]="";
char value_time[10]="";

char test[100]="$GPRMC,004015,A,2812.0498,N,11313.1361,E,0.0,180.0,150122,3.9,W,A*";
char test1[100] = "No '*' found in the string.\n";

usart_t * GPRMC_usart_instance;

static void GPRMC_Rx_Callback(void)
{
    ;
}

void GPRMC_Init(UART_HandleTypeDef *GPRMC_usart_handle)
{
    usart_init_config_t config;
    config.module_callback = GPRMC_Rx_Callback;
    config.usart_handle = GPRMC_usart_handle;
    config.recv_buff_size = 255;

    GPRMC_usart_instance = USART_Register(&config); 
}

int checkNum(const char *gprmcContext)
{
    if (gprmcContext == NULL)
    {
        return -1;
    }

    result = gprmcContext[1];

    for (i = 2; gprmcContext[i] != '*' && gprmcContext[i] != '\0'; i++)
    {
        result ^= gprmcContext[i];
    }

    if (gprmcContext[i] != '*')
    {
        USART_Send(GPRMC_usart_instance, (uint8_t *)test1, strlen(test1),USART_TRANSFER_DMA);
        return -1;
    }

    // printf("Final result before returning: %02X\n", result);
    return result;
}

void GPRMC_Tx_Test(void)
{
    if (ss < 59)
    {
        ss++;
    }
    else
    {
        ss = 0;
        if (mm < 59)
        {
            mm++;
        }
        else
        {
            mm = 0;
            if (hh < 23)
            {
                hh++;
            }
            else
            {
                hh = 0;
            }
        }
    }

    sprintf(value_2, "%s%02d%02d%02d%s", gprmcStr, hh, mm, ss, ".00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*");
    strcpy(value_1, value_2);
    chckNum = checkNum(value_1);
    sprintf(chckNumChar, "%02X", chckNum);
	sprintf(value_2,"%s%s",value_2,chckNumChar);

    USART_Send(GPRMC_usart_instance, (uint8_t *)value_2, strlen(value_2),USART_TRANSFER_DMA);
//    USART_Send(GPRMC_usart_instance, (uint8_t *)chckNumChar, strlen(chckNumChar),USART_TRANSFER_DMA);
}

