#include "bsp_can.h"
#include "dm_motor.h"

#include "Chassis_Leg_Task.h"

void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 扩展ID不接收
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000; //
    sFilterConfig.FilterID2 = 0x00000000; //
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 全局过滤设置 */
    /* 接收到消息ID与标准ID过滤不匹配，不接受 */
    /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
    /* 过滤标准ID远程帧 */
    /* 过滤扩展ID远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    /* 开启RX FIFO0的新数据中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

//    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1); // 开启FDCAN的发送延迟补偿

//    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 14, 14); // 设置补偿时间 参数2和参数3都为TimeSeg1的值

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 扩展ID不接收
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000; //
    sFilterConfig.FilterID2 = 0x00000000; //
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 全局过滤设置 */
    /* 接收到消息ID与标准ID过滤不匹配，不接受 */
    /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
    /* 过滤标准ID远程帧 */
    /* 过滤扩展ID远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    /* 开启RX FIFO0的新数据中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

//    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1); // 开启FDCAN的发送延迟补偿

//    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 14, 14); // 设置补偿时间 参数2和参数3都为TimeSeg1的值

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 扩展ID不接收
    sFilterConfig.FilterIndex = 3;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000; //
    sFilterConfig.FilterID2 = 0x00000000; //
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 全局过滤设置 */
    /* 接收到消息ID与标准ID过滤不匹配，不接受 */
    /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
    /* 过滤标准ID远程帧 */
    /* 过滤扩展ID远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    /* 开启RX FIFO0的新数据中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度     ！！达妙例程使用len << 16，移植后使用出问题！！
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
    TxHeader.DataLength = len;                        // len << 16			！！！！！！					// 发送数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
    TxHeader.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
    {
        return 1;
    } // 发送
    return 0;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static FDCAN_RxHeaderTypeDef RxHeader1;
    static uint8_t g_Can1RxData[64];
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retrieve Rx messages from RX FIFO0 */
        memset(g_Can1RxData, 0, sizeof(g_Can1RxData)); // 接收前先清空数组
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
        if (hfdcan->Instance == FDCAN3)
        {
            switch (RxHeader1.Identifier)
            {
            case 0x201:
                DJI3508_Decode(&chassis_move.wheel_motor[0], g_Can1RxData, RxHeader1.DataLength);
                break;
            case 0x202:
                DJI3508_Decode(&chassis_move.wheel_motor[1], g_Can1RxData, RxHeader1.DataLength);
                break;
            default:
                break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    static FDCAN_RxHeaderTypeDef RxHeader2;
    static uint8_t g_Can2RxData[64];
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);

            switch (RxHeader2.Identifier)
            {
            case 0x10:
                DM8009p_Decode(&chassis_move.joint_motor[0], g_Can2RxData, RxHeader2.DataLength);
                break;
            case 0x11:
                DM8009p_Decode(&chassis_move.joint_motor[1], g_Can2RxData, RxHeader2.DataLength);
                break;
            case 0x12:
                DM8009p_Decode(&chassis_move.joint_motor[2], g_Can2RxData, RxHeader2.DataLength);
                break;
            case 0x13:
                DM8009p_Decode(&chassis_move.joint_motor[3], g_Can2RxData, RxHeader2.DataLength);
                break;
            case 0x14:
                DM4310_Decode(&chassis_move.joint_motor[4], g_Can2RxData, RxHeader2.DataLength);
                break;
            case 0x15:
                DM4310_Decode(&chassis_move.joint_motor[5], g_Can2RxData, RxHeader2.DataLength);
                break;
            default:
                break;
            }
        }
    }
}
