#ifndef CRC_H
#define CRC_H

#include "stm32h7xx.h"
#include "main.h"
#include "cmsis_os.h"

// 注：此为裁判系统所用CRC，与USB-CDC所用算法不同
extern uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage,
                                  uint32_t dwLength,
                                  uint8_t ucCRC8);
extern void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,
                                    uint32_t dwLength,
                                    uint16_t wCRC);
extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage,
                                       uint32_t dwLength);
extern void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif // DM_BALANCEV1_HOME_CRC_H
