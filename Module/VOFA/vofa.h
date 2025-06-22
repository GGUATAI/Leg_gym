#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "main.h"
#include "cmsis_os.h"

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))

extern float data_view[20];

void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
extern void vofa_demo(float data1,float data2,float data3,float data4);
extern void vofa_send_indefinite_len_data(float *buf, uint8_t len);
#endif // DEBUG
