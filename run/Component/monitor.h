#ifndef __MONITOR_H_
#define __MONITOR_H_

#include "stdint.h"  // ��׼���Ͷ���
#include "mydefine.h"

int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
void Uart3_Receive(void);
void Uart3_Parse(void);
void RHO_Receive(void);
void RHO_Parse(void);
void JY61P_Send(uint8_t type);

// 卡尔曼滤波相关函数
void Kalman_Init(void);
float Kalman_Filter_Yaw(float measurement);
#endif 
