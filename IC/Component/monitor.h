#ifndef __MONITOR_H_
#define __MONITOR_H_

#include "mydefine.h"
#include "stdint.h" // 标准整型定义

// 外部变量声明
extern uint8_t Recv_buf6[100];
extern uint8_t Recv_index6;
extern uint8_t Recv_flag6;

int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
void Uart3_Receive(void);
void Uart3_Parse(void);
void Uart6_Receive(void);
void Uart6_Parse(void);

#endif
