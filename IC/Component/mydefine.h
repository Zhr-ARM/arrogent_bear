#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "main.h"
#include "monitor.h"
#include "usart.h"


extern uint8_t Recv3;
extern uint8_t Recv6;
extern float f_roll, f_pitch, f_yaw;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
