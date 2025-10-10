#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdint.h"
#include "stdlib.h"
#include <stdbool.h>

#include "usart.h"
#include "main.h"

#include "monitor.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "PID.h"
#include "path_planning.h"

#include "motor_app.h"
#include "encoder_app.h"
#include "PID_app.h"

extern uint8_t Recv3;
extern uint8_t Recv5;
extern float f_roll,f_pitch,f_yaw;
extern float f_rho;
extern uint8_t f_cross;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

extern TIM_HandleTypeDef htim1;		// ��������ʱ��(A)
extern TIM_HandleTypeDef htim2;		// ��������ʱ��(B)
extern TIM_HandleTypeDef htim3;		// ��������ʱ��(C)
extern TIM_HandleTypeDef htim4;		// ��������ʱ��(D)
extern TIM_HandleTypeDef htim10;  // PWM��ʱ��(A)
extern TIM_HandleTypeDef htim11;  // PWM��ʱ��(B)
extern TIM_HandleTypeDef htim13;  // PWM��ʱ��(C)
extern TIM_HandleTypeDef htim14;  // PWM��ʱ��(D)


