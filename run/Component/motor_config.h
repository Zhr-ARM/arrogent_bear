#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "mydefine.h"

/* ������� */
#define MOTOR_NUM               4

/* �ٶȷ�Χ */
#define MOTOR_SPEED_MIN         -100
#define MOTOR_SPEED_MAX         100

/* PWM���� */
#define MOTOR_PWM_MAX           1000        // ARRֵ
#define MOTOR_PWM_FREQ          16800        // 16.8kHz

/* ==================== ���A (��ǰ) ==================== */
// �����������
#define MOTOR_A_IN1_PORT        GPIOE
#define MOTOR_A_IN1_PIN         GPIO_PIN_0
#define MOTOR_A_IN2_PORT        GPIOE
#define MOTOR_A_IN2_PIN         GPIO_PIN_1

// PWM����
#define MOTOR_A_PWM_TIM         htim10
#define MOTOR_A_PWM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_A_REVERSE         0           

/* ==================== ���B (��ǰ) ==================== */
#define MOTOR_B_IN1_PORT        GPIOC
#define MOTOR_B_IN1_PIN         GPIO_PIN_14
#define MOTOR_B_IN2_PORT        GPIOC
#define MOTOR_B_IN2_PIN         GPIO_PIN_15

#define MOTOR_B_PWM_TIM         htim11
#define MOTOR_B_PWM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_B_REVERSE         1

/* ==================== ���C (���) ==================== */
#define MOTOR_C_IN1_PORT        GPIOF
#define MOTOR_C_IN1_PIN         GPIO_PIN_0
#define MOTOR_C_IN2_PORT        GPIOF
#define MOTOR_C_IN2_PIN         GPIO_PIN_1

#define MOTOR_C_PWM_TIM         htim13
#define MOTOR_C_PWM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_C_REVERSE         0

/* ==================== ���D (�Һ�) ==================== */
#define MOTOR_D_IN1_PORT        GPIOF
#define MOTOR_D_IN1_PIN         GPIO_PIN_2
#define MOTOR_D_IN2_PORT        GPIOF
#define MOTOR_D_IN2_PIN         GPIO_PIN_3

#define MOTOR_D_PWM_TIM         htim14
#define MOTOR_D_PWM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_D_REVERSE         1

#endif /* MOTOR_CONFIG_H */


