#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "mydefine.h"
#include <stdint.h>

/* ���ID */
typedef enum {
    MOTOR_A = 0,    // ��ǰ
    MOTOR_B = 1,    // ��ǰ
    MOTOR_C = 2,    // ���
    MOTOR_D = 3     // �Һ�
} MotorID_t;

/* �����ת���� */
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD
} MotorDir_t;

/* ���Ӳ������ */
typedef struct {
    // �����������
    GPIO_TypeDef* in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t in2_pin;
    
    // PWM
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_channel;
    
    // �����־
    uint8_t reverse;
} MotorHW_t;

/* �������ʵ�� */
typedef struct {
    MotorHW_t hw;								//Ӳ������
    int8_t speed;               // -100 ~ 100
    MotorDir_t direction;				//��ת����
    uint8_t enable;
} Motor_t;

/* ���������� */
typedef struct {
    Motor_t motors[MOTOR_NUM];
    uint8_t initialized;
} MotorDriver_t;

/* ==================== ��ʼ�� ==================== */

/**
 * @brief ��ʼ����·�������
 * @param driver ������ָ��
 * @return 0:�ɹ� -1:ʧ��
 */
int8_t MotorDriver_Init(MotorDriver_t* driver);

/* ==================== �������� ==================== */

/**
 * @brief ���õ�������ٶ�
 * @param driver ������ָ��
 * @param motor_id ���ID (MOTOR_A~MOTOR_D)
 * @param speed �ٶ� (-100~100)
 */
int8_t MotorDriver_SetMotor(MotorDriver_t* driver, MotorID_t motor_id, int8_t speed);

/**
 * @brief ֹͣ���е��
 */
void MotorDriver_StopAll(MotorDriver_t* driver);

/* ==================== �����˶� ==================== */

/**
 * @brief ǰ��/����
 * @param speed �ٶ� (-100~100)
 */
int8_t MotorDriver_Move(MotorDriver_t* driver, int8_t speed);

/**
 * @brief ����ת��
 * @param speed ǰ���ٶ� (-100~100)
 * @param turn_rate ת���� (-100~100)
 *        -100: ����ת  0: ֱ��  +100: ����ת
 */
int8_t MotorDriver_Turn(MotorDriver_t* driver, int8_t speed, int8_t turn_rate);

/**
 * @brief ԭ����ת
 * @param speed ��ת�ٶ� (0~100)
 * @param direction 0:���� 1:����
 */
int8_t MotorDriver_Rotate(MotorDriver_t* driver, int8_t speed, uint8_t direction);

/**
 * @brief 原地掉头转向
 * @param driver 驱动器指针
 * @param speed 转向速度 (0~100)
 * @param direction 转向方向 (0:左转180度 1:右转180度)
 * @return 0:成功 -1:失败
 */
int8_t MotorDriver_U_Turn(MotorDriver_t* driver, int8_t speed, uint8_t direction);


#endif
