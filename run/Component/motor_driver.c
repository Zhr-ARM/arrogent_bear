#include "motor_driver.h"
#include <stdlib.h>

/* ==================== �ڲ��������� ==================== */

/**
 * @brief ������ֵ��Χ
 */
static int8_t clamp(int8_t value, int8_t min, int8_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief ˫H�ŷ������
 * @note IN1  IN2  ����
 *       0    0    ֹͣ(�ƶ�)
 *       0    1    ��ת
 *       1    0    ��ת
 *       1    1    ֹͣ(�ƶ�)
 */
static void set_motor_direction(Motor_t* motor, MotorDir_t dir) {
    switch (dir) {
        case MOTOR_DIR_STOP:
            HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_DIR_BACKWARD:
            HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_SET);
            break;
    }
    motor->direction = dir;
}

/**
 * @brief ����PWMռ�ձ�
 */
static void set_motor_pwm(Motor_t* motor, uint16_t duty) {
    __HAL_TIM_SET_COMPARE(motor->hw.pwm_tim, motor->hw.pwm_channel, duty);
}

/**
 * @brief Ӧ�õ������
 */
static void apply_motor_control(Motor_t* motor, int8_t speed) {
    if (!motor || !motor->enable) return;
    
    // ���Ƿ����־
    int8_t actual_speed = motor->hw.reverse ? -speed : speed;
    motor->speed = speed;
    
    if (speed == 0) {
        // ֹͣ
        set_motor_direction(motor, MOTOR_DIR_STOP);
        set_motor_pwm(motor, 0);
    }
    else if (actual_speed > 0) {
        // ��ת
        set_motor_direction(motor, MOTOR_DIR_FORWARD);
        set_motor_pwm(motor, (uint16_t)(actual_speed * MOTOR_PWM_MAX / 100));
    }
    else {
        // ��ת
        set_motor_direction(motor, MOTOR_DIR_BACKWARD);
        set_motor_pwm(motor, (uint16_t)((-actual_speed) * MOTOR_PWM_MAX / 100));
    }
}

/* ==================== ����APIʵ�� ==================== */

int8_t MotorDriver_Init(MotorDriver_t* driver) {
    if (!driver) return -1;
    
    // Ӳ�����ñ�
    const struct {
        GPIO_TypeDef* in1_port;
        uint16_t in1_pin;
        GPIO_TypeDef* in2_port;
        uint16_t in2_pin;
        TIM_HandleTypeDef* pwm_tim;
        uint32_t pwm_channel;
        uint8_t reverse;
    } hw_cfg[MOTOR_NUM] = {
        {MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
         &MOTOR_A_PWM_TIM, MOTOR_A_PWM_CHANNEL,MOTOR_A_REVERSE},
        
        {MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
         &MOTOR_B_PWM_TIM, MOTOR_B_PWM_CHANNEL,MOTOR_B_REVERSE},
        
        {MOTOR_C_IN1_PORT, MOTOR_C_IN1_PIN, MOTOR_C_IN2_PORT, MOTOR_C_IN2_PIN,
         &MOTOR_C_PWM_TIM, MOTOR_C_PWM_CHANNEL,MOTOR_C_REVERSE},
        
        {MOTOR_D_IN1_PORT, MOTOR_D_IN1_PIN, MOTOR_D_IN2_PORT, MOTOR_D_IN2_PIN,
         &MOTOR_D_PWM_TIM, MOTOR_D_PWM_CHANNEL,MOTOR_D_REVERSE}
    };
    
    // ��ʼ���ĸ����
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        Motor_t* m = &driver->motors[i];
        
        // Ӳ������
        m->hw.in1_port = hw_cfg[i].in1_port;
        m->hw.in1_pin = hw_cfg[i].in1_pin;
        m->hw.in2_port = hw_cfg[i].in2_port;
        m->hw.in2_pin = hw_cfg[i].in2_pin;
        m->hw.pwm_tim = hw_cfg[i].pwm_tim;
        m->hw.pwm_channel = hw_cfg[i].pwm_channel;
        m->hw.reverse = hw_cfg[i].reverse;
        
        // ״̬��ʼ��
        m->speed = 0;
        m->direction = MOTOR_DIR_STOP;
        m->enable = 1;
        
        // ����PWM
        HAL_TIM_PWM_Start(m->hw.pwm_tim, m->hw.pwm_channel);
        
        // ��ʼ��Ϊֹͣ
        apply_motor_control(m, 0);
    }
    
    driver->initialized = 1;
    return 0;
}

int8_t MotorDriver_SetMotor(MotorDriver_t* driver, MotorID_t motor_id, int8_t speed) {
    if (!driver || !driver->initialized || motor_id >= MOTOR_NUM) {
        return -1;
    }
    
    speed = clamp(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    apply_motor_control(&driver->motors[motor_id], speed);
    
    return 0;
}

void MotorDriver_StopAll(MotorDriver_t* driver) {
    if (!driver) return;
    
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        apply_motor_control(&driver->motors[i], 0);
    }
}

int8_t MotorDriver_Move(MotorDriver_t* driver, int8_t speed) {
    if (!driver || !driver->initialized) return -1;
    
    speed = clamp(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        apply_motor_control(&driver->motors[i], speed);
    }
    
    return 0;
}

int8_t MotorDriver_Turn(MotorDriver_t* driver, int8_t speed, int8_t turn_rate) {
    if (!driver || !driver->initialized) return -1;
    
    speed = clamp(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
    turn_rate = clamp(turn_rate, -100, 100);
    
    int8_t left_speed, right_speed;
    
    /* ����ת���㷨 */
    if (turn_rate == 0) {
        left_speed = right_speed = speed;
    }
    else if (turn_rate > 0) {       
			 // 左转
        left_speed = speed;
        right_speed = -10;
    }
    else {
       // 右转
        right_speed = speed;
        left_speed = -10;
    }
    
    // �����(B��C)
    apply_motor_control(&driver->motors[MOTOR_B], left_speed);
    apply_motor_control(&driver->motors[MOTOR_C], left_speed);
    
    // �Ҳ���(A��D)
    apply_motor_control(&driver->motors[MOTOR_A], right_speed);
    apply_motor_control(&driver->motors[MOTOR_D], right_speed);
    
    return 0;
}

int8_t MotorDriver_Rotate(MotorDriver_t* driver, int8_t speed, uint8_t direction) {
    if (!driver || !driver->initialized) return -1;
    
    speed = clamp(speed, 0, 100);
    
    if (direction == 0) {
        // ����
        apply_motor_control(&driver->motors[MOTOR_B], -speed);
        apply_motor_control(&driver->motors[MOTOR_C], -speed);
        apply_motor_control(&driver->motors[MOTOR_A], speed);
        apply_motor_control(&driver->motors[MOTOR_D], speed);
    }
    else {
        // ����
        apply_motor_control(&driver->motors[MOTOR_B], speed);
        apply_motor_control(&driver->motors[MOTOR_C], speed);
        apply_motor_control(&driver->motors[MOTOR_A], -speed);
        apply_motor_control(&driver->motors[MOTOR_D], -speed);
    }
    
    return 0;
}

/**
 * @brief 原地掉头转向
 * @param driver 驱动器指针
 * @param speed 转向速度 (0~100)
 * @param direction 转向方向 (0:左转180度 1:右转180度)
 * @return 0:成功 -1:失败
 */
int8_t MotorDriver_U_Turn(MotorDriver_t* driver, int8_t speed, uint8_t direction) {
    if (!driver || !driver->initialized) return -1;
    
    speed = clamp(speed, 0, 100);
    
    if (direction == 0) {
        // 左转180度：左轮后退，右轮前进
        apply_motor_control(&driver->motors[MOTOR_B], -speed);  // 左前轮后退
        apply_motor_control(&driver->motors[MOTOR_C], -speed);  // 左后轮后退
        apply_motor_control(&driver->motors[MOTOR_A], speed);   // 右前轮前进
        apply_motor_control(&driver->motors[MOTOR_D], speed);   // 右后轮前进
    }
    else {
        // 右转180度：左轮前进，右轮后退
        apply_motor_control(&driver->motors[MOTOR_B], speed);   // 左前轮前进
        apply_motor_control(&driver->motors[MOTOR_C], speed);   // 左后轮前进
        apply_motor_control(&driver->motors[MOTOR_A], -speed);  // 右前轮后退
        apply_motor_control(&driver->motors[MOTOR_D], -speed);  // 右后轮后退
    }
    
    return 0;
}



