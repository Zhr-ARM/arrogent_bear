#ifndef __PID_APP_H__
#define __PID_APP_H__

#include "mydefine.h"

// PID�����ṹ��
typedef struct
{
    float kp;          // ����ϵ��
    float ki;          // ����ϵ��
    float kd;          // ΢��ϵ��
    float out_min;     // �����Сֵ
    float out_max;     // ������ֵ
} PidParams_t;

void PID_Init(void);
void PID_Task(void);
void VisionControl_Update(float f_rho);
void AdaptivePID_Update_A(void);
void AdaptivePID_Update_B(void);
void AdaptivePID_Update_C(void);
void AdaptivePID_Update_D(void);
// 转向类型枚举
typedef enum {
    TURN_NONE = 0,    // 无转向
    TURN_LEFT = 1,    // 左转
    TURN_RIGHT = 2,   // 右转
    TURN_U_TURN = 3   // 掉头
} TurnType_t;

void YAWControl_Update(TurnType_t turn_type);

// 外部变量声明
extern uint8_t f_cross;  // 十字路口检测值
extern float f_yaw;      // 陀螺仪航向角

#endif
