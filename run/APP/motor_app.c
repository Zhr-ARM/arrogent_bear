#include "motor_app.h"

MotorDriver_t car;

void Motor_Init(void)
{
	MotorDriver_Init(&car);
}
