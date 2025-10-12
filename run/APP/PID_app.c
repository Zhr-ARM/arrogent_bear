#include "PID_app.h"
#include "path_planning.h"
#include <math.h>  // 添加数学库头文件，用于fabs函数

#define basic_speed 15
extern MotorDriver_t car;
extern float f_rho;

typedef enum {
    SPEED_LOW = 0,    // < 30
    SPEED_MID = 1,    // 30-50 
    SPEED_HIGH_L = 2,   // 60-80
		SPEED_HIGH_H = 3,   // 80-100
} SpeedRange_t;

PID_T pid_speed_A;  
PID_T pid_speed_B;  
PID_T pid_speed_C;  
PID_T pid_speed_D;  
PID_T pid_speed_rho; 
PID_T pid_speed_yaw; 

PidParams_t pid_params_A = {
    .kp = 5.0f,
    .ki = 0.69f,
    .kd = 0.8f,
    .out_min = -999.0f,
    .out_max = 80.0f,
};

// 不同速度段的PID参数
PidParams_t pid_params_A_speed[4] = {
    // 低速参数
    {.kp = 2.2f, .ki = 0.89f, .kd = 0.9f, .out_min = -999.0f, .out_max = 80.0f},
    // 中速参数
    {.kp = 5.0f, .ki = 0.69f, .kd = 0.8f, .out_min = -999.0f, .out_max = 80.0f},
    // 高速(L)参数
    {.kp = 6.0f, .ki = 0.69f, .kd = 0.8f, .out_min = -999.0f, .out_max = 80.0f},
		// 高速(H)参数
    {.kp = 11.0f, .ki = 0.69f, .kd = 0.8f, .out_min = -999.0f, .out_max = 80.0f}
};

PidParams_t pid_params_B = {
    .kp = 3.0f,
    .ki = 0.48f,
    .kd = 0.08f,
    .out_min = -80.0f,
    .out_max = 80.0f,
};

// 不同速度段的PID参数
PidParams_t pid_params_B_speed[4] = {
    // 低速参数
    {.kp = 2.2f, .ki = 0.89f, .kd = 0.9f, .out_min = -80.0f, .out_max = 80.0f},
    // 中速参数
    {.kp = 3.0f, .ki = 0.48f, .kd = 0.08f, .out_min = -999.0f, .out_max = 80.0f},
    // 高速(L)参数
    {.kp = 3.5f, .ki = 0.48f, .kd = 0.08f, .out_min = -999.0f, .out_max = 80.0f},
		// 高速(H)参数
    {.kp = 6.8f, .ki = 0.69f, .kd = 0.8f, .out_min = -80.0f, .out_max = 80.0f}
};

PidParams_t pid_params_C = {
    .kp = 4.5f,
    .ki = 0.93f,
    .kd = 2.2f,
    .out_min = -80.0f,
    .out_max = 80.0f,
};

// 不同速度段的PID参数
PidParams_t pid_params_C_speed[4] = {
    // 低速参数
    {.kp = 1.8f, .ki = 0.2f, .kd = 0.8f, .out_min = -80.0f, .out_max = 80.0f},
    // 中速参数
    {.kp = 4.5f, .ki = 0.2f, .kd = 0.2f, .out_min = -80.0f, .out_max = 80.0f},
    // 高速(L)参数
    {.kp = 3.5f, .ki = 0.93f, .kd = 2.2f, .out_min = -80.0f, .out_max = 80.0f},
		// 高速(H)参数
    {.kp = 5.6f, .ki = 0.66f, .kd = 0.2f, .out_min = -80.0f, .out_max = 80.0f}
};

PidParams_t pid_params_D = {
    .kp = 4.58f,    // 适度增加P参数，提高控制力
    .ki = 0.96f,    // 增加I参数，消除稳态误差
    .kd = 1.2f,    // 保持D参数，维持稳定性
    .out_min = -70.0f,
    .out_max = 70.0f,
};

// 不同速度段的PID参数
PidParams_t pid_params_D_speed[4] = {
    // 低速参数
    {.kp = 1.2f, .ki = 0.2f, .kd = 0.2f, .out_min = -70.0f, .out_max = 70.0f},
    // 中速参数
    {.kp = 3.28f, .ki = 1.6f, .kd = 3.9f, .out_min = -999.0f, .out_max = 90.0f},
    // 高速(L)参数
    {.kp = 4.58f, .ki = 0.66f, .kd = 0.6f, .out_min = -70.0f, .out_max = 70.0f},
		// 高速(H)参数
    {.kp = 99.0f, .ki = 0.96f, .kd = 3.0f, .out_min = -999.0f, .out_max = 120.0f}
};

PidParams_t pid_params_rho = {
    .kp = 1.2f,
    .ki = 0.02f,
    .kd = 0.3f,
    .out_min = -25.0f,
    .out_max = 25.0f,
};

PidParams_t pid_params_yaw = {//!!!!!!!!!!!!!!!!!!!!!
    .kp = 0.25f,    // 进一步减小比例系数，提高稳定性
    .ki = 0.005f,   // 进一步减小积分项，减少积分饱和
    .kd = 0.15f,    // 适当增加微分项，提高阻尼
    .out_min = -30.0f,   // 进一步限制输出范围
    .out_max = 30.0f,
};

SpeedRange_t GetSpeedRange(float speed) {
    if (speed < 30.0f) return SPEED_LOW;
    else if (30.0f <= speed && speed < 59.0f) return SPEED_MID;
		else if (59.0f <= speed && speed < 80.0f) return SPEED_HIGH_L;
    else return SPEED_HIGH_H;
}

void PID_Init(void)
{
  pid_init(&pid_speed_A,
           pid_params_A.kp, pid_params_A.ki, pid_params_A.kd,
           0.0f, pid_params_A.out_max);
  
  pid_init(&pid_speed_B,
           pid_params_B.kp, pid_params_B.ki, pid_params_B.kd,
           0.0f, pid_params_B.out_max);
	
  pid_init(&pid_speed_C,
           pid_params_C.kp, pid_params_C.ki, pid_params_C.kd,
           0.0f, pid_params_C.out_max);
  
  pid_init(&pid_speed_D,
           pid_params_D.kp, pid_params_D.ki, pid_params_D.kd,
           0.0f, pid_params_D.out_max);	
	
	pid_init(&pid_speed_rho,
           pid_params_rho.kp, pid_params_rho.ki, pid_params_rho.kd,
           0.0f, pid_params_rho.out_max);	
	
	pid_init(&pid_speed_yaw,
           pid_params_yaw.kp, pid_params_yaw.ki, pid_params_yaw.kd,
           0.0f, pid_params_yaw.out_max);	

  
  pid_set_target(&pid_speed_A, basic_speed);
  pid_set_target(&pid_speed_B, basic_speed);
	pid_set_target(&pid_speed_C, basic_speed);
  pid_set_target(&pid_speed_D, basic_speed);
  pid_set_target(&pid_speed_rho, 76);
	pid_set_target(&pid_speed_yaw, 0);
  

}
void VisionControl_Update(float f_rho) 
{
	int PID_vision_output = 0;
    
	// 视觉PID计算：目标=0（轨道中心），反馈=rho
	PID_vision_output = pid_calculate_positional(&pid_speed_rho, f_rho);
	
	// 限制速度范围
	PID_vision_output = pid_constrain(PID_vision_output, pid_params_rho.out_min, pid_params_rho.out_max);

	// 设置到PID系统
	pid_update_target(&pid_speed_A, basic_speed+PID_vision_output);   
	pid_update_target(&pid_speed_B, basic_speed-PID_vision_output);  
	pid_update_target(&pid_speed_C, basic_speed-PID_vision_output);   
	pid_update_target(&pid_speed_D, basic_speed+PID_vision_output);  

}


void YAWControl_Update(TurnType_t turn_type) 
{
    switch (turn_type) {
        case TURN_NONE:
            // 无转向，停止所有电机
            MotorDriver_StopAll(&car);
            my_printf(&huart4,"[DEBUG] YAW: No turn, stopping\n");
            break;
            
        case TURN_LEFT:
            // 左转90度
            MotorDriver_Turn(&car, basic_speed, -100);
            my_printf(&huart4,"[DEBUG] YAW: Turning left\n");
            break;
            
        case TURN_RIGHT:
            // 右转90度
            MotorDriver_Turn(&car, basic_speed, +100); 
            my_printf(&huart4,"[DEBUG] YAW: Turning right\n");
            break;
            
        case TURN_U_TURN:
            // 掉头（右转180度）
            MotorDriver_U_Turn(&car, 30, 1);  // 1表示右转180度
            my_printf(&huart4,"[DEBUG] YAW: U-turn started\n");
            break;
            
        default:
            // 未知转向类型，停止
            MotorDriver_StopAll(&car);
            my_printf(&huart4,"[DEBUG] YAW: Unknown turn type, stopping\n");
            break;
    }
}

// 自适应PID更新
void AdaptivePID_Update_A(void) {
    static SpeedRange_t last_range = SPEED_MID;
    
    // 获取当前速度
    SpeedRange_t current_range = GetSpeedRange(pid_speed_A.target);
    
    // 如果速度段改变，更新PID参数
    if (current_range != last_range) {
        pid_set_params(&pid_speed_A,
                       pid_params_A_speed[current_range].kp,
                       pid_params_A_speed[current_range].ki,
                       pid_params_A_speed[current_range].kd);
        pid_set_limit(&pid_speed_A, 
                      pid_params_A_speed[current_range].out_max);
        last_range = current_range;
    }
}

// 自适应PID更新
void AdaptivePID_Update_B(void) {
    static SpeedRange_t last_range = SPEED_MID;
    
    // 获取当前速度
    SpeedRange_t current_range = GetSpeedRange(pid_speed_B.target);
    
    // 如果速度段改变，更新PID参数
    if (current_range != last_range) {
        pid_set_params(&pid_speed_B,
                       pid_params_B_speed[current_range].kp,
                       pid_params_B_speed[current_range].ki,
                       pid_params_B_speed[current_range].kd);
        pid_set_limit(&pid_speed_B, 
                      pid_params_B_speed[current_range].out_max);
        last_range = current_range;
    }
}

// 自适应PID更新
void AdaptivePID_Update_C(void) {
    static SpeedRange_t last_range = SPEED_MID;
    
    // 获取当前速度
    SpeedRange_t current_range = GetSpeedRange(pid_speed_C.target);
    
    // 如果速度段改变，更新PID参数
    if (current_range != last_range) {
        pid_set_params(&pid_speed_C,
                       pid_params_C_speed[current_range].kp,
                       pid_params_C_speed[current_range].ki,
                       pid_params_C_speed[current_range].kd);
        pid_set_limit(&pid_speed_C, 
                      pid_params_C_speed[current_range].out_max);
        last_range = current_range;
    }
}

// 自适应PID更新
void AdaptivePID_Update_D(void) {
    static SpeedRange_t last_range = SPEED_MID;
    
    // 获取当前速度
    SpeedRange_t current_range = GetSpeedRange(pid_speed_D.target);
    
    // 如果速度段改变，更新PID参数
    if (current_range != last_range) {
        pid_set_params(&pid_speed_D,
                       pid_params_D_speed[current_range].kp,
                       pid_params_D_speed[current_range].ki,
                       pid_params_D_speed[current_range].kd);
        pid_set_limit(&pid_speed_D, 
                      pid_params_D_speed[current_range].out_max);
        last_range = current_range;
    }
}

uint8_t pid_running = 1; 
extern volatile uint8_t system_started;  // 外部系统启动标志

void PID_Task(void)
{
    if(!pid_running || !system_started) return;

    float output_A, output_B,output_C,output_D;
	
//		VisionControl_Update(f_rho);
//		YAWControl_Update(f_yaw , -180);
		
		AdaptivePID_Update_A();
		AdaptivePID_Update_B();
		AdaptivePID_Update_C();
		AdaptivePID_Update_D();
	
    output_A 	 = pid_calculate_positional(&pid_speed_A, A_encoder.speed_cm_s);
    output_B 	 = pid_calculate_positional(&pid_speed_B, B_encoder.speed_cm_s);
		output_C 	 = pid_calculate_positional(&pid_speed_C, C_encoder.speed_cm_s);
    output_D   = pid_calculate_positional(&pid_speed_D, D_encoder.speed_cm_s);


    output_A 	 = pid_constrain(output_A, pid_params_A.out_min, pid_params_A.out_max);
    output_B 	 = pid_constrain(output_B, pid_params_B.out_min, pid_params_B.out_max);
		output_C	 = pid_constrain(output_C, pid_params_C.out_min, pid_params_C.out_max);
		output_D 	 = pid_constrain(output_D, pid_params_D.out_min, pid_params_D.out_max);
	
    MotorDriver_SetMotor(&car,MOTOR_A,output_A);
		MotorDriver_SetMotor(&car,MOTOR_B,output_B);
		MotorDriver_SetMotor(&car,MOTOR_C,output_C);
		MotorDriver_SetMotor(&car,MOTOR_D,output_D);

}

