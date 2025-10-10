#include "monitor.h"

#include "stdio.h"
#include "stdarg.h"

float f_roll,f_pitch,f_yaw;
float f_rho;
uint8_t f_cross;
	
// 卡尔曼滤波器结构体
typedef struct {
    float q;        // 过程噪声协方差
    float r;        // 测量噪声协方差
    float x;        // 状态估计值
    float p;        // 估计误差协方差
    float k;        // 卡尔曼增益
} KalmanFilter_t;

static KalmanFilter_t kalman_yaw;

uint8_t Recv3;
uint8_t Recv_buf3[100];
uint8_t Recv_index3 = 0; 
uint8_t Recv_flag3 = 0;

uint8_t Recv5;
uint8_t Recv_buf5[100];
uint8_t Recv_index5 = 0; 
uint8_t Recv_flag5 = 0;

int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512]; // ��ʱ�洢��ʽ������ַ���
	va_list arg;      // �����ɱ����
	int len;          // �����ַ�������

	va_start(arg, format);
	// ��ȫ�ظ�ʽ���ַ����� buffer
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	// ͨ�� HAL �ⷢ�� buffer �е�����
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}


void Uart3_Receive(void)
{
    if(Recv_flag3)
        return;
    // ֡ͷ���
    if(Recv_index3 == 0 && Recv3 != 0x55)
        return;
    if(Recv_index3 == 1 && Recv3 != 0x53) {
        Recv_index3 = 0; // ֡ͷ�ڶ��ֽڲ��ԣ����µȴ�֡ͷ
        return;
    }
    Recv_buf3[Recv_index3++] = Recv3;
    // ֡β���
    if(Recv_index3 >= 2 && Recv_buf3[Recv_index3-2] == 0xFB && Recv_buf3[Recv_index3-1] == 0x46)
        Recv_flag3 = 1;
    // ��ֹԽ��
    if(Recv_index3 >= sizeof(Recv_buf3)) {
        Recv_index3 = 0;
        Recv_flag3 = 0;
    }
}

void Uart3_Parse(void)
{
    if(Recv_flag3)
    {
        // ���������Ƕ����ݣ�����Э��������ǰ��һ�£�
        // ֡��ʽ��0x55 0x53 RollL RollH PitchL PitchH YawL YawH ... 0xFB 0x46
        if(Recv_index3 >= 10) // ����Ҫ��ͷ�����ݡ�β
        {
          int16_t roll = (int16_t)((Recv_buf3[3]<<8) | Recv_buf3[2]);
          int16_t pitch = (int16_t)((Recv_buf3[5]<<8) | Recv_buf3[4]);
          int16_t yaw = (int16_t)((Recv_buf3[7]<<8) | Recv_buf3[6]);
          f_roll = roll / 32768.0f * 180.0f;
          f_pitch = pitch / 32768.0f * 180.0f;
          f_yaw = yaw / 32768.0f * 180.0f;
          f_yaw = Kalman_Filter_Yaw(f_yaw); // 对yaw值进行卡尔曼滤波
        
        }
        Recv_flag3 = 0;
        Recv_index3 = 0;
    }
}

void RHO_Receive(void)
{
    if(Recv_flag5)
        return;
    // ֡ͷ���
    if(Recv_index5 == 0 && Recv5 != 0xfd)
        return;
    Recv_buf5[Recv_index5++] = Recv5;
		
    // ֡β���
    if(Recv_index5 >= 2 &&Recv_buf5[Recv_index5-1] == 0xfe)
        Recv_flag5 = 1;
    // ��ֹԽ��
    if(Recv_index5 >= sizeof(Recv_buf5)) {
        Recv_index5 = 0;
        Recv_flag5 = 0;
    }
}

void RHO_Parse(void)
{
    if(Recv_flag5)
    {
        // ���������Ƕ����ݣ�����Э��������ǰ��һ�£�
        // ֡��ʽ��0xfd Rho CrossL CrossH 0xfe
        if(Recv_index5 >= 3) // ����Ҫ��ͷ�����ݡ�β
        {
          int16_t rho = (int16_t)Recv_buf5[1];
					int8_t cross = Recv_buf5[2];
//          int16_t cross = (int16_t)((Recv_buf5[2]<<8)|Recv_buf5[3]);
          f_rho = rho*1.0;
          f_cross = cross;
        }
        Recv_flag5 = 0;
        Recv_index5 = 0;
    }
}

// 卡尔曼滤波器初始化
void Kalman_Init(void)
{
    kalman_yaw.q = 0.002f;   // 过程噪声，进一步减小，提高平滑度
    kalman_yaw.r = 0.02f;    // 测量噪声，进一步减小，更相信传感器
    kalman_yaw.x = 0.0f;     // 初始状态估计
    kalman_yaw.p = 1.0f;     // 初始估计误差协方差
    kalman_yaw.k = 0.0f;     // 初始卡尔曼增益
}

// 角度归一化函数，将角度限制在-180到180度之间
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 卡尔曼滤波处理yaw值
float Kalman_Filter_Yaw(float measurement)
{
    // 角度归一化
    measurement = normalize_angle(measurement);
    kalman_yaw.x = normalize_angle(kalman_yaw.x);
    
    // 预测步骤
    kalman_yaw.p = kalman_yaw.p + kalman_yaw.q;
    
    // 计算角度差，考虑跨越180度边界的情况
    float angle_diff = measurement - kalman_yaw.x;
    angle_diff = normalize_angle(angle_diff);
    
    // 更新步骤
    kalman_yaw.k = kalman_yaw.p / (kalman_yaw.p + kalman_yaw.r);
    kalman_yaw.x = kalman_yaw.x + kalman_yaw.k * angle_diff;
    kalman_yaw.x = normalize_angle(kalman_yaw.x);
    kalman_yaw.p = (1.0f - kalman_yaw.k) * kalman_yaw.p;
    
    return kalman_yaw.x;
}


const uint8_t unlock_cmd[5]     = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
const uint8_t save_cmd[5]       = {0xFF, 0xAA, 0x00, 0x00, 0x00};
const uint8_t cali_angle_ref[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
const uint8_t cali_z_axis[5]    = {0xFF, 0xAA, 0x01, 0x04, 0x00};
const uint8_t gyro_auto_on[5]   = {0xFF, 0xAA, 0x61, 0x00, 0x00};
const uint8_t gyro_auto_off[5]  = {0xFF, 0xAA, 0x61, 0x01, 0x00};

void JY61P_Send(uint8_t type)
{
    const uint8_t *cali_cmd = NULL;

    switch (type)
    {
    case 0: // 角度参考归零
        cali_cmd = cali_angle_ref;
        break;
    case 1: // Z轴置零
        cali_cmd = cali_z_axis;
        break;
    case 2: // 开启陀螺仪自动校准
        cali_cmd = gyro_auto_on;
        break;
    case 3: // 关闭陀螺仪自动校准
        cali_cmd = gyro_auto_off;
        break;
    default:
        return;
    }

    HAL_UART_Transmit(&huart3, cali_cmd, 5,HAL_TIMEOUT);
    HAL_Delay(3000);
}



