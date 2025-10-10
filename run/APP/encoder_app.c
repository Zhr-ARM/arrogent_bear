#include "encoder_app.h"

// ���ұ��������
Encoder A_encoder;
Encoder B_encoder;
Encoder C_encoder;
Encoder D_encoder;

/**
 * @brief ��ʼ��������Ӧ��
 */
void Encoder_Init(void)
{
  Encoder_Driver_Init(&A_encoder, &htim1, 0);  // A电机不反向，编码器也不反向
  Encoder_Driver_Init(&B_encoder, &htim2, 1);  // B电机不反向，编码器反向
	Encoder_Driver_Init(&C_encoder, &htim3, 0);  // C电机不反向，编码器不反向
  Encoder_Driver_Init(&D_encoder, &htim4, 0);  // D电机反向，编码器也反向
}

/**
 * @brief ������Ӧ���������� (Ӧ�ɵ����������Ե���)
 */
void Encoder_Task(void)
{
  Encoder_Driver_Update(&A_encoder);
  Encoder_Driver_Update(&B_encoder);
  Encoder_Driver_Update(&C_encoder);
  Encoder_Driver_Update(&D_encoder);	
	
//	my_printf(&huart1,"Left:%.2f Right:%.2f\r\n",left_encoder.speed_cm_s,right_encoder.speed_cm_s);
}
