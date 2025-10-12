#include "monitor.h"

#include "stdarg.h"
#include "stdio.h"

float f_roll, f_pitch, f_yaw;
uint8_t Recv3;
uint8_t Recv6;
uint8_t Recv_buf3[100];
uint8_t Recv_buf6[100];
uint8_t Recv_index3 = 0;
uint8_t Recv_index6 = 0;
uint8_t Recv_flag3 = 0;
uint8_t Recv_flag6 = 0;

int my_printf(UART_HandleTypeDef *huart, const char *format, ...) {
  char buffer[512]; // 临时存储格式化后的字符串
  va_list arg;      // 处理可变参数
  int len;          // 最终字符串长度

  va_start(arg, format);
  // 安全地格式化字符串到 buffer
  len = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);

  // 通过 HAL 库发送 buffer 中的内容
  HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
  return len;
}

void Uart3_Receive(void) {
  if (Recv_flag3)
    return;
  // 帧头检测
  if (Recv_index3 == 0 && Recv3 != 0x55)
    return;
  if (Recv_index3 == 1 && Recv3 != 0x53) {
    Recv_index3 = 0; // 帧头第二字节不对，重新等待帧头
    return;
  }
  Recv_buf3[Recv_index3++] = Recv3;
  // 帧尾检测
  if (Recv_index3 >= 2 && Recv_buf3[Recv_index3 - 2] == 0xFB &&
      Recv_buf3[Recv_index3 - 1] == 0x46)
    Recv_flag3 = 1;
  // 防止越界
  if (Recv_index3 >= sizeof(Recv_buf3)) {
    Recv_index3 = 0;
    Recv_flag3 = 0;
  }
}

void Uart3_Parse(void) {
  if (Recv_flag3) {
    // 例：解析角度数据（假设协议内容与前述一致）
    // 帧格式：0x55 0x53 RollL RollH PitchL PitchH YawL YawH ... 0xFB 0x46
    if (Recv_index3 >= 10) // 至少要有头、数据、尾
    {
      int16_t roll = (int16_t)((Recv_buf3[3] << 8) | Recv_buf3[2]);
      int16_t pitch = (int16_t)((Recv_buf3[5] << 8) | Recv_buf3[4]);
      int16_t yaw = (int16_t)((Recv_buf3[7] << 8) | Recv_buf3[6]);
      f_roll = roll / 32768.0f * 180.0f;
      f_pitch = pitch / 32768.0f * 180.0f;
      f_yaw = yaw / 32768.0f * 180.0f;
    }
    Recv_flag3 = 0;
    Recv_index3 = 0;
  }
}

// USART6接收处理函数（用于PN532通信）
void Uart6_Receive(void) {
  if (Recv_flag6)
    return;

  // 调试：显示接收到的字节
  char debug_msg[50];
  sprintf(debug_msg, "DEBUG: Uart6_Receive: 0x%02X (index: %d)\r\n", Recv6,
          Recv_index6);
  HAL_UART_Transmit(&huart4, (uint8_t *)debug_msg, strlen(debug_msg), 1000);

  // 简单的数据接收，不进行帧格式检测
  // 因为PN532的响应格式可能不同
  Recv_buf6[Recv_index6++] = Recv6;

  // 防止越界
  if (Recv_index6 >= sizeof(Recv_buf6)) {
    Recv_index6 = 0;
    Recv_flag6 = 0;
    HAL_UART_Transmit(&huart4, (uint8_t *)"DEBUG: Buffer overflow, reset!\r\n",
                      31, 1000);
  }

  // 如果接收到足够的数据，设置标志
  if (Recv_index6 >= 6) { // 至少6字节（ACK帧长度）
    Recv_flag6 = 1;
    HAL_UART_Transmit(&huart4, (uint8_t *)"DEBUG: Uart6_Receive: Flag set!\r\n",
                      33, 1000);
  }
}

// USART6数据解析函数
void Uart6_Parse(void) {
  HAL_UART_Transmit(&huart4, (uint8_t *)"DEBUG: Uart6_Parse called\r\n", 26,
                    1000);

  if (Recv_flag6) {
    HAL_UART_Transmit(
        &huart4, (uint8_t *)"DEBUG: Flag is set, parsing data\r\n", 32, 1000);

    // 打印接收到的数据用于调试
    HAL_UART_Transmit(&huart4, (uint8_t *)"UART6 received: ", 16, 1000);
    for (int i = 0; i < Recv_index6; i++) {
      char hex[5];
      sprintf(hex, "%02X ", Recv_buf6[i]);
      HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

    // 重置标志和索引
    Recv_flag6 = 0;
    Recv_index6 = 0;
  } else {
    HAL_UART_Transmit(&huart4,
                      (uint8_t *)"DEBUG: Flag not set, no data to parse\r\n",
                      37, 1000);
  }
}
