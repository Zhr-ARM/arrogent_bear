/**************************************************************************
 *  @file     pn532.c
 *  @author   Yehui from Waveshare
 *  @license  BSD
 *
 *  This is a library for the Waveshare PN532 NFC modules
 *
 *  Check out the links above for our tutorials and wiring diagrams
 *  These chips use UART communicate.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **************************************************************************/

#include "pn532.h"
#include "main.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_FRAME_START[] = {0x00, 0x00, 0xFF};
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
#define PN532_FRAME_MAX_LENGTH 255
#define PN532_DEFAULT_TIMEOUT 1000

// MFOC风格唤醒序列定义
#define PN532_WAKEUP_LEN 16

// UART底层通信函数 - 基于MFOC逻辑
int PN532_UART_WriteData(PN532 *pn532, uint8_t *data, uint16_t length) {
  // MFOC风格：先发唤醒序列（激活PN532）
  uint8_t wakeup[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00};

  // 打印唤醒序列
  HAL_UART_Transmit(&huart4, (uint8_t *)"Wakeup sequence: ", 17, 1000);
  for (int i = 0; i < sizeof(wakeup); i++) {
    char hex[5];
    sprintf(hex, "%02X ", wakeup[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  if (HAL_UART_Transmit(&huart6, wakeup, sizeof(wakeup), 1000) != HAL_OK) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Wakeup TX error!\r\n", 18, 1000);
    return PN532_STATUS_ERROR;
  }

  // 打印实际数据
  HAL_UART_Transmit(&huart4, (uint8_t *)"Data to send: ", 14, 1000);
  for (int i = 0; i < length; i++) {
    char hex[5];
    sprintf(hex, "%02X ", data[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 再发实际数据
  if (HAL_UART_Transmit(&huart6, data, length, 1000) != HAL_OK) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Data TX error!\r\n", 16, 1000);
    return PN532_STATUS_ERROR;
  }
  return PN532_STATUS_OK;
}

int PN532_UART_ReadData(PN532 *pn532, uint8_t *buff, uint16_t length) {
  // MFOC风格：轮询接收数据，超时处理
  uint32_t start = HAL_GetTick();
  uint16_t recv_len = 0;

  while (HAL_GetTick() - start < PN532_DEFAULT_TIMEOUT && recv_len < length) {
    uint8_t byte;
    if (HAL_UART_Receive(&huart6, &byte, 1, 10) == HAL_OK) {
      buff[recv_len++] = byte;
    }
  }

  if (recv_len == 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"UART RX timeout!\r\n", 18, 1000);
    return PN532_STATUS_ERROR;
  }

  return recv_len; // 返回实际读长
}

// MFOC风格唤醒函数 - 让PN532彻底"醒"
void PN532_Wakeup(PN532 *pn532) {
  // MFOC风格唤醒序列 - 在函数内定义以避免C89兼容性问题
  static const uint8_t wakeup_sequence[PN532_WAKEUP_LEN] = {
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
      0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  HAL_UART_Transmit(&huart6, (uint8_t *)wakeup_sequence, PN532_WAKEUP_LEN,
                    1000);
  HAL_Delay(10); // 等稳定

  // 清缓冲：收掉垃圾数据
  uint8_t dummy;
  while (HAL_UART_Receive(&huart6, &dummy, 1, 10) == HAL_OK) {
    // 清空缓冲区
  }
}

// MFOC风格帧读取函数 - 轮询+校验
int PN532_ReadFrame(PN532 *pn532, uint8_t *response, uint16_t length) {
  uint8_t buff[PN532_FRAME_MAX_LENGTH + 7];
  uint16_t recv_idx = 0;
  uint32_t start = HAL_GetTick();

  // MFOC轮询：收直到超时或够长
  while (HAL_GetTick() - start < PN532_DEFAULT_TIMEOUT &&
         recv_idx < sizeof(buff)) {
    uint8_t byte;
    if (HAL_UART_Receive(&huart6, &byte, 1, 50) == HAL_OK) { // 超时加到50ms
      buff[recv_idx++] = byte;
    }
  }

  // 调试：显示接收到的原始数据
  HAL_UART_Transmit(&huart4, (uint8_t *)"Raw response from PN532: ", 25, 1000);
  for (int i = 0; i < recv_idx && i < 30; i++) {
    char hex[5];
    sprintf(hex, "%02X ", buff[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 显示接收到的字节数
  char len_msg[30];
  sprintf(len_msg, "Received %d bytes total\r\n", recv_idx);
  HAL_UART_Transmit(&huart4, (uint8_t *)len_msg, strlen(len_msg), 1000);

  if (recv_idx < 6) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Response too short!\r\n", 21, 1000);
    return PN532_STATUS_ERROR;
  }

  // MFOC解析：跳前导0x00，找00 FF
  uint16_t offset = 0;
  while (offset < recv_idx && buff[offset] == 0x00)
    offset++;
  if (offset >= recv_idx || buff[offset] != 0xFF) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"No 00 FF header!\r\n", 18, 1000);
    return PN532_STATUS_ERROR;
  }
  offset++; // 跳FF

  // 取LEN和LCS，校验LEN + LCS == 0
  if (offset + 1 >= recv_idx)
    return PN532_STATUS_ERROR;
  uint8_t frame_len = buff[offset];
  uint8_t lcs = buff[offset + 1];

  // 调试：显示长度信息
  char len_info[50];
  sprintf(len_info, "Frame length: %d, LCS: %d, Sum: %d\r\n", frame_len, lcs,
          frame_len + lcs);
  HAL_UART_Transmit(&huart4, (uint8_t *)len_info, strlen(len_info), 1000);

  if ((frame_len + lcs) != 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Length checksum fail!\r\n", 22,
                      1000);
    return PN532_STATUS_ERROR;
  }
  offset += 2;

  // 取数据，计算校验
  if (offset + frame_len + 1 > recv_idx)
    return PN532_STATUS_ERROR;
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < frame_len; i++) {
    response[i] = buff[offset + i];
    checksum += response[i];
  }
  uint8_t dcs = buff[offset + frame_len];

  // 调试：显示数据校验信息
  char data_info[50];
  sprintf(data_info, "Data checksum: %d, DCS: %d, Sum: %d\r\n", checksum, dcs,
          checksum + dcs);
  HAL_UART_Transmit(&huart4, (uint8_t *)data_info, strlen(data_info), 1000);

  if ((checksum + dcs) != 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Data checksum fail!\r\n", 20, 1000);
    return PN532_STATUS_ERROR;
  }
  offset += frame_len + 1; // 跳DCS，检查后缀00

  if (offset >= recv_idx || buff[offset] != 0x00) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"No trailing 00!\r\n", 17, 1000);
  }

  // 调试：显示解析后的数据
  HAL_UART_Transmit(&huart4, (uint8_t *)"Parsed data: ", 13, 1000);
  for (int i = 0; i < frame_len && i < 20; i++) {
    char hex[5];
    sprintf(hex, "%02X ", response[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  return frame_len; // 返回数据长度
}

// MFOC风格帧写入函数
int PN532_WriteFrame(PN532 *pn532, uint8_t *data, uint16_t length) {
  uint8_t frame[PN532_FRAME_MAX_LENGTH + 7];
  uint16_t frame_len = 0;

  // 前导码
  frame[frame_len++] = 0x00;
  frame[frame_len++] = 0x00;
  frame[frame_len++] = 0xFF;

  // 长度和长度校验和
  frame[frame_len++] = length;
  frame[frame_len++] = 0x100 - length; // LCS

  // 数据
  for (uint16_t i = 0; i < length; i++) {
    frame[frame_len++] = data[i];
  }

  // 数据校验和
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < length; i++) {
    checksum += data[i];
  }
  frame[frame_len++] = 0x100 - checksum; // DCS

  // 后导码
  frame[frame_len++] = 0x00;

  return PN532_UART_WriteData(pn532, frame, frame_len);
}

// 读取直到匹配 ACK: 00 00 FF 00 FF 00 （返回1=收到了，0=超时）
static int recv_ack(UART_HandleTypeDef *huart, uint32_t timeout_ms) {
  const uint8_t ACK_PREFIX[3] = {0x00, 0x00, 0xFF};
  extern uint16_t uart6_rx_index;
  extern uint8_t uart6_rx_buffer[256];

  uint32_t t0 = HAL_GetTick();
  uint16_t start_index = uart6_rx_index;

  // 添加调试信息
  HAL_UART_Transmit(&huart4, (uint8_t *)"Waiting for ACK...\r\n", 19, 100);

  while (HAL_GetTick() - t0 < timeout_ms) {
    // 检查是否有新数据到达
    if (uart6_rx_index > start_index) {
      // 打印接收到的每个字节
      for (int i = start_index; i < uart6_rx_index; i++) {
        char hex[5];
        sprintf(hex, "%02X ", uart6_rx_buffer[i]);
        HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 100);
      }

      // 检查是否有ACK前缀
      for (int i = start_index; i <= uart6_rx_index - 3; i++) {
        if (uart6_rx_buffer[i] == ACK_PREFIX[0] &&
            uart6_rx_buffer[i + 1] == ACK_PREFIX[1] &&
            uart6_rx_buffer[i + 2] == ACK_PREFIX[2]) {
          HAL_UART_Transmit(&huart4, (uint8_t *)"\r\nACK received!\r\n", 17,
                            100);
          return 1; // 命中ACK前缀
        }
      }
      start_index = uart6_rx_index; // 更新起始索引
    }
    vTaskDelay(10); // 短暂延时
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\nACK timeout!\r\n", 15, 100);
  return 0; // 超时无ACK
}

// 读取"完整响应帧"，剥掉主机帧头，返回 Data 段到 resp（含 TFI 和 response
// cmd，比如 D5 41 ...） 返回 >0 表示 data_len，=0 超时
static int recv_response(UART_HandleTypeDef *huart, uint8_t *resp, int resp_max,
                         uint32_t timeout_ms) {
  extern uint16_t uart6_rx_index;
  extern uint8_t uart6_rx_buffer[256];

  uint32_t t0 = HAL_GetTick();
  uint16_t start_index = uart6_rx_index;
  int stage = 0; // 0:找第1个0x00, 1:找第2个0x00, 2:找0xFF, 3:收剩余
  int frame_start = -1;

  // 添加调试信息
  HAL_UART_Transmit(&huart4, (uint8_t *)"Waiting for response...\r\n", 24, 100);

  // 首先检查缓冲区中是否已经有完整响应（从后往前找最新的响应）
  for (int i = uart6_rx_index - 3; i >= 0; i--) {
    if (uart6_rx_buffer[i] == 0x00 && uart6_rx_buffer[i + 1] == 0x00 &&
        uart6_rx_buffer[i + 2] == 0xFF) {
      // 找到帧头，检查是否有完整帧
      if (i + 4 < uart6_rx_index) {
        uint8_t LEN = uart6_rx_buffer[i + 3];
        int need = 3 + 2 + LEN + 1 + 1; // 帧头 + LEN/LCS + 数据 + DCS + 帧尾

        if (i + need <= uart6_rx_index) {
          // 完整帧已存在，进行校验
          uint8_t LCS = uart6_rx_buffer[i + 4];
          if ((uint8_t)(LEN + LCS) == 0x00) {
            // 校验 DCS
            int data_off = i + 5;
            uint16_t sum = 0;
            for (int j = 0; j < LEN; j++) {
              sum += uart6_rx_buffer[data_off + j];
            }
            uint8_t DCS = uart6_rx_buffer[data_off + LEN];
            if ((uint8_t)((sum + DCS) & 0xFF) == 0x00) {
              // 检查是否是PN532响应 (D5 41)
              if (LEN >= 3 && uart6_rx_buffer[data_off] == 0xD5 &&
                  uart6_rx_buffer[data_off + 1] == 0x41) {
                // 提取 data 段
                int copy = (LEN < resp_max) ? LEN : resp_max;
                memcpy(resp, &uart6_rx_buffer[data_off], copy);
                char debug_msg[50];
                sprintf(debug_msg, "PN532 response found: LEN=%d, copy=%d\r\n",
                        LEN, copy);
                HAL_UART_Transmit(&huart4, (uint8_t *)debug_msg,
                                  strlen(debug_msg), 100);
                return copy;
              }
            }
          }
        }
      }
    }
  }

  while (HAL_GetTick() - t0 < timeout_ms) {
    // 检查是否有新数据到达
    if (uart6_rx_index > start_index) {
      // 打印接收到的每个字节
      for (int i = start_index; i < uart6_rx_index; i++) {
        char hex[5];
        sprintf(hex, "%02X ", uart6_rx_buffer[i]);
        HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 100);
      }

      // 处理新接收的数据
      for (int i = start_index; i < uart6_rx_index; i++) {
        uint8_t b = uart6_rx_buffer[i];

        if (stage == 0) {
          if (b == 0x00) {
            frame_start = i;
            stage = 1;
          }
          continue;
        }

        if (stage == 1) {
          if (b == 0x00) {
            stage = 2;
          } else {
            stage = 0;
            frame_start = -1;
          }
          continue;
        }

        if (stage == 2) {
          if (b == 0xFF) {
            stage = 3;
          } else {
            stage = 0;
            frame_start = -1;
          }
          continue;
        }

        if (stage == 3) {
          // 检查是否有足够的数据来解析帧
          if (i >= frame_start + 4) {
            uint8_t LEN = uart6_rx_buffer[frame_start + 3];
            int need =
                3 + 2 + LEN + 1 + 1; // 帧头 + LEN/LCS + 数据 + DCS + 帧尾

            if (i >= frame_start + need - 1) {
              // 完整帧已接收，进行校验
              uint8_t LCS = uart6_rx_buffer[frame_start + 4];
              if ((uint8_t)(LEN + LCS) != 0x00) {
                // LCS 错误，重新开始
                stage = 0;
                frame_start = -1;
                continue;
              }

              // 校验 DCS
              int data_off = frame_start + 5;
              uint16_t sum = 0;
              for (int j = 0; j < LEN; j++) {
                sum += uart6_rx_buffer[data_off + j];
              }
              uint8_t DCS = uart6_rx_buffer[data_off + LEN];
              if ((uint8_t)((sum + DCS) & 0xFF) != 0x00) {
                // DCS 错误，重新开始
                stage = 0;
                frame_start = -1;
                continue;
              }

              // 检查是否是PN532响应 (D5 41)
              if (LEN >= 3 && uart6_rx_buffer[data_off] == 0xD5 &&
                  uart6_rx_buffer[data_off + 1] == 0x41) {
                // 提取 data 段
                int copy = (LEN < resp_max) ? LEN : resp_max;
                memcpy(resp, &uart6_rx_buffer[data_off], copy);
                char debug_msg[50];
                sprintf(debug_msg,
                        "\r\nPN532 response received: LEN=%d, copy=%d\r\n", LEN,
                        copy);
                HAL_UART_Transmit(&huart4, (uint8_t *)debug_msg,
                                  strlen(debug_msg), 100);
                return copy;
              } else {
                // 不是PN532响应，重新开始
                stage = 0;
                frame_start = -1;
                continue;
              }
            }
          }
        }
      }
      start_index = uart6_rx_index; // 更新起始索引
    }
    vTaskDelay(10); // 短暂延时
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\nResponse timeout!\r\n", 20, 100);
  return 0; // 超时无完整响应
}

// 发送整帧（发前清空RX），不打印
static int pn532_send_frame_quiet(UART_HandleTypeDef *huart,
                                  const uint8_t *tfi_data, int tfi_len) {
  uint8_t frame[64];
  int idx = 0;
  uint8_t LEN = (uint8_t)tfi_len;
  uint8_t LCS = (uint8_t)(0x100 - LEN);
  uint16_t sum = 0;
  for (int i = 0; i < tfi_len; i++)
    sum += tfi_data[i];
  uint8_t DCS = (uint8_t)(0x100 - (sum & 0xFF));
  // 清空残留
  uint8_t d;
  while (HAL_UART_Receive(huart, &d, 1, 0) == HAL_OK) {
  }

  frame[idx++] = 0x00;
  frame[idx++] = 0x00;
  frame[idx++] = 0xFF;
  frame[idx++] = LEN;
  frame[idx++] = LCS;
  memcpy(&frame[idx], tfi_data, tfi_len);
  idx += tfi_len;
  frame[idx++] = DCS;
  frame[idx++] = 0x00;

  // 打印完整发送的PN532帧（使用非阻塞方式避免干扰UART6）
  HAL_UART_Transmit(&huart4, (uint8_t *)"Full TX frame: ", 16, 100);
  for (int i = 0; i < idx; i++) {
    char hex[5];
    sprintf(hex, "%02X ", frame[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 100);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 100);

  HAL_StatusTypeDef status = HAL_UART_Transmit(huart, frame, idx, 1000);
  if (status != HAL_OK) {
    char error_msg[50];
    sprintf(error_msg, "UART TX failed: %d\r\n", status);
    HAL_UART_Transmit(&huart4, (uint8_t *)error_msg, strlen(error_msg), 100);
  }
  return (status == HAL_OK) ? 1 : 0;
}

// 发命令→等ACK→等响应：期间严禁任何打印
int pn532_call_quiet(UART_HandleTypeDef *huart, uint8_t cmd,
                     const uint8_t *params, int params_len, uint8_t *resp,
                     int resp_max) {
  uint8_t tfi[32];
  int n = 0;
  tfi[n++] = 0xD4;
  tfi[n++] = cmd;
  if (params_len > 0) {
    memcpy(&tfi[n], params, params_len);
    n += params_len;
  }

  if (!pn532_send_frame_quiet(huart, tfi, n)) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Send frame failed\r\n", 19, 100);
    return -1;
  }

  // 发送后立即检查UART6是否有数据
  HAL_UART_Transmit(&huart4, (uint8_t *)"Checking UART6 for data...\r\n", 28,
                    100);
  uint8_t test_byte;
  if (HAL_UART_Receive(huart, &test_byte, 1, 10) == HAL_OK) {
    char hex[5];
    sprintf(hex, "Found: %02X ", test_byte);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, strlen(hex), 100);
  } else {
    HAL_UART_Transmit(&huart4, (uint8_t *)"No immediate data\r\n", 19, 100);
  }

  // **只收ACK，不打印**
  if (!recv_ack(huart, 200)) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"ACK timeout\r\n", 13, 100);
    HAL_UART_Transmit(&huart4, (uint8_t *)"Auth command result: 0\r\n", 23,
                      100);
    HAL_UART_Transmit(&huart4,
                      (uint8_t *)"Waiting 1000ms after auth command...\r\n", 37,
                      1000);
    vTaskDelay(1000);
    return 0; // 给200ms窗口
  }

  // **收响应，不打印**
  int m = recv_response(huart, resp, resp_max, 500);
  if (m <= 0) {
    char debug_msg[30];
    sprintf(debug_msg, "Response timeout: %d\r\n", m);
    HAL_UART_Transmit(&huart4, (uint8_t *)debug_msg, strlen(debug_msg), 100);
  }
  return m; // >0 有响应；0 超时；-1 发送错
}

// 返回：>0 表示收到响应字节数（放在 resp），0 表示超时/错误，-1 表示底层错误
int PN532_CallFunction_HSU(uint8_t cmd, uint8_t *resp, int resp_max,
                           uint8_t *params, uint16_t params_len,
                           uint32_t timeout_ms) {
  // 组装 TFI+DATA : TFI = 0xD4, CMD = cmd
  int tfi_len = 2 + params_len;
  uint8_t tfi_and_data[256];
  tfi_and_data[0] = 0xD4;
  tfi_and_data[1] = cmd; // PN532_COMMAND_INDATAEXCHANGE == 0x40
  if (params_len)
    memcpy(&tfi_and_data[2], params, params_len);

  // LEN/LCS
  uint8_t LEN = (uint8_t)tfi_len;
  uint8_t LCS = (uint8_t)(0x100 - LEN);

  // DCS = 0x100 - (sum(TFI+DATA) & 0xFF)
  uint16_t sum = 0;
  for (int i = 0; i < tfi_len; i++)
    sum += tfi_and_data[i];
  uint8_t DCS = (uint8_t)(0x100 - (sum & 0xFF));

  // 构造完整帧
  // 00 00 FF LEN LCS [TFI+DATA] DCS 00
  uint8_t frame[512];
  int idx = 0;
  frame[idx++] = 0x00;
  frame[idx++] = 0x00;
  frame[idx++] = 0xFF;
  frame[idx++] = LEN;
  frame[idx++] = LCS;
  memcpy(&frame[idx], tfi_and_data, tfi_len);
  idx += tfi_len;
  frame[idx++] = DCS;
  frame[idx++] = 0x00;

  // 发送前：打印完整主机帧
  HAL_UART_Transmit(&huart4, (uint8_t *)"Sending complete frame: ", 25, 1000);
  for (int i = 0; i < idx; i++) {
    char hex[5];
    sprintf(hex, "%02X ", frame[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 发送整帧（以二进制发送）
  if (HAL_UART_Transmit(&huart6, frame, idx, 200) != HAL_OK) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Frame send failed!\r\n", 20, 1000);
    return -1;
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"Frame sent successfully!\r\n", 26,
                    1000);

  // 发送后：使用优化的ACK接收函数
  HAL_UART_Transmit(&huart4, (uint8_t *)"Waiting for ACK...\r\n", 20, 1000);

  if (recv_ack(&huart6, 50)) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"ACK OK!\r\n", 9, 1000);
  } else {
    HAL_UART_Transmit(&huart4, (uint8_t *)"NO ACK received!\r\n", 18, 1000);
    return 0;
  }

  // 收响应帧：使用优化的响应接收函数
  HAL_UART_Transmit(&huart4, (uint8_t *)"Reading response...\r\n", 20, 1000);

  int n = recv_response(&huart6, resp, resp_max, timeout_ms);

  if (n > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Response data: ", 15, 1000);
    for (int i = 0; i < n; i++) {
      char hex[5];
      sprintf(hex, "%02X ", resp[i]);
      HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);
    return n;
  } else {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Response timeout!\r\n", 19, 1000);
    return 0;
  }
}

// 保持原有接口兼容性
int PN532_CallFunction(PN532 *pn532, uint8_t command, uint8_t *response,
                       uint16_t response_length, uint8_t *params,
                       uint16_t params_length, uint32_t timeout) {
  return PN532_CallFunction_HSU(command, response, response_length, params,
                                params_length, timeout);
}

// MFOC风格找卡函数
int PN532_ReadPassiveTarget_MFOC(PN532 *pn532, uint8_t *uid, uint8_t card_baud,
                                 uint32_t timeout) {
  // 原命令：D4 4A 01 00 (MIFARE Classic)
  uint8_t params[] = {0x01, 0x00}; // 1张卡，MIFARE 106kbps
  uint8_t buff[32];                // 加长缓冲

  // 重试机制：循环5次
  for (int retry = 0; retry < 5; retry++) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Card search retry ", 18, 1000);
    char retry_str[4];
    sprintf(retry_str, "%d", retry + 1);
    HAL_UART_Transmit(&huart4, (uint8_t *)retry_str, strlen(retry_str), 1000);
    HAL_UART_Transmit(&huart4, (uint8_t *)"/5...\r\n", 7, 1000);

    int len = PN532_CallFunction(pn532, PN532_COMMAND_INLISTPASSIVETARGET, buff,
                                 sizeof(buff), params, (uint16_t)sizeof(params),
                                 timeout);
    if (len < 0) {
      HAL_UART_Transmit(&huart4, (uint8_t *)"CallFunction failed\r\n", 21,
                        1000);
      HAL_Delay(2000); // 等待2秒再重试
      continue;
    }

    // MFOC解析：buff[0]应D5, buff[1]4B, buff[2] num_targets=1, buff[7] uid_len
    if (buff[0] != PN532_PN532TOHOST ||
        buff[1] != (PN532_COMMAND_INLISTPASSIVETARGET + 1)) {
      HAL_UART_Transmit(&huart4, (uint8_t *)"Not list target response!\r\n", 27,
                        1000);
      HAL_Delay(2000);
      continue;
    }

    uint8_t num_targets = buff[2];
    if (num_targets != 1) {
      HAL_UART_Transmit(&huart4, (uint8_t *)"No or multiple cards!\r\n", 22,
                        1000);
      HAL_Delay(2000);
      continue;
    }

    uint8_t uid_len = buff[7];
    if (uid_len > 7 || uid_len < 4) {
      HAL_UART_Transmit(&huart4, (uint8_t *)"Invalid UID len!\r\n", 18, 1000);
      HAL_Delay(2000);
      continue;
    }

    memcpy(uid, &buff[8], uid_len); // 存UID

    // 打印日志，像MFOC
    HAL_UART_Transmit(&huart4, (uint8_t *)"Card found! UID: ", 17, 1000);
    for (uint8_t i = 0; i < uid_len; i++) {
      char hex[5];
      sprintf(hex, "%02X ", uid[i]);
      HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

    return uid_len; // 返回UID长度
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)"All retries failed!\r\n", 20, 1000);
    return PN532_STATUS_ERROR;
  }

// MIFARE Classic认证函数
// 成功返回 PN532_STATUS_OK，失败 PN532_STATUS_ERROR
int PN532_MifareClassicAuthenticate_UART(PN532 *pn532, uint8_t *uid,
                                         uint8_t uid_len, uint8_t block_number,
                                         uint8_t *key, uint8_t key_type) {
  // params 只放 InDataExchange 的 "DATA" 部分：01 60/61 block KEY[6] UID[4]
  uint8_t params[13];
  uint8_t t = (key_type == 0x61) ? 0x61 : 0x60; // 0x60=KeyA, 0x61=KeyB
  uint8_t uid4[4] = {0};

  // 只取 UID 的后 4 字节（常见 4 字节卡即为全部）
  if (uid_len >= 4) {
    uid4[0] = uid[0];
    uid4[1] = uid[1];
    uid4[2] = uid[2];
    uid4[3] = uid[3];
  }

  params[0] = 0x01;         // Target #1（通常为 1）
  params[1] = t;            // 0x60 (KeyA) or 0x61 (KeyB)
  params[2] = block_number; // 块号

  // 密钥 6 字节（用传入 key，而不是写死全 FF）
  for (int i = 0; i < 6; i++) {
    params[3 + i] = key ? key[i] : 0xFF;
  }

  // UID 后 4 字节
  for (int i = 0; i < 4; i++) {
    params[9 + i] = uid4[i];
  }

  // 调试：打印实际 13 字节参数
  HAL_UART_Transmit(&huart4, (uint8_t *)"Auth params(13): ", 17, 1000);
  for (int i = 0; i < 13; i++) {
    char hex[4];
    sprintf(hex, "%02X ", params[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  uint8_t resp[40] = {0};
  // 使用静默函数发送认证命令，避免日志打印干扰
  HAL_UART_Transmit(
      &huart4, (uint8_t *)"Sending auth command (quiet mode)...\r\n", 35, 100);

  int n = pn532_call_quiet(&huart6, PN532_COMMAND_INDATAEXCHANGE, params,
                           sizeof(params), resp, sizeof(resp));

  // 添加调试信息
  char debug_msg[50];
  sprintf(debug_msg, "Auth command result: %d\r\n", n);
  HAL_UART_Transmit(&huart4, (uint8_t *)debug_msg, strlen(debug_msg), 100);

  // 打印响应
  HAL_UART_Transmit(&huart4, (uint8_t *)"Auth resp: ", 11, 1000);
  for (int i = 0; i < n && i < 32; i++) {
    char hex[4];
    sprintf(hex, "%02X ", resp[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 兼容两种返回形态：
  // 形态A（常见）：resp = D5 41 00            （len >= 3）
  // 形态B（库已剥头）：resp = 00 [ ..data.. ] （len >= 1）
  if (n >= 3 && resp[0] == 0xD5 && resp[1] == 0x41 && resp[2] == 0x00) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Auth OK (D5 41 00)\r\n", 20, 1000);
  return PN532_STATUS_OK;
}
  if (n >= 1 && resp[0] == 0x00) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Auth OK (status=00)\r\n", 21, 1000);
  return PN532_STATUS_OK;
}

  HAL_UART_Transmit(&huart4, (uint8_t *)"Auth FAIL\r\n", 11, 1000);
    return PN532_STATUS_ERROR;
  }

int PN532_MifareClassicRead_UART(PN532 *pn532, uint8_t block_number,
                                 uint8_t *data) {
  // params: 01 30 <block>
  uint8_t params[3];
  params[0] = 0x01;         // Target #1
  params[1] = 0x30;         // Read (Mifare Classic)
  params[2] = block_number; // Block

  HAL_UART_Transmit(&huart4, (uint8_t *)"Read params: ", 13, 1000);
  for (int i = 0; i < 3; i++) {
    char hex[4];
    sprintf(hex, "%02X ", params[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  uint8_t resp[64] = {0};
  int n = pn532_call_quiet(&huart6, PN532_COMMAND_INDATAEXCHANGE, params,
                           sizeof(params), resp, sizeof(resp));

  HAL_UART_Transmit(&huart4, (uint8_t *)"Read resp: ", 11, 1000);
  for (int i = 0; i < n && i < 40; i++) {
    char hex[4];
    sprintf(hex, "%02X ", resp[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 形态A：D5 41 00 + 16字节数据
  if (n >= 3 + 16 && resp[0] == 0xD5 && resp[1] == 0x41 && resp[2] == 0x00) {
    for (int i = 0; i < 16; i++)
      data[i] = resp[3 + i];
    HAL_UART_Transmit(&huart4, (uint8_t *)"Read OK (D5 41 00)\r\n", 20, 1000);
    return PN532_STATUS_OK;
  }
  // 形态B：00 + 16字节数据（库已剥头）
  if (n >= 1 + 16 && resp[0] == 0x00) {
    for (int i = 0; i < 16; i++)
      data[i] = resp[1 + i];
    HAL_UART_Transmit(&huart4, (uint8_t *)"Read OK (status=00)\r\n", 21, 1000);
    return PN532_STATUS_OK;
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)"Read FAIL\r\n", 11, 1000);
  return PN532_STATUS_ERROR;
}

// 新的MIFARE Classic读取函数 - 使用PN532标准格式
int PN532_MifareClassicReadWithUID_UART(PN532 *pn532, uint8_t block_number,
                                        uint8_t *data, uint8_t *uid,
                                        uint8_t uid_length) {
  // PN532标准读取命令格式: D4 40 01 30 <block_number>
  // 注意：标准PN532读取命令不需要UID，认证后直接读取即可
  uint8_t params[3];
  params[0] = 0x01;         // 目标编号
  params[1] = 0x30;         // 读取命令
  params[2] = block_number; // 块号

  // 调试：打印读取命令 (D4 40 01 30 <block_number>)
  HAL_UART_Transmit(&huart4, (uint8_t *)"Read TFI+DATA: D4 40 01 30 ", 30,
                    1000);
  char block_hex[5];
  sprintf(block_hex, "%02X\r\n", block_number);
  HAL_UART_Transmit(&huart4, (uint8_t *)block_hex, 4, 1000);

  // 按照公式计算DCS: (0x00 - (TFI + 所有DATA字节之和)) & 0xFF
  // TFI+DATA: D4 40 01 30 <block>
  uint16_t sum_tfi_data = 0xD4 + 0x40 + 0x01 + 0x30 + block_number;
  uint8_t dcs_calc = (uint8_t)((0x00 - (sum_tfi_data & 0xFF)) & 0xFF);
  HAL_UART_Transmit(&huart4, (uint8_t *)"Read DCS calc: ", 14, 1000);
  char dcs_hex[6];
  sprintf(dcs_hex, "%02X\r\n", dcs_calc);
  HAL_UART_Transmit(&huart4, (uint8_t *)dcs_hex, 4, 1000);

  uint8_t response[32];
  int result = pn532_call_quiet(&huart6, PN532_COMMAND_INDATAEXCHANGE, params,
                                3, response, sizeof(response));

  // 调试：打印读取响应
  HAL_UART_Transmit(&huart4, (uint8_t *)"Read response: ", 15, 1000);
  for (int i = 0; i < result && i < 20; i++) {
    char hex[5];
    sprintf(hex, "%02X ", response[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  // 解析读取响应
  return parse_read_response(response, result, data);
}

// 测试函数
int PN532_MFOC_Test(PN532 *pn532) {
  HAL_UART_Transmit(&huart4, (uint8_t *)"=== PN532 MFOC Test ===\r\n", 25,
                    1000);

  // 测试GetFirmwareVersion
  uint8_t response[32];
  uint8_t params[1] = {0}; // 空参数数组
  int result =
      PN532_CallFunction(pn532, PN532_COMMAND_GETFIRMWAREVERSION, response,
                         sizeof(response), params, (uint16_t)0, 1000);

  if (result > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Firmware version: ", 18, 1000);
    for (int i = 0; i < result; i++) {
      char hex[5];
      sprintf(hex, "%02X ", response[i]);
      HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);
  }

  // 测试找卡
  uint8_t uid[7];
  int uid_len = PN532_ReadPassiveTarget_MFOC(pn532, uid, 0, 1000);
  if (uid_len > 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"Card found!\r\n", 12, 1000);
  } else {
    HAL_UART_Transmit(&huart4, (uint8_t *)"No card found\r\n", 15, 1000);
  }

  return PN532_STATUS_OK;
}

// SAM配置函数 - 在唤醒后必须执行，避免InDataExchange阶段异常
int pn532_sam_configuration(UART_HandleTypeDef *huart) {
  // SAM配置命令: 00 00 FF FB D4 14 01 00 00 00 00 00
  uint8_t sam_cmd[] = {0x00, 0x00, 0xFF, 0xFB, 0xD4, 0x14,
                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

  // 打印SAM配置完整命令（使用非阻塞方式避免干扰UART6）
  HAL_UART_Transmit(&huart4, (uint8_t *)"SAM config full command: ", 25, 100);
  for (int i = 0; i < sizeof(sam_cmd); i++) {
    char hex[5];
    sprintf(hex, "%02X ", sam_cmd[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 100);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 100);

  // 发送前清空接收缓冲区
  uint8_t dummy_clear[256];
  HAL_UART_Receive(huart, dummy_clear, 256, 50);

  // 发送SAM配置命令
  if (pn532_send_frame_quiet(huart, sam_cmd, sizeof(sam_cmd)) != 0) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"SAM config send failed!\r\n", 25,
                      1000);
    return -1;
  }

  // 增加延时等待模块处理
  HAL_Delay(100);

  // 等待ACK (00 00 FF 00 FF 00) - 增加超时时间
  uint8_t ack[6];
  if (HAL_UART_Receive(huart, ack, 6, 500) != HAL_OK) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"SAM config ACK timeout!\r\n", 25,
                      1000);
    return -1;
  }

  // 检查ACK格式
  if (ack[0] != 0x00 || ack[1] != 0x00 || ack[2] != 0xFF || ack[3] != 0x00 ||
      ack[4] != 0xFF || ack[5] != 0x00) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"SAM config ACK format error!\r\n",
                      30, 1000);
    return -1;
  }

  // 增加延时等待响应
  HAL_Delay(200);

  // 等待响应 (D5 15 00) - 增加超时时间
  uint8_t response[3];
  if (HAL_UART_Receive(huart, response, 3, 1000) != HAL_OK) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"SAM config response timeout!\r\n",
                      30, 1000);
    return -1;
  }

  // 检查响应格式
  if (response[0] != 0xD5 || response[1] != 0x15 || response[2] != 0x00) {
    HAL_UART_Transmit(
        &huart4, (uint8_t *)"SAM config response format error!\r\n", 35, 1000);
    return -1;
  }

  return 0;
}

// 解析读取响应的独立函数
int parse_read_response(uint8_t *response, int result, uint8_t *data) {
  // 支持两种响应格式
  if (result >= 3 && response[0] == 0xD5 && response[1] == 0x41 &&
      response[2] == 0x00) {
    // PN532标准格式：D5 41 00 + 16字节数据
    if (result >= 3 + 16) {
      for (int i = 0; i < 16; i++) {
        data[i] = response[3 + i];
      }
      HAL_UART_Transmit(&huart4, (uint8_t *)"Read successful! (D5 41 00)\r\n",
                        28, 1000);
      return PN532_STATUS_OK;
    } else {
      HAL_UART_Transmit(
          &huart4, (uint8_t *)"Read failed: insufficient data\r\n", 31, 1000);
      return PN532_STATUS_ERROR;
    }
  } else if (result > 0 && response[0] == 0x00) {
    // 简化格式：00 + 16字节数据
    if (result >= 1 + 16) {
      for (int i = 0; i < 16; i++) {
        data[i] = response[1 + i];
      }
      HAL_UART_Transmit(&huart4, (uint8_t *)"Read successful! (status=00)\r\n",
                        29, 1000);
      return PN532_STATUS_OK;
    } else {
      HAL_UART_Transmit(
          &huart4, (uint8_t *)"Read failed: insufficient data\r\n", 31, 1000);
      return PN532_STATUS_ERROR;
    }
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)"Read failed: unknown format\r\n", 28,
                    1000);
  return PN532_STATUS_ERROR;
}
