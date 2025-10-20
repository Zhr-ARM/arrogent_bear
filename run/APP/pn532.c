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
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_FRAME_START[] = {0x00, 0x00, 0xFF};
extern UART_HandleTypeDef huart6;

#define PN532_FRAME_MAX_LENGTH 255
#define PN532_DEFAULT_TIMEOUT 1000

// 静态函数声明
static int recv_ack(UART_HandleTypeDef *huart, uint32_t timeout_ms);
static int recv_response(UART_HandleTypeDef *huart, uint8_t *resp, int resp_max, uint32_t timeout_ms);
static int pn532_send_frame_quiet(UART_HandleTypeDef *huart, const uint8_t *tfi_data, int tfi_len);
int pn532_call_quiet(UART_HandleTypeDef *huart, uint8_t cmd, const uint8_t *params, int params_len, uint8_t *resp, int resp_max);

// 读取ACK帧
static int recv_ack(UART_HandleTypeDef *huart, uint32_t timeout_ms) {
  const uint8_t ACK_PREFIX[3] = {0x00, 0x00, 0xFF};
  extern uint16_t uart6_rx_index;
  extern uint8_t uart6_rx_buffer[256];

  uint32_t t0 = HAL_GetTick();
  uint16_t start_index = uart6_rx_index;

  while (HAL_GetTick() - t0 < timeout_ms) {
    if (uart6_rx_index > start_index) {
      int end = (int)uart6_rx_index - 3;
      if (end < (int)start_index) {
        start_index = uart6_rx_index;
      } else {
        for (int i = (int)start_index; i <= end; i++) {
        if (uart6_rx_buffer[i] == ACK_PREFIX[0] &&
            uart6_rx_buffer[i + 1] == ACK_PREFIX[1] &&
            uart6_rx_buffer[i + 2] == ACK_PREFIX[2]) {
          return 1;
        }
        }
        start_index = uart6_rx_index;
      }
    }
    vTaskDelay(10);
  }
  return 0;
}

// 读取响应帧
static int recv_response(UART_HandleTypeDef *huart, uint8_t *resp, int resp_max, uint32_t timeout_ms) {
  extern uint16_t uart6_rx_index;
  extern uint8_t uart6_rx_buffer[256];

  uint32_t t0 = HAL_GetTick();
  uint16_t start_index = uart6_rx_index;
  int stage = 0;
  int frame_start = -1;

  // 检查缓冲区中是否已有完整响应
  if (uart6_rx_index >= 3) {
    for (int i = (int)uart6_rx_index - 3; i >= 0; i--) {
    if (uart6_rx_buffer[i] == 0x00 && uart6_rx_buffer[i + 1] == 0x00 &&
        uart6_rx_buffer[i + 2] == 0xFF) {
      if (i + 4 < uart6_rx_index) {
        uint8_t LEN = uart6_rx_buffer[i + 3];
        int need = 3 + 2 + LEN + 1 + 1;

        if (i + need <= uart6_rx_index) {
          uint8_t LCS = uart6_rx_buffer[i + 4];
          if ((uint8_t)(LEN + LCS) == 0x00) {
            int data_off = i + 5;
            uint16_t sum = 0;
            for (int j = 0; j < LEN; j++) {
              sum += uart6_rx_buffer[data_off + j];
            }
            uint8_t DCS = uart6_rx_buffer[data_off + LEN];
            if ((uint8_t)((sum + DCS) & 0xFF) == 0x00) {
              if (LEN >= 3 && uart6_rx_buffer[data_off] == 0xD5 &&
                  uart6_rx_buffer[data_off + 1] == 0x41) {
                int copy = (LEN < resp_max) ? LEN : resp_max;
                memcpy(resp, &uart6_rx_buffer[data_off], copy);
                return copy;
              }
            }
          }
        }
      }
    }
    }
  }

  while (HAL_GetTick() - t0 < timeout_ms) {
    if (uart6_rx_index > start_index) {
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
          if (i >= frame_start + 4) {
            uint8_t LEN = uart6_rx_buffer[frame_start + 3];
            int need = 3 + 2 + LEN + 1 + 1;

            if (i >= frame_start + need - 1) {
              uint8_t LCS = uart6_rx_buffer[frame_start + 4];
              if ((uint8_t)(LEN + LCS) != 0x00) {
                stage = 0;
                frame_start = -1;
                continue;
              }

              int data_off = frame_start + 5;
              uint16_t sum = 0;
              for (int j = 0; j < LEN; j++) {
                sum += uart6_rx_buffer[data_off + j];
              }
              uint8_t DCS = uart6_rx_buffer[data_off + LEN];
              if ((uint8_t)((sum + DCS) & 0xFF) != 0x00) {
                stage = 0;
                frame_start = -1;
                continue;
              }

              if (LEN >= 3 && uart6_rx_buffer[data_off] == 0xD5 &&
                  uart6_rx_buffer[data_off + 1] == 0x41) {
                int copy = (LEN < resp_max) ? LEN : resp_max;
                memcpy(resp, &uart6_rx_buffer[data_off], copy);
                return copy;
              } else {
                stage = 0;
                frame_start = -1;
                continue;
              }
            }
          }
        }
      }
      start_index = uart6_rx_index;
    }
    vTaskDelay(10);
  }
  return 0;
}

// 发送帧
static int pn532_send_frame_quiet(UART_HandleTypeDef *huart, const uint8_t *tfi_data, int tfi_len) {
  uint8_t frame[64];
  int idx = 0;
  uint8_t LEN = (uint8_t)tfi_len;
  uint8_t LCS = (uint8_t)(0x100 - LEN);
  uint16_t sum = 0;
  for (int i = 0; i < tfi_len; i++)
    sum += tfi_data[i];
  uint8_t DCS = (uint8_t)(0x100 - (sum & 0xFF));

  uint8_t d;
  while (HAL_UART_Receive(huart, &d, 1, 0) == HAL_OK) {}

  frame[idx++] = 0x00;
  frame[idx++] = 0x00;
  frame[idx++] = 0xFF;
  frame[idx++] = LEN;
  frame[idx++] = LCS;
  memcpy(&frame[idx], tfi_data, tfi_len);
  idx += tfi_len;
  frame[idx++] = DCS;
  frame[idx++] = 0x00;

  HAL_StatusTypeDef status = HAL_UART_Transmit(huart, frame, idx, 1000);
  return (status == HAL_OK) ? 1 : 0;
}

// 静默调用函数
int pn532_call_quiet(UART_HandleTypeDef *huart, uint8_t cmd, const uint8_t *params, int params_len, uint8_t *resp, int resp_max) {
  uint8_t tfi[32];
  int n = 0;
  tfi[n++] = 0xD4;
  tfi[n++] = cmd;
  if (params_len > 0) {
    memcpy(&tfi[n], params, params_len);
    n += params_len;
  }

  if (!pn532_send_frame_quiet(huart, tfi, n)) {
    return -1;
  }

  if (!recv_ack(huart, 200)) {
    vTaskDelay(1000);
    return 0;
  }

  int m = recv_response(huart, resp, resp_max, 500);
  return m;
}

// MIFARE Classic认证函数
int PN532_MifareClassicAuthenticate_UART(PN532 *pn532, uint8_t *uid, uint8_t uid_len, uint8_t block_number, uint8_t *key, uint8_t key_type) {
  uint8_t params[13];
  uint8_t t = (key_type == 0x61) ? 0x61 : 0x60;
  uint8_t uid4[4] = {0};

  if (uid_len >= 4) {
    uid4[0] = uid[0];
    uid4[1] = uid[1];
    uid4[2] = uid[2];
    uid4[3] = uid[3];
  }

  params[0] = 0x01;
  params[1] = t;
  params[2] = block_number;

  for (int i = 0; i < 6; i++) {
    params[3 + i] = key ? key[i] : 0xFF;
  }

  for (int i = 0; i < 4; i++) {
    params[9 + i] = uid4[i];
  }

  uint8_t resp[40] = {0};
  int n = pn532_call_quiet(&huart6, PN532_COMMAND_INDATAEXCHANGE, params, sizeof(params), resp, sizeof(resp));

  if (n >= 3 && resp[0] == 0xD5 && resp[1] == 0x41 && resp[2] == 0x00) {
    return PN532_STATUS_OK;
  }
  if (n >= 1 && resp[0] == 0x00) {
    return PN532_STATUS_OK;
  }

  return PN532_STATUS_ERROR;
}

// MIFARE Classic读取函数（带UID）
int PN532_MifareClassicReadWithUID_UART(PN532 *pn532, uint8_t block_number, uint8_t *data, uint8_t *uid, uint8_t uid_length) {
  uint8_t params[3];
  params[0] = 0x01;
  params[1] = 0x30;
  params[2] = block_number;

  uint8_t response[32];
  int result = pn532_call_quiet(&huart6, PN532_COMMAND_INDATAEXCHANGE, params, 3, response, sizeof(response));

  // PN532标准格式：D5 41 00 + 16字节数据
  if (result >= 3 && response[0] == 0xD5 && response[1] == 0x41 && response[2] == 0x00) {
    if (result >= 3 + 16) {
      for (int i = 0; i < 16; i++) {
        data[i] = response[3 + i];
      }
      return PN532_STATUS_OK;
    }
  }
  // 简化格式：00 + 16字节数据
  else if (result > 0 && response[0] == 0x00) {
    if (result >= 1 + 16) {
      for (int i = 0; i < 16; i++) {
        data[i] = response[1 + i];
      }
      return PN532_STATUS_OK;
    }
  }

  return PN532_STATUS_ERROR;
}
