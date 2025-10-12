/**************************************************************************
 *  @file     pn532.h
 *  @author   Yehui from Waveshare
 *  @license  BSD
 *
 *  Header file for pn532.c
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

#ifndef PN532_H
#define PN532_H

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PN532帧格式定义
#define PN532_PREAMBLE (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

// PN532通信方向
#define PN532_HOSTTOPN532 (0xD4)
#define PN532_PN532TOHOST (0xD5)

// 实际使用的PN532命令
#define PN532_COMMAND_GETFIRMWAREVERSION (0x02)
#define PN532_COMMAND_SAMCONFIGURATION (0x14)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
#define PN532_COMMAND_INDATAEXCHANGE (0x40)

// PN532响应
#define PN532_RESPONSE_INDATAEXCHANGE (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET (0x4B)

// MIFARE Classic相关
#define PN532_MIFARE_ISO14443A (0x00)

// MIFARE命令
#define MIFARE_CMD_AUTH_A (0x60)
#define MIFARE_CMD_AUTH_B (0x61)
#define MIFARE_CMD_READ (0x30)
#define MIFARE_CMD_WRITE (0xA0)

// 状态定义
#define PN532_STATUS_ERROR (-1)
#define PN532_STATUS_OK (0)

// PN532设备结构体
typedef struct _PN532 {
  UART_HandleTypeDef *uart_handle; // UART句柄
  int (*reset)(void);
  int (*read_data)(uint8_t *data, uint16_t count);
  int (*write_data)(uint8_t *data, uint16_t count);
  bool (*wait_ready)(uint32_t timeout);
  int (*wakeup)(void);
  void (*log)(const char *log);
} PN532;

// 实际使用的函数声明
int PN532_WriteFrame(PN532 *pn532, uint8_t *data, uint16_t length);
int PN532_ReadFrame(PN532 *pn532, uint8_t *buff, uint16_t length);
int PN532_CallFunction(PN532 *pn532, uint8_t command, uint8_t *response,
                       uint16_t response_length, uint8_t *params,
                       uint16_t params_length, uint32_t timeout);

// UART专用函数接口
int PN532_UART_WriteData(PN532 *pn532, uint8_t *data, uint16_t length);
int PN532_UART_ReadData(PN532 *pn532, uint8_t *buff, uint16_t length);
void PN532_Wakeup(PN532 *pn532);
int PN532_ReadPassiveTarget_MFOC(PN532 *pn532, uint8_t *uid, uint8_t card_baud,
                                 uint32_t timeout);
int PN532_MifareClassicAuthenticate_UART(PN532 *pn532, uint8_t *uid,
                                         uint8_t uid_len, uint8_t block_number,
                                         uint8_t *key, uint8_t key_type);
int PN532_MifareClassicRead_UART(PN532 *pn532, uint8_t block_number,
                                 uint8_t *data);
int PN532_MifareClassicReadWithUID_UART(PN532 *pn532, uint8_t block_number,
                                        uint8_t *data, uint8_t *uid,
                                        uint8_t uid_length);
int PN532_MFOC_Test(PN532 *pn532);

// 新的UART命令发送和UID解析函数
int pn532_write_command(uint8_t cmd, uint8_t *params, uint16_t params_len,
                        uint8_t *response, uint16_t resp_max_len,
                        uint32_t timeout);
int parse_mifare_uid(uint8_t *response, uint16_t resp_len, uint8_t *uid,
                     uint8_t *uid_len);

// 静默通信函数（无调试输出）
int pn532_call_quiet(UART_HandleTypeDef *huart, uint8_t cmd,
                     const uint8_t *params, int params_len, uint8_t *resp,
                     int resp_max);

// SAM配置函数
int pn532_sam_configuration(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* PN532_H */
