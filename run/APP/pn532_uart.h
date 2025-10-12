#ifndef PN532_UART_H
#define PN532_UART_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// UART设备结构
typedef struct {
  UART_HandleTypeDef *uart_handle;
  GPIO_TypeDef *rst_port;
  uint16_t rst_pin;
} PN532_Device_UART;

// UART模式函数声明
PN532_Device_UART PN532_CreateDevice_UART(void);
void PN532_Init_UART(PN532_Device_UART *dev);
int PN532_Reset_UART(PN532_Device_UART *dev);
void PN532_Wakeup_UART(PN532_Device_UART *dev);
bool PN532_WaitReady_UART(PN532_Device_UART *dev, uint32_t timeout);
int PN532_WriteData_UART(PN532_Device_UART *dev, uint8_t *data,
                         uint16_t length);
int PN532_ReadData_UART(PN532_Device_UART *dev, uint8_t *buffer,
                        uint16_t length);

// 新增的帧格式函数
int PN532_SendFrame_UART(PN532_Device_UART *dev, uint8_t *data,
                         uint16_t length);
int PN532_ReadFrame_UART(PN532_Device_UART *dev, uint8_t *buffer,
                         uint16_t max_length);

// 按照文章逻辑的通用函数
void PN532_SendFrame(uint8_t *frame, uint16_t len);
int PN532_ReceiveResponse(uint8_t *buf, uint16_t max_len, uint32_t timeout);

// PN532通用函数(UART版本)
int PN532_GetFirmwareVersion_UART(PN532_Device_UART *dev, uint8_t *version);
int PN532_SamConfiguration_UART(PN532_Device_UART *dev);
int PN532_ReadPassiveTarget_UART(PN532_Device_UART *dev, uint8_t *uid,
                                 uint8_t *uid_len);
int PN532_MifareClassicAuthenticate_UART(PN532_Device_UART *dev, uint8_t *uid,
                                         uint8_t uid_len, uint8_t block_num,
                                         uint8_t *key);
int PN532_MifareClassicRead_UART(PN532_Device_UART *dev, uint8_t *uid,
                                 uint8_t uid_len, uint8_t block_num,
                                 uint8_t *data);

#endif
