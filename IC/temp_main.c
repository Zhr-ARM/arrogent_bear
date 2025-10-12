/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "monitor.h"
#include "mydefine.h"
#include "pn532.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PN532非阻塞接收全局变量

// USART6标准中断接收变量
uint32_t uart6_rx_ticks = 0;        // 最后接收时�?uint16_t uart6_rx_index = 0;        // 接收计数�?uint8_t uart6_rx_buffer[256] = {0}; // 接收缓冲�?volatile uint8_t uart6_rx_flag = 0; // 接收完成标志
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void uart6_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART6任务处理函数
void uart6_task(void) {
  // 1. 检查货架：如果计数器为0，说明没货或刚处理完，休息�?  if (uart6_rx_index == 0)
    return;

  // 2. 检查手表：当前时间 - 最后收货时�?> 规定的超时时间？
  if (HAL_GetTick() - uart6_rx_ticks > 200) // 增加�?00ms超时
  {
    // --- 3. 超时！开始理�?---
    // 检查是否收到完整帧（PN532帧格式：00 00 FF LEN LCS ... DCS 00�?    bool frame_complete = false;

    // 检查PN532帧格�?    if (uart6_rx_index >= 6) { // 最小帧长度
      // 检查前导码 00 00 FF
      if (uart6_rx_buffer[0] == 0x00 && uart6_rx_buffer[1] == 0x00 &&
          uart6_rx_buffer[2] == 0xFF) {
        // 检查后导码 00
        if (uart6_rx_buffer[uart6_rx_index - 1] == 0x00) {
          frame_complete = true;
        }
      }
    }

    // 如果收到完整帧或超时，处理数�?    if (frame_complete || uart6_rx_index > 0) {
      HAL_UART_Transmit(&huart4, (uint8_t *)"UART6 received: ", 16, 1000);
      for (int i = 0; i < uart6_rx_index; i++) {
        char hex[5];
        sprintf(hex, "%02X ", uart6_rx_buffer[i]);
        HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
      }
      HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

      // 检查ACK响应
      if (uart6_rx_index >= 6) {
        uint8_t expected_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
        if (memcmp(uart6_rx_buffer, expected_ack, 6) == 0) {
          HAL_UART_Transmit(&huart4, (uint8_t *)"Perfect ACK received!\r\n", 22,
                            1000);
        }
      }
    }

    // --- 理货结束 ---

    // 4. 清理现场：把处理完的货从货架上拿走，计数器归�?    memset(uart6_rx_buffer, 0, uart6_rx_index);
    uart6_rx_index = 0;

    // 5. 将UART接收缓冲区指针重置为接收缓冲区的起始位置
    huart6.pRxBuffPtr = uart6_rx_buffer;
  }
  // 如果没超时，啥也不做，等下次再检�?}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_TIM11_Init();
  MX_TIM14_Init();

  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t temp;
  HAL_UART_Receive_IT(&huart2, &temp, 1);               // 开启第一次接�?  HAL_UART_Receive_IT(&huart1, &temp, 1);               // 开启下一次接�?  HAL_UART_Receive_IT(&huart3, &Recv3, 1);              // 开启下一次接�?  HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer[0], 1); // 开启USART6接收中断

  // PN532 IC Card Reader - Now handled by FreeRTOS task
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n=== PN532 IC Card Reader ===\r\n",
                    33, 1000);
  HAL_UART_Transmit(
      &huart4,
      (uint8_t *)"PN532 operations are now handled by FreeRTOS task.\r\n", 48,
      1000);
  HAL_UART_Transmit(
      &huart4, (uint8_t *)"Check UART4 output for card reading results.\r\n",
      44, 1000);

  HAL_UART_Transmit(&huart4,
                    (uint8_t *)"Entering FreeRTOS for card reading...\r\n", 38,
                    1000);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
