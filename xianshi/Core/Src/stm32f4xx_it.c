/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern QueueHandle_t recordHandle; // Declare recordHandle as a QueueHandle_t
extern QueueHandle_t showHandle;   // Declare showHandle as a QueueHandle_t

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint8_t u2temp=0;
extern SemaphoreHandle_t xUartSemaphore;
extern volatile uint8_t uart_processing;
void u2_calculate(uint8_t data)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t record_index = 0; // 用于检测 "record"
  static uint8_t connect_index = 0; // 用于检测 "connect"
  static uint8_t show_index = 0; // 用于检测展示什么数据
  // 检测 "record"
  if (data == 'r' && record_index == 0)
  {
    record_index++;
  }
  else if (data == 'e' && record_index == 1)
  {
    record_index++;
  }
  else if (data == 'c' && record_index == 2)
  {
    record_index++;
  }
  else if (data == 'o' && record_index == 3)
  {
    record_index++;
  }
  else if (data == 'r' && record_index == 4)
  {
    record_index++;
  }
  else if (data == 'd' && record_index == 5)
  {
    record_index = 0;
    /* 向消息队列发送消息 */
    uint16_t msg = 1; // 消息内容（可以是任意值）
    if (recordHandle != NULL)
    {
      xQueueSendFromISR(recordHandle, &msg, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  {
    record_index = 0;
  }
  // 检测 "connect"
  if (data == 'c' && connect_index == 0)
  {
    connect_index++;
  }
  else if (data == 'o' && connect_index == 1)
  {
    connect_index++;
  }
  else if (data == 'n' && connect_index == 2)
  {
    connect_index++;
  }
  else if (data == 'n' && connect_index == 3)
  {
    connect_index++;
  }
  else if (data == 'e' && connect_index == 4)
  {
    connect_index++;
  }
  else if (data == 'c' && connect_index == 5)
  {
    connect_index++;
  }
  else if (data == 't' && connect_index == 6)
  {
    connect_index = 0;
    /* 释放信号量 */
    if (xUartSemaphore != NULL)
    {
      if(uart_processing == 0) // 仅当未在处理时才释放
      {
        xSemaphoreGiveFromISR(xUartSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        uart_processing = 1;
      }
    }
  }
  else
  {
    connect_index = 0;
  }
  // 检测 "show"
  if(data == 's' && show_index == 0)
  {
    show_index++;
  }
  else if(data<='16'&&data>'0'&&show_index==1)
  {
    // 处理 show[x] 的情况
    uint16_t msg = (uint16_t)data;
    if (showHandle != NULL)
    {
      xQueueSendFromISR(showHandle, &msg, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else
  {
    show_index = 0;
  }
}


/* 推荐修复：不要把同一个接收缓冲 u2temp 用在 USART1/USART2 上，避免冲突 */
static uint8_t u1temp = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    u2_calculate(u2temp);
    HAL_UART_Receive_IT(&huart2,&u2temp,1); //开启下一次接收
  }
  else if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1,&u1temp,1); // 使用独立缓冲
  }
}
/* USER CODE END 1 */
