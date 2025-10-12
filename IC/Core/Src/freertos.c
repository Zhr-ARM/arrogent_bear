/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "monitor.h"
#include "pn532_task.h"
#include "pn532.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "uart2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SemaphoreHandle_t xUartSemaphore;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId uart1Handle;
osThreadId UART4_monitorHandle;
osThreadId US3_JY91Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart1 */
  osThreadDef(uart1, StartTask02, osPriorityIdle, 0, 512);
  uart1Handle = osThreadCreate(osThread(uart1), NULL);

  /* definition and creation of UART4_monitor */
  osThreadDef(UART4_monitor, StartTask03, osPriorityIdle, 0, 128);
  UART4_monitorHandle = osThreadCreate(osThread(UART4_monitor), NULL);

  /* definition and creation of US3_JY91 */
  osThreadDef(US3_JY91, StartTask04, osPriorityIdle, 0, 128);
  US3_JY91Handle = osThreadCreate(osThread(US3_JY91), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* definition and creation of PN532_ICCardTask */
  osThreadDef(PN532_ICCardTask, StartPN532_ICCardTask, osPriorityHigh, 0, 1024);
  osThreadCreate(osThread(PN532_ICCardTask), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the uart1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  //  uint8_t show[450]={0};
  //  show[50]=20;
  //  show[100]=40;
  //  show[150]=60;
  //  show[250]=175;
  //  xUartSemaphore = xSemaphoreCreateBinary();
  //  configASSERT(xUartSemaphore);
  //  printf("Hello FreeRTOS\r\n");
  //  u2printf("rest\xff\xff\xff");
  /* Infinite loop */
  for (;;) {
    //    if (xSemaphoreTake(xUartSemaphore, portMAX_DELAY) == pdTRUE)
    //    {
    //      u2printf("main.t2.txt=\"connect\"\xff\xff\xff");
    //      u2printf("road.t2.txt=\"connect\"\xff\xff\xff");
    //      u2printf("record.t2.txt=\"connect\"\xff\xff\xff");
    //      u2printf("setting.t2.txt=\"connect\"\xff\xff\xff");
    //      for (int i = 0; i < 450; i++)
    //      {
    //        // 向曲线s0的通道0传输1个数据,add指令不支持跨页面
    //        char buffer[32];
    //        snprintf(buffer, sizeof(buffer), "add
    //        main.s0.id,0,%d\xff\xff\xff", show[i]); u2printf(buffer);
    //      }
    //      printf("Get Semaphore\r\n");
    //    }
    osDelay(50);
  }
  
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the UART4_monitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  // 注释掉其他功能，只保留UART4

  // Infinite loop
  for (;;) {
    //    my_printf(&huart4, "f_roll:	%f\r\n", f_roll);
    //    my_printf(&huart4, "f_pitch:	%f\r\n", f_pitch);
    //    my_printf(&huart4, "f_yaw:		%f\r\n", f_yaw);

    //    my_printf(&huart4, "------------------------\r\n");

    //    osDelay(1000);
  }
osDelay(2000);  // 等待系统稳定

  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the US3_JY91 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;) {
    Uart3_Parse();

    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE BEGIN Header_StartPN532_ICCardTask */
/**
 * @brief Function implementing the PN532_ICCard thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPN532_ICCardTask */


/* USER CODE END Application */
