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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart2.h"
#include <math.h>
#include "record.h"
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
volatile uint8_t uart_processing = 0; // 0: idle, 1: processing
uint8_t warn_flag = 0;       // 0: no warn, 1: warn
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId uart1Handle;
osThreadId myTask03Handle;
osThreadId run_showHandle;
osThreadId WarnHandle;
osThreadId recordShowHandle;
osMessageQId recordHandle;
osMessageQId showHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
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

  /* Create the queue(s) */
  /* definition and creation of record */
  osMessageQDef(record, 1, uint16_t);
  recordHandle = osMessageCreate(osMessageQ(record), NULL);

  /* definition and creation of show */
  osMessageQDef(show, 1, uint16_t);
  showHandle = osMessageCreate(osMessageQ(show), NULL);

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

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of run_show */
  osThreadDef(run_show, StartTask04, osPriorityIdle, 0, 1024);
  run_showHandle = osThreadCreate(osThread(run_show), NULL);

  /* definition and creation of Warn */
  osThreadDef(Warn, StartTask05, osPriorityIdle, 0, 128);
  WarnHandle = osThreadCreate(osThread(Warn), NULL);

  /* definition and creation of recordShow */
  osThreadDef(recordShow, StartTask06, osPriorityIdle, 0, 1024);
  recordShowHandle = osThreadCreate(osThread(recordShow), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  for (;;)
  {
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

/*
 * 语法：add main.s0.id,0,%d\xff\xff\xff，show[x]    (y=show[x])
 * 连接成功后，向 uart2 发送数据，更新界面
 * 屏幕的x=50<-->ch=128           y=30<-->cps=100
 * 屏幕的x=100<-->ch=256          y=65<-->cps=200
 * 屏幕的x=150<-->ch=512          y=98<-->cps=300
 * 屏幕的x=200<-->ch=768          y=130<-->cps=400
 * 屏幕的x=250<-->ch=1024
 * 屏幕的x=300<-->ch=1280
 * 屏幕的x=350<-->ch=1536
 * 屏幕的x=400<-->ch=1792
 */

/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  xUartSemaphore = xSemaphoreCreateBinary();
  configASSERT(xUartSemaphore);
  // 如果创建时信号量默认为"已给"，这里清除一次，确保从空状态开始
  xSemaphoreTake(xUartSemaphore, 0);
  printf("Hello FreeRTOS\r\n");
  u2printf("rest\xff\xff\xff");
  // printf("rest\xff\xff\xff");
  /* Infinite loop */
  for (;;)
  {
    if (xSemaphoreTake(xUartSemaphore, portMAX_DELAY) == pdTRUE)
    {
      u2printf("main.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("road.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("record.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("setting.t2.txt=\"connect\"\xff\xff\xff");
      // co57_show();
      // co60_show();
      // co57_show_char();
      // co60_show_char();
      printf("Get Semaphore\r\n");
      /* 处理完成，允许再次响应 connect */
      uart_processing = 0;
    }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  // float angle = 0.0f;  // 角度 (float)
  // int speed = 0;       // 速度 (int)
  // float time = 0.0f;   // 时间变量，用于生成曲线
  // const float step = 0.1f; // 时间步长 (秒)
  // const float angular_velocity = 10.0f; // 角速度 (度/秒)
  // const float radius = 1.0f; // 圆的半径 (单位长度)
  // char buffer[36];
  /* Infinite loop */
  for (;;)
  {
    // // 计算角度 (0 到 360 度)
    // angle = fmod(time * angular_velocity, 360.0f);

    // // 计算速度 (线速度)
    // speed = (int)(radius * angular_velocity);

    // // 通过 u2printf 发送数据
    // snprintf(buffer, sizeof(buffer), "#angle=%.2f,speed=%d$\r\n", angle, speed);
    // u2printf(buffer);

    // // 增加时间，形成曲线
    // time += step;

    // // 延时 100ms，控制发送频率
    osDelay(10);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */

/*
 * 每次进入行驶路径界面自动打印数组，数组的0，0坐标在屏幕左上角
 * 语法：line x0,y0,x1,y1,color
 * 例子：line 10,460,50,0xff00
 * 画一条点（10,460）到点（50,240）的红色线段
 * 屏幕因控件优先级更高的问题，实际可显示的区域为
 *  *(0，70）-----------------------------------*(480,70)
 *  |                                               |
 *  |                                               |
 *  |       屏幕可显示区域                           |
 *  |                                               |
 *  |                                               |
 *  |                                               |
 *  *(0,270)------------------------------------*(480,270)
 */

/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  char buffer[100];
  uint16_t msg; // 用于接收消息
  /* Infinite loop */
  for (;;)
  {
    // 等待消息队列
    if (xQueueReceive(recordHandle, &msg, portMAX_DELAY) == pdTRUE)
    {
      // 收到消息后执行打印功能
      osDelay(200); // 确保界面已经切换到 record 页面
      snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", 0, 70, 480, 70, 0);
      u2printf(buffer);
      printf("Record Command Sent\r\n");
    }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Warn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  uint8_t i = 0;
  /* Infinite loop */
  for(;;)
  {
    if(i++ > 5 && warn_flag == 0) // 每5秒检查一次警告状态
    {
      i = 0;
      warn_flag = 1; // 假设有警告
    }
    else if(i++ > 5 && warn_flag == 1)
    {
      warn_flag = 0; // 清除警告标志
      i=0;
    }
    if(warn_flag == 1)
    {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
    osDelay(500);
    }
    else 
    {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
      osDelay(1000);
    }
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the recordShow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  uint16_t msg; // 用于接收消息
  /* Infinite loop */
  for(;;)
  {
    // 等待消息队列
    if (xQueueReceive(showHandle, &msg, portMAX_DELAY) == pdTRUE)
    {
      // 收到消息后执行打印功能
      osDelay(500); // 确保界面已经切换到 main 页面
      
      u2printf("cle s0.id,255\xff\xff\xff"); // 清除 id 显示区域
      if(msg=='1')
      {
        record_show_name(co57_record);
        record_show(co57_record);
        record_show_char(co57_record);
      }
      else if(msg=='2')
      {
        record_show_name(co60_record);
        record_show(co60_record);
        record_show_char(co60_record);
      }
      osDelay(1000);
    }
  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
