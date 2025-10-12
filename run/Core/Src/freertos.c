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
#include "mydefine.h"
#include "uart2.h"
#include <math.h>
#include "record.h"
#include "simple_path_tracker.h"
#include "pn532_task.h"
#include "pn532.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SemaphoreHandle_t xUartSemaphore;
extern MotorDriver_t car;
extern PID_T pid_speed_A;  // A轮速度环
extern PID_T pid_speed_B;  // B轮速度环
extern PID_T pid_speed_C;  // C轮速度环
extern PID_T pid_speed_D;  // D轮速度环

SPT_Tracker g_tracker; 
SPT_Tracker g_shortest;
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
volatile uint8_t system_started = 0;  // 0: 停止, 1: 启动、
volatile uint8_t final_flag=0;  // 0: 否, 1: 终点停车
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId uart1Handle;
osThreadId UART4_monitorHandle;
osThreadId US3_JY91Handle;
osThreadId encoderHandle;
osThreadId run_motorHandle;
osThreadId rho_getHandle;
osThreadId run_taskHandle;
osThreadId run_recordHandle;
osThreadId RGBwarnHandle;
osThreadId recordsHandle;
osThreadId record_calHandle;
osThreadId hisHandle;
osMessageQId recordHandle;
osMessageQId showHandle;
osMessageQId historyHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Connect(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);
void StartTask07(void const * argument);
void StartTask08(void const * argument);
void run_show(void const * argument);
void Warn(void const * argument);
void recordShow(void const * argument);
void records_cal(void const * argument);
void history_show(void const * argument);

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

  /* definition and creation of history */
  osMessageQDef(history, 1, uint16_t);
  historyHandle = osMessageCreate(osMessageQ(history), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uart1 */
  osThreadDef(uart1, Connect, osPriorityIdle, 0, 256);
  uart1Handle = osThreadCreate(osThread(uart1), NULL);

  /* definition and creation of UART4_monitor */
  osThreadDef(UART4_monitor, StartTask03, osPriorityNormal, 0, 512);
  UART4_monitorHandle = osThreadCreate(osThread(UART4_monitor), NULL);

  /* definition and creation of US3_JY91 */
  osThreadDef(US3_JY91, StartTask04, osPriorityIdle, 0, 128);
  US3_JY91Handle = osThreadCreate(osThread(US3_JY91), NULL);

  /* definition and creation of encoder */
  osThreadDef(encoder, StartTask05, osPriorityIdle, 0, 128);
  encoderHandle = osThreadCreate(osThread(encoder), NULL);

  /* definition and creation of run_motor */
  osThreadDef(run_motor, StartTask06, osPriorityIdle, 0, 512);
  run_motorHandle = osThreadCreate(osThread(run_motor), NULL);

  /* definition and creation of rho_get */
  osThreadDef(rho_get, StartTask07, osPriorityIdle, 0, 128);
  rho_getHandle = osThreadCreate(osThread(rho_get), NULL);

  /* definition and creation of run_task */
  osThreadDef(run_task, StartTask08, osPriorityIdle, 0, 512);
  run_taskHandle = osThreadCreate(osThread(run_task), NULL);

  /* definition and creation of run_record */
  osThreadDef(run_record, run_show, osPriorityRealtime, 0, 1024);
  run_recordHandle = osThreadCreate(osThread(run_record), NULL);

  /* definition and creation of RGBwarn */
  osThreadDef(RGBwarn, Warn, osPriorityIdle, 0, 128);
  RGBwarnHandle = osThreadCreate(osThread(RGBwarn), NULL);

  /* definition and creation of records */
  osThreadDef(records, recordShow, osPriorityIdle, 0, 1024);
  recordsHandle = osThreadCreate(osThread(records), NULL);

  /* definition and creation of record_cal */
  osThreadDef(record_cal, records_cal, osPriorityRealtime, 0, 1024);
  record_calHandle = osThreadCreate(osThread(record_cal), NULL);

  /* definition and creation of his */
  osThreadDef(his, history_show, osPriorityIdle, 0, 512);
  hisHandle = osThreadCreate(osThread(his), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
//LED闪烁任务,用于检测任务有无卡住
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
    osDelay(250);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
    osDelay(250);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Connect */
/**
* @brief Function implementing the uart1 thread.
* @param argument: Not used
* @retval None
*/
/*
* 用于第一次与串口屏通信，判断是否连接成功
* 连接成功后，发送初始化命令，更新界面显示
*/
/* USER CODE END Header_Connect */
void Connect(void const * argument)
{
  /* USER CODE BEGIN Connect */
  xUartSemaphore = xSemaphoreCreateBinary();
  configASSERT(xUartSemaphore);
  // 如果创建时信号量默认为"已给"，这里清除一次，确保从空状态开始
  xSemaphoreTake(xUartSemaphore, 0);
  printf("Hello FreeRTOS\r\n");
  u2printf("rest\xff\xff\xff");
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(xUartSemaphore, portMAX_DELAY) == pdTRUE)
    {
      u2printf("main.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("road.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("record.t2.txt=\"connect\"\xff\xff\xff");
      u2printf("setting.t2.txt=\"connect\"\xff\xff\xff");

      // 启动小车系统
      if (system_started == 0) {
        system_started = 1;       
      }      

      printf("Get Semaphore\r\n");
      /* 处理完成，允许再次响应 connect */
      uart_processing = 0;
    }
  }
  /* USER CODE END Connect */
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
  /* Infinite loop */
  for(;;)
  {
//		my_printf(&huart4,"f_roll:	%f\r\n"	,	f_roll);
//		my_printf(&huart4,"f_pitch:	%f\r\n"	,	f_pitch);
//		my_printf(&huart4,"f_yaw:		%f\r\n"	,	f_yaw);
//		my_printf(&huart4,"------------------------\r\n");
//		
//		my_printf(&huart4,"Left:%.2f Right:%.2f\r\n",B_encoder.speed_cm_s,A_encoder.speed_cm_s);
//		my_printf(&huart4,"Left:%.2f Right:%.2f\r\n",C_encoder.speed_cm_s,D_encoder.speed_cm_s);
//		my_printf(&huart4,"------------------------\r\n");
//		
//		my_printf(&huart4,"f_rho:	%f\r\n"	,	f_rho);
//		my_printf(&huart4,"f_cross:%f\r\n",	f_cross);
//		my_printf(&huart4,"------------------------\r\n");
		
//		my_printf(&huart4,"{A_filtered}%.2f,%.2f\r\n", pid_speed_A.target, A_encoder.speed_cm_s);
//		my_printf(&huart4,"{B_filtered}%.2f,%.2f\r\n", pid_speed_B.target, B_encoder.speed_cm_s);
//		my_printf(&huart4,"{C_filtered}%.2f,%.2f\r\n", pid_speed_C.target, C_encoder.speed_cm_s);
//		my_printf(&huart4,"{D_filtered}%.2f,%.2f\r\n", pid_speed_D.target, D_encoder.speed_cm_s);
		
    // my_printf(&huart4,"#%f,%d$"	,	f_yaw,(int)((A_encoder.speed_cm_s+B_encoder.speed_cm_s+C_encoder.speed_cm_s+D_encoder.speed_cm_s)/4.0f));
    //my_printf(&huart4,"(%d,%d);",g_tracker.points[g_tracker.length-1].grid_x,g_tracker.points[g_tracker.length-1].grid_y);
    osDelay(100);
  }
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
  for(;;)
  {
		Uart3_Parse();
		
    osDelay(5);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
		Encoder_Task();		
		
    osDelay(10);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the run_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
		//电机驱动测试 
		PID_Task();
		
    osDelay(5);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the rho_get thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void const * argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
		RHO_Parse();
		
    osDelay(5);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the run_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void const * argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
    // 只有在系统启动时才执行路径规划
    if(system_started) {
      PathPlanning_Update(f_cross);
    }
			
    osDelay(10);
  }
  /* USER CODE END StartTask08 */
}

/* USER CODE BEGIN Header_run_show */
/**
* @brief Function implementing the run_record thread.
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

/* USER CODE END Header_run_show */
void run_show(void const * argument)
{
  /* USER CODE BEGIN run_show */
  uint16_t msg; // 用于接收消息
  /* Infinite loop */
  for(;;)
  {
     // 等待消息队列
    if (xQueueReceive(recordHandle, &msg, portMAX_DELAY) == pdTRUE)
    {
      // 收到消息后执行打印功能
      osDelay(200); // 确保界面已经切换到 record 页面
      g_shortest.length = 0;
      SPT_BuildShortestPathFromHistory(g_tracker.points, g_tracker.length, &g_shortest, g_tracker.x_cm, g_tracker.y_cm);
      road_show(g_shortest.points, g_tracker.points, g_shortest.length, g_tracker.length);
      printf("Record Command Sent\r\n");
			// snprintf(buffer, sizeof(buffer), "line %d,%d,%d,%d,%d\xff\xff\xff", 0, 70, 480, 70, 0);
      // u2printf(buffer);
    }
  }
  /* USER CODE END run_show */
}

/* USER CODE BEGIN Header_Warn */
/**
* @brief Function implementing the RGBwarn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Warn */
void Warn(void const * argument)
{
  /* USER CODE BEGIN Warn */
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
		//识别到IC卡，停止小车
		system_started = 0;
		MotorDriver_StopAll(&car);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    osDelay(500);

 		if(!final_flag){
		//结束报警，启动小车
		system_started = 1;
		}   
    }
    else 
    {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
      osDelay(1000);
    }
  }
  /* USER CODE END Warn */
}

/* USER CODE BEGIN Header_recordShow */
/**
* @brief Function implementing the records thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_recordShow */
void recordShow(void const * argument)
{
  /* USER CODE BEGIN recordShow */
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
  /* USER CODE END recordShow */
}

/* USER CODE BEGIN Header_records_cal */
/**
* @brief Function implementing the record_cal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_records_cal */
void records_cal(void const * argument)
{
  /* USER CODE BEGIN records_cal */
  SPT_Init(&g_tracker);
  /* Infinite loop */
  for(;;)
  {
    int sp = (int)((A_encoder.speed_cm_s+B_encoder.speed_cm_s+C_encoder.speed_cm_s+D_encoder.speed_cm_s)/4.0f);
    float yaw = f_yaw;
    SPT_Update(&g_tracker, sp, yaw);
    osDelay(10);
  }
  /* USER CODE END records_cal */
}

/* USER CODE BEGIN Header_history_show */
/**
* @brief Function implementing the his thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_history_show */
void history_show(void const * argument)
{
  /* USER CODE BEGIN history_show */
  uint16_t msg; // 用于接收消息
  /* Infinite loop */
  for(;;)
  {
    // 等待消息队列
    if (xQueueReceive(historyHandle, &msg, portMAX_DELAY) == pdTRUE)
    { 
      if(msg==1)
      {
        history_show1_record(co57_record,record_num);
      }
      else if(msg==2)
      {
        history_show2_record(co60_record,record_num);
      }
      osDelay(1000);
    }
  }
  /* USER CODE END history_show */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
