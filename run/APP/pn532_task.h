/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : pn532_task.h
 * @brief          : PN532 IC Card Reader Task Interface
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __PN532_TASK_H
#define __PN532_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "pn532.h"  // 在文件顶部添加

/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint8_t uid[10];                // 卡片UID
  uint8_t uid_length;             // UID长度
  uint8_t sector_data[16][4][16]; // 扇区数据 [扇区][块][字节]
  bool success;                   // 读取是否成功
  uint32_t timestamp;             // 读取时间戳
} PN532_ReadResult;
/* 峰位信息结构体 */
typedef struct {
    uint8_t peak1_channel;      // 第一个峰位道址
    uint8_t peak2_channel;      // 第二个峰位道址
    uint8_t peak1_data[3];      // 第一个峰位的3字节数据
    uint8_t peak2_data[3];      // 第二个峰位的3字节数据
    bool valid;                 // 数据是否有效
} PeakInfo;



/* 单张卡的有效数据结构 */
typedef struct {
    uint16_t peak_channels[2];    // 峰位道址（最多2个）
    uint8_t peak_spectrums[2][3]; // 能谱数据（每个峰3字节）
    uint8_t valid_peak_count;     // 有效峰位数量（1或2）
    uint32_t timestamp;           // 读取时间戳
    uint8_t uid[7];              // 卡片UID
    uint8_t uid_length;          // UID长度
} CardDataRecord;

/* 卡片数据循环缓冲区 */
typedef struct {
    CardDataRecord records[12];   // 最多存储12张卡
    uint8_t write_index;          // 写入位置（0-11）
    uint8_t count;                // 当前存储数量（0-12）
} CardDataBuffer;

/* 全局缓冲区声明 */
extern CardDataBuffer g_card_data_buffer;

/* 缓冲区操作函数 */
void CardBuffer_Init(void);
bool CardBuffer_AddRecord(const PeakInfo *peak_info, const uint8_t *uid, uint8_t uid_length);
bool CardBuffer_GetRecord(uint8_t index, CardDataRecord *record);
uint8_t CardBuffer_GetCount(void);
void CardBuffer_Clear(void);
void CardBuffer_Print(void);

// 卡片缓冲区打印函数
void CardBuffer_PrintToUART4(void);           // 格式化打印
void CardBuffer_PrintRawDataToUART4(void);    // 原始内存打印


/* 全局变量声明 */
extern PeakInfo g_peak_info;
extern UART_HandleTypeDef huart4; // 调试串口
extern UART_HandleTypeDef huart6; // PN532通信串口
extern uint16_t uart6_rx_index;
extern uint8_t uart6_rx_buffer[256];

/* 函数声明 */
bool PN532_ReadPeakInfoSelective(PN532 *pn532, uint8_t *uid, uint8_t uid_length, PeakInfo *peak_info);

/* Exported constants --------------------------------------------------------*/
#define PN532_TASK_PRIORITY (osPriorityNormal)
#define PN532_TASK_STACK_SIZE (512)

/* Exported variables --------------------------------------------------------*/
extern osThreadId PN532_ICCardTaskHandle;
extern PN532_ReadResult g_last_read_result;
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void const * argument);
void StartPN532_ICCardTask(void const * argument);  // Add this line
/* USER CODE END FunctionPrototypes */


/* 块1和块2数据变量 */
extern uint8_t g_block1_array[4];   // 块1数组（只保留非零数据）
extern int g_block1_array_size;     // 块1数组大小
extern uint8_t g_block2_first_byte; // 块2第一个字节

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 启动PN532 IC卡读取任务
 * @return true=启动成功, false=启动失败
 */
bool PN532_StartReadTask(void);

/**
 * @brief 获取块1数组数据
 * @param array 输出参数，用于存储块1数组数据
 * @param size 输出参数，用于存储数组大小
 * @return true=数据有效, false=数据无效
 */
bool PN532_GetBlock1Array(uint8_t *array, int *size);

/**
 * @brief 获取块2第一个字节
 * @return 块2第一个字节的十进制值，如果数据无效返回0
 */
uint8_t PN532_GetBlock2FirstByte(void);




#ifdef __cplusplus
}
#endif

#endif /* __PN532_TASK_H */
