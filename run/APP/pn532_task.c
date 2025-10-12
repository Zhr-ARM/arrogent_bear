/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : pn532_task.c
 * @brief          : PN532 IC Card Reader Task Implementation
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pn532_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "pn532.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
osThreadId PN532_ICCardTaskHandle = NULL;
PN532_ReadResult g_last_read_result = {0};
static bool pn532_initialized = false;
/* 峰位信息全局变量 */
PeakInfo g_peak_info = {0};

/* 块1和块2数据变量 */
uint8_t g_block1_array[4] = {0}; // 块1数组（只保留非零数据）
int g_block1_array_size = 0;     // 块1数组大小
uint8_t g_block2_first_byte = 0; // 块2第一个字节

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart4; // 调试串口
extern UART_HandleTypeDef huart6; // PN532通信串口
extern uint16_t uart6_rx_index;
extern uint8_t uart6_rx_buffer[256];

/* Private function prototypes -----------------------------------------------*/
static void PN532_Init_Hardware(void);
static bool PN532_SearchCard(uint8_t *uid, uint8_t *uid_length);
static bool PN532_ReadAllSectors(PN532 *pn532, PN532_ReadResult *result);

/* ============================================================================
   核心读卡函数（同步执行）
   ============================================================================
 */
/**
 * @brief 根据道址计算扇区、块号和字节偏移
 */
static bool CalculateAddress(uint16_t channel, uint8_t *sector, uint8_t *block, uint8_t *byte_offset) {
    if (channel >= 720) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 道址超出范围\r\n", 24, 1000);
        return false;
    }
    
    // 每个扇区有48字节 (3块 × 16字节)
    *sector = (channel / 48) + 1;  // 扇区1-15
    
    uint16_t offset_in_sector = channel % 48;
    *block = offset_in_sector / 16;  // 块0-2
    *byte_offset = offset_in_sector % 16;  // 字节0-15
    
    char msg[80];
    sprintf(msg, "[Peak] 道址%d -> 扇区%d 块%d 偏移%d\r\n", 
            channel, *sector, *block, *byte_offset);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    return true;
}

/**
 * @brief 读取指定块的数据（按需认证）
 */
static bool ReadBlockSelective(PN532 *pn532, uint8_t *uid, uint8_t uid_length,
                               uint8_t sector, uint8_t block, uint8_t *data) {
    uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t block_addr = sector * 4 + block;
    
    char msg[60];
    sprintf(msg, "[Peak] 认证扇区%d...\r\n", sector);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    // 认证扇区
    if (PN532_MifareClassicAuthenticate_UART(pn532, uid, uid_length, 
                                             sector * 4, key, 0x60) != PN532_STATUS_OK) {
        sprintf(msg, "[Peak] 扇区%d认证失败\r\n", sector);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
        return false;
    }
    
    osDelay(50);
    
    sprintf(msg, "[Peak] 读取块%d...\r\n", block_addr);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    // 读取块数据
    if (PN532_MifareClassicReadWithUID_UART(pn532, block_addr, data, uid, uid_length) != PN532_STATUS_OK) {
        sprintf(msg, "[Peak] 块%d读取失败\r\n", block_addr);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
        return false;
    }
    
    // 打印数据
    sprintf(msg, "[Peak] 块%d: ", block_addr);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    for (int i = 0; i < 16; i++) {
        char hex[5];
        sprintf(hex, "%02X ", data[i]);
        HAL_UART_Transmit(&huart4, (uint8_t*)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 1000);
    
    return true;
}

/**
 * @brief 从块数据中提取3字节峰值数据
 */
/**
 * @brief 从块数据中提取3字节峰值数据
 */
static bool ExtractPeakData(PN532 *pn532, uint8_t *uid, uint8_t uid_length,
                           uint8_t sector, uint8_t block, uint8_t byte_offset,
                           uint8_t *block_data, uint8_t *peak_data) {
    
    // 【修正】向前移动1位，从 byte_offset-1 开始提取
    if (byte_offset == 0) {
        // 特殊情况：需要从上一个块的最后一个字节开始
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 跨块：需要上一块\r\n", 27, 1000);
        
        uint8_t prev_sector = sector;
        uint8_t prev_block = block - 1;
        
        if (block == 0) {
            // 需要上一个扇区的块2
            if (sector == 1) {
                HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 超出范围\r\n", 18, 1000);
                return false;
            }
            prev_sector = sector - 1;
            prev_block = 2;
        }
        
        uint8_t prev_data[16];
        if (!ReadBlockSelective(pn532, uid, uid_length, prev_sector, prev_block, prev_data)) {
            return false;
        }
        
        peak_data[0] = prev_data[15];
        peak_data[1] = block_data[0];
        peak_data[2] = block_data[1];
        
    } else if (byte_offset == 1) {
        // 特殊情况：需要从上一块的最后1字节
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 跨块：需要上一块1字节\r\n", 33, 1000);
        
        uint8_t prev_sector = sector;
        uint8_t prev_block = block - 1;
        
        if (block == 0) {
            if (sector == 1) {
                HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 超出范围\r\n", 18, 1000);
                return false;
            }
            prev_sector = sector - 1;
            prev_block = 2;
        }
        
        uint8_t prev_data[16];
        if (!ReadBlockSelective(pn532, uid, uid_length, prev_sector, prev_block, prev_data)) {
            return false;
        }
        
        peak_data[0] = prev_data[15];
        peak_data[1] = block_data[0];
        peak_data[2] = block_data[1];
        
    } else if (byte_offset <= 14) {
        // 常规情况：3字节在同一块内，从 byte_offset-1 开始
        peak_data[0] = block_data[byte_offset - 1];
        peak_data[1] = block_data[byte_offset];
        peak_data[2] = block_data[byte_offset + 1];
        
    } else if (byte_offset == 15) {
        // 跨块：需要下一块的1字节
        peak_data[0] = block_data[14];
        peak_data[1] = block_data[15];
        
        uint8_t next_sector = (block == 2) ? sector + 1 : sector;
        uint8_t next_block = (block == 2) ? 0 : block + 1;
        
        if (next_sector > 15) {
            HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 超出范围\r\n", 18, 1000);
            return false;
        }
        
        uint8_t next_data[16];
        if (!ReadBlockSelective(pn532, uid, uid_length, next_sector, next_block, next_data)) {
            return false;
        }
        peak_data[2] = next_data[0];
    }
    
    char msg[100];
    sprintf(msg, "[Peak] 峰值: %02X %02X %02X (十进制: %u %u %u)\r\n",
            peak_data[0], peak_data[1], peak_data[2],
            peak_data[0], peak_data[1], peak_data[2]);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    return true;
}

/**
 * @brief 选择性读取峰位信息（只读必要的块）
 */
bool PN532_ReadPeakInfoSelective(PN532 *pn532, uint8_t *uid, uint8_t uid_length, PeakInfo *peak_info) {
    uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t block1_data[16];
    
    memset(peak_info, 0, sizeof(PeakInfo));
    
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n===== 读取峰位信息 =====\r\n", 30, 1000);
    
    // 步骤1：读取扇区0块1
    HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 读取扇区0块1...\r\n", 24, 1000);
    
    if (PN532_MifareClassicAuthenticate_UART(pn532, uid, uid_length, 0, key, 0x60) != PN532_STATUS_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 扇区0认证失败\r\n", 22, 1000);
        return false;
    }
    
    osDelay(50);
    
    if (PN532_MifareClassicReadWithUID_UART(pn532, 1, block1_data, uid, uid_length) != PN532_STATUS_OK) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 块1读取失败\r\n", 20, 1000);
        return false;
    }
    
    // 打印块1数据
    HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 块1: ", 12, 1000);
    for (int i = 0; i < 16; i++) {
        char hex[5];
        sprintf(hex, "%02X ", block1_data[i]);
        HAL_UART_Transmit(&huart4, (uint8_t*)hex, 3, 1000);
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 1000);
    
    // 【关键修正】从索引0和索引2读取峰位道址
    peak_info->peak1_channel = block1_data[0];  // 第1字节（索引0）
    peak_info->peak2_channel = block1_data[2];  // 第3字节（索引2）
    
    char msg[100];
    sprintf(msg, "[Peak] 峰位: 峰1=%d道(0x%02X), 峰2=%d道(0x%02X)\r\n", 
            peak_info->peak1_channel, peak_info->peak1_channel,
            peak_info->peak2_channel, peak_info->peak2_channel);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    osDelay(100);
    
    // 步骤2：读取峰1数据
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n[Peak] 读取峰1数据...\r\n", 26, 1000);
    
    uint8_t sector1, block1, offset1;
    if (!CalculateAddress(peak_info->peak1_channel, &sector1, &block1, &offset1)) {
        return false;
    }
    
    uint8_t peak1_block_data[16];
    if (!ReadBlockSelective(pn532, uid, uid_length, sector1, block1, peak1_block_data)) {
        return false;
    }
    
    if (!ExtractPeakData(pn532, uid, uid_length, sector1, block1, offset1, 
                        peak1_block_data, peak_info->peak1_data)) {
        return false;
    }
    
    osDelay(100);
    
    // 步骤3：读取峰2数据
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n[Peak] 读取峰2数据...\r\n", 26, 1000);
    
    uint8_t sector2, block2, offset2;
    if (!CalculateAddress(peak_info->peak2_channel, &sector2, &block2, &offset2)) {
        return false;
    }
    
    uint8_t peak2_block_data[16];
    
    // 优化：同一块则复用数据
    if (sector2 == sector1 && block2 == block1) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"[Peak] 峰2与峰1同块，复用\r\n", 28, 1000);
        memcpy(peak2_block_data, peak1_block_data, 16);
    } else {
        if (!ReadBlockSelective(pn532, uid, uid_length, sector2, block2, peak2_block_data)) {
            return false;
        }
    }
    
    if (!ExtractPeakData(pn532, uid, uid_length, sector2, block2, offset2, 
                        peak2_block_data, peak_info->peak2_data)) {
        return false;
    }
    
    peak_info->valid = true;
    
    // 汇总
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n===== 峰位汇总 =====\r\n", 26, 1000);
    
    sprintf(msg, "峰1: %d道, 数据=[%02X %02X %02X] (十进制:[%u %u %u])\r\n",
            peak_info->peak1_channel,
            peak_info->peak1_data[0], peak_info->peak1_data[1], peak_info->peak1_data[2],
            peak_info->peak1_data[0], peak_info->peak1_data[1], peak_info->peak1_data[2]);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    sprintf(msg, "峰2: %d道, 数据=[%02X %02X %02X] (十进制:[%u %u %u])\r\n",
            peak_info->peak2_channel,
            peak_info->peak2_data[0], peak_info->peak2_data[1], peak_info->peak2_data[2],
            peak_info->peak2_data[0], peak_info->peak2_data[1], peak_info->peak2_data[2]);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    HAL_UART_Transmit(&huart4, (uint8_t*)"====================\r\n\r\n", 24, 1000);
    
    return true;
}

PN532_ReadResult PN532_ReadCard_Once(void) {
  PN532_ReadResult result = {0};
  PN532 pn532;
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  HAL_UART_Transmit(
      &huart4, (uint8_t *)"[PN532] === ReadCard_Once Start ===\r\n", 38, 1000);

  // 初始化PN532硬件（仅首次执行）
  if (!pn532_initialized) {
    HAL_UART_Transmit(
        &huart4, (uint8_t *)"[PN532] First run, initializing...\r\n", 36, 1000);
    PN532_Init_Hardware();
    pn532_initialized = true;
    HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Init completed\r\n", 24,
                      1000);
  } else {
    HAL_UART_Transmit(
        &huart4, (uint8_t *)"[PN532] Already initialized, skip\r\n", 35, 1000);
  }

  // 寻卡
  HAL_UART_Transmit(&huart4,
                    (uint8_t *)"[PN532] Step 1: Searching for card...\r\n", 40,
                    1000);

  if (!PN532_SearchCard(result.uid, &result.uid_length)) {
    HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] ERROR: No card detected\r\n",
                      33, 1000);
    return result;
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Step 2: Card found!\r\n", 29,
                    1000);

  // 打印UID
  HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Card UID (", 18, 1000);
  char len_str[5];
  sprintf(len_str, "%d", result.uid_length);
  HAL_UART_Transmit(&huart4, (uint8_t *)len_str, strlen(len_str), 1000);
  HAL_UART_Transmit(&huart4, (uint8_t *)" bytes): ", 9, 1000);

  for (int i = 0; i < result.uid_length; i++) {
    char hex[5];
    sprintf(hex, "%02X ", result.uid[i]);
    HAL_UART_Transmit(&huart4, (uint8_t *)hex, 3, 1000);
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 1000);

  HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Step 3: Wait 50ms...\r\n", 30,
                    1000);
  osDelay(50); // 短暂等待，避免命令冲突

  // 认证并读取所有扇区
  HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Step 4: Authenticating...\r\n",
                    35, 1000);

  pn532.uart_handle = &huart6;

  if (PN532_MifareClassicAuthenticate_UART(&pn532, result.uid,
                                           result.uid_length, 1, key,
                                           0x60) == PN532_STATUS_OK) {

    HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] Authentication OK!\r\n", 28,
                      1000);

    HAL_UART_Transmit(&huart4,
                      (uint8_t *)"[PN532] Step 5: Reading all sectors...\r\n",
                      40, 1000);

    if (PN532_ReadAllSectors(&pn532, &result)) {
      result.success = true;
      result.timestamp = HAL_GetTick();
      HAL_UART_Transmit(
          &huart4, (uint8_t *)"[PN532] Card reading completed!\r\n", 33, 1000);
    } else {
      HAL_UART_Transmit(&huart4,
                        (uint8_t *)"[PN532] ERROR: Failed to read sectors\r\n",
                        39, 1000);
    }
  } else {
    HAL_UART_Transmit(&huart4,
                      (uint8_t *)"[PN532] ERROR: Authentication failed\r\n", 38,
                      1000);
  }

  HAL_UART_Transmit(&huart4, (uint8_t *)"[PN532] === ReadCard_Once End ===\r\n",
                    35, 1000);

  return result;
}

/* ============================================================================
   FreeRTOS任务包装器
   ============================================================================
 */void StartPN532_ICCardTask(void const *argument) {
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"\r\n========================================\r\n", 42);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"[PN532 Task] 任务启动\r\n", 24);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"========================================\r\n\r\n", 44);
    osDelay(500);

    // 初始化
    if (!pn532_initialized) {
        PN532_Init_Hardware();
        pn532_initialized = true;
    }

    // 寻卡
    uint8_t uid[7];
    uint8_t uid_length;
    
    if (!PN532_SearchCard(uid, &uid_length)) {
        HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"[PN532 Task] 未发现卡片\r\n", 26);
        goto task_end;
    }

    // 打印UID
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"[PN532 Task] UID: ", 18);
    for (int i = 0; i < uid_length; i++) {
        char hex[5];
        sprintf(hex, "%02X ", uid[i]);
        HAL_UART_Transmit_DMA(&huart4, (uint8_t*)hex, 3);
    }
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"\r\n", 2);

    osDelay(200);

    // 读取峰位信息
    PN532 pn532;
    pn532.uart_handle = &huart6;
    
    if (PN532_ReadPeakInfoSelective(&pn532, uid, uid_length, &g_peak_info)) {
        HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"[PN532 Task] *** 读取成功 ***\r\n", 32);
    } else {
        HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"[PN532 Task] *** 读取失败 ***\r\n", 32);
    }

task_end:
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"\r\n[PN532 Task] 任务完成\r\n", 26);
    osDelay(1000);

    PN532_ICCardTaskHandle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
   外部调用接口
   ============================================================================
 */
bool PN532_StartReadTask(void) {
  // 检查任务是否已存在
  if (PN532_ICCardTaskHandle != NULL) {
    HAL_UART_Transmit_DMA(
        &huart4,
        (uint8_t *)"[PN532 API] Task already running, please wait...\r\n", 50);
    return false;
  }

  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 API] Creating task...\r\n", 30);

  // 创建任务
  osThreadDef(PN532_ICCardTask, StartPN532_ICCardTask, PN532_TASK_PRIORITY, 0,
              PN532_TASK_STACK_SIZE);
  PN532_ICCardTaskHandle = osThreadCreate(osThread(PN532_ICCardTask), NULL);

  if (PN532_ICCardTaskHandle == NULL) {
    HAL_UART_Transmit(&huart4,
                      (uint8_t *)"[PN532 API] ERROR: Failed to create task\r\n",
                      42, 1000);
    return false;
  }

  HAL_UART_Transmit_DMA(&huart4,
                    (uint8_t *)"[PN532 API] Task created successfully\r\n", 39);
  return true;
}

bool PN532_IsTaskRunning(void) { return (PN532_ICCardTaskHandle != NULL); }

void PN532_ForceReset(void) {
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 API] Force reset...\r\n", 28);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(500);

  pn532_initialized = false; // 强制重新初始化

  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 API] Reset completed\r\n", 29);
}

/* ============================================================================
   私有函数实现
   ============================================================================
 */
static void PN532_Init_Hardware(void) {
  HAL_UART_Transmit_DMA(
      &huart4, (uint8_t *)"[PN532 HW] Initializing hardware...\r\n", 38);

  // 硬件复位
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 HW] Hardware reset...\r\n", 30);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  osDelay(200);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
  osDelay(3000);

  // 发送唤醒 + SAM配置
  HAL_UART_Transmit_DMA(&huart4,
                    (uint8_t *)"[PN532 HW] Sending wakeup + SAM config...\r\n",
                    43);

  static const uint8_t fused_wakeup_sam[] = {
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};

  HAL_UART_Transmit_DMA(&huart6, fused_wakeup_sam, sizeof(fused_wakeup_sam));
  osDelay(1000);

  // 清空接收缓冲
  uint8_t dummy[256];
  HAL_UART_Receive(&huart6, dummy, 256, 100);

  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 HW] Hardware ready\r\n", 27);
}

static bool PN532_SearchCard(uint8_t *uid, uint8_t *uid_length) {
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 Search] Clearing buffer...\r\n",
                    35);

  uint8_t dummy[256];
  HAL_UART_Receive(&huart6, dummy, 256, 100);

  uart6_rx_index = 0;
  HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer[0], 1);

  // 发送寻卡命令
  HAL_UART_Transmit_DMA(
      &huart4, (uint8_t *)"[PN532 Search] Sending InListPassiveTarget...\r\n",
      47);

  uint8_t card_search[] = {0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4,
                           0x4A, 0x01, 0x00, 0xE1, 0x00};
  HAL_UART_Transmit_DMA(&huart6, card_search, sizeof(card_search));
  osDelay(3000);

  // 打印接收到的数据
  char msg[50];
  sprintf(msg, "[PN532 Search] Received %d bytes\r\n", uart6_rx_index);
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));

  if (uart6_rx_index == 0) {
    HAL_UART_Transmit_DMA(
        &huart4, (uint8_t *)"[PN532 Search] ERROR: No response\r\n", 35);
    return false;
  }

  // 打印原始响应（十六进制）
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"[PN532 Search] Raw response: ", 29);
  for (int i = 0; i < uart6_rx_index; i++) {
    char hex[5];
    sprintf(hex, "%02X ", uart6_rx_buffer[i]);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)hex, 3);
  }
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"\r\n", 2);

  // 查找D5 4B响应
  HAL_UART_Transmit_DMA(
      &huart4, (uint8_t *)"[PN532 Search] Parsing response...\r\n", 36);

  for (int i = 0; i < uart6_rx_index - 1; i++) {
    if (uart6_rx_buffer[i] == 0xD5 && uart6_rx_buffer[i + 1] == 0x4B) {
      sprintf(msg, "[PN532 Search] Found D5 4B at index %d\r\n", i);
      HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));

      if (uart6_rx_index >= i + 13) {
        uint8_t num_targets = uart6_rx_buffer[i + 2];
        sprintf(msg, "[PN532 Search] num_targets = %d\r\n", num_targets);
        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));

        if (num_targets == 1) {
          *uid_length = uart6_rx_buffer[i + 7];
          sprintf(msg, "[PN532 Search] UID length = %d\r\n", *uid_length);
          HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));

          if (*uid_length >= 4 && *uid_length <= 7 &&
              i + 8 + *uid_length <= uart6_rx_index) {
            memcpy(uid, &uart6_rx_buffer[i + 8], *uid_length);

            HAL_UART_Transmit_DMA(
                &huart4, (uint8_t *)"[PN532 Search] UID extracted: ", 30);
            for (int j = 0; j < *uid_length; j++) {
              char hex[5];
              sprintf(hex, "%02X ", uid[j]);
              HAL_UART_Transmit_DMA(&huart4, (uint8_t *)hex, 3);
            }
            HAL_UART_Transmit_DMA(&huart4, (uint8_t *)"\r\n", 2);

            return true;
          } else {
            HAL_UART_Transmit_DMA(
                &huart4,
                (uint8_t
                     *)"[PN532 Search] ERROR: Invalid UID length or buffer\r\n",
                52);
          }
        } else {
          HAL_UART_Transmit_DMA(
              &huart4, (uint8_t *)"[PN532 Search] ERROR: num_targets != 1\r\n",
              40);
        }
      } else {
        HAL_UART_Transmit_DMA(
            &huart4, (uint8_t *)"[PN532 Search] ERROR: Response too short\r\n",
            42);
      }
    }
  }

  HAL_UART_Transmit_DMA(
      &huart4,
      (uint8_t *)"[PN532 Search] ERROR: No D5 4B found in response\r\n", 50);
  return false;
}

static bool PN532_ReadAllSectors(PN532 *pn532, PN532_ReadResult *result) {
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  int success_count = 0;
  int fail_count = 0;

  HAL_UART_Transmit_DMA(
      &huart4, (uint8_t *)"[PN532 Read] Starting to read sector 0 only...\r\n",
      47);

  // 只读取扇区0
  int sector = 0;

  // 认证扇区0
  if (PN532_MifareClassicAuthenticate_UART(pn532, result->uid,
                                           result->uid_length, sector, key,
                                           0x60) != PN532_STATUS_OK) {
    char msg[60];
    sprintf(msg, "[PN532 Read] Sector %2d: Auth FAILED\r\n", sector);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));
    fail_count++;
  } else {
    // 读取扇区0的块0、1、2
    for (int block = 0; block < 3; block++) {
      uint8_t block_addr = sector * 4 + block; // 0, 1, 2

      if (PN532_MifareClassicReadWithUID_UART(
              pn532, block_addr, result->sector_data[sector][block],
              result->uid, result->uid_length) == PN532_STATUS_OK) {
        success_count++;
        char msg[60];
        sprintf(msg, "[PN532 Read] Block %2d: Read SUCCESS\r\n", block_addr);
        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));
      } else {
        char msg[60];
        sprintf(msg, "[PN532 Read] Block %2d: Read FAILED\r\n", block_addr);
        HAL_UART_Transmit_DMA(&huart4, (uint8_t *)msg, strlen(msg));
        fail_count++;
      }
    }
  }

  char summary[80];
  sprintf(summary, "[PN532 Read] Summary: %d success, %d failed\r\n",
          success_count, fail_count);
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)summary, strlen(summary));

  return (fail_count == 0);
}

/**
 * @brief 获取块1数组数据
 * @param array 输出参数，用于存储块1数组数据
 * @param size 输出参数，用于存储数组大小
 * @return true=数据有效, false=数据无效
 */
bool PN532_GetBlock1Array(uint8_t *array, int *size) {
  if (array == NULL || size == NULL) {
    return false;
  }

  if (g_block1_array_size == 0) {
    return false; // 没有有效数据
  }

  // 复制数据
  for (int i = 0; i < g_block1_array_size; i++) {
    array[i] = g_block1_array[i];
  }
  *size = g_block1_array_size;

  return true;
}

/**
 * @brief 获取块2第一个字节
 * @return 块2第一个字节的十进制值，如果数据无效返回0
 */
uint8_t PN532_GetBlock2FirstByte(void) { return g_block2_first_byte; }
