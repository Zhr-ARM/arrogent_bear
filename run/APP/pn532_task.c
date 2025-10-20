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
static volatile uint8_t stop_flag = 0;
static volatile uint8_t warn_flag = 0;
static uint32_t stop_start_time = 0;
static const uint32_t STOP_DURATION = 8000;

/* 卡片数据缓冲区 */
CardDataBuffer g_card_data_buffer = {0};

/* 峰位信息全局变量 */
PeakInfo g_peak_info = {0};

/* 块1和块2数据变量 */
uint8_t g_block1_array[4] = {0};
int g_block1_array_size = 0;
uint8_t g_block2_first_byte = 0;

/* External variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void PN532_Init_Hardware(void);
static bool PN532_SearchCard(uint8_t *uid, uint8_t *uid_length);
static bool PN532_ReadAllSectors(PN532 *pn532, PN532_ReadResult *result);

// USART6标准中断接收变量
uint32_t uart6_rx_ticks = 0;
uint16_t uart6_rx_index = 0;
uint8_t uart6_rx_buffer[256] = {0};
volatile uint8_t uart6_rx_flag = 0;

/* ============================================================================
   核心读卡函数（同步执行）
   ============================================================================
 */
void uart6_task(void) {
  if (uart6_rx_index == 0)
    return;

  if (HAL_GetTick() - uart6_rx_ticks > 200) {
    uart6_rx_index = 0;
    huart6.pRxBuffPtr = uart6_rx_buffer;
  }
}

/**
 * @brief 根据道址计算扇区、块号和字节偏移
 */
static bool CalculateAddress(uint16_t channel, uint8_t *sector, uint8_t *block, uint8_t *byte_offset) {
    if (channel >= 720) {
        return false;
    }
    
    *sector = (channel / 48) + 1;
    uint16_t offset_in_sector = channel % 48;
    *block = offset_in_sector / 16;
    *byte_offset = offset_in_sector % 16;
    
    return true;
}

/**
 * @brief 初始化卡片数据缓冲区
 */
void CardBuffer_Init(void) {
    memset(&g_card_data_buffer, 0, sizeof(CardDataBuffer));
}

/**
 * @brief 添加一条卡片记录（自动过滤无效数据）
 */
bool CardBuffer_AddRecord(const PeakInfo *peak_info, const uint8_t *uid, uint8_t uid_length) {
    if (peak_info == NULL || uid == NULL || !peak_info->valid) {
        return false;
    }
    
    CardDataRecord *new_record = &g_card_data_buffer.records[g_card_data_buffer.write_index];
    memset(new_record, 0, sizeof(CardDataRecord));
    
    memcpy(new_record->uid, uid, uid_length);
    new_record->uid_length = uid_length;
    new_record->timestamp = HAL_GetTick();
    
    uint8_t valid_index = 0;
    
    bool peak1_valid = (peak_info->peak1_channel > 0 && peak_info->peak1_channel < 720);
    if (peak1_valid) {
        if (peak_info->peak1_data[0] != 0 || 
            peak_info->peak1_data[1] != 0 || 
            peak_info->peak1_data[2] != 0) {
            
            new_record->peak_channels[valid_index] = peak_info->peak1_channel;
            memcpy(new_record->peak_spectrums[valid_index], 
                   peak_info->peak1_data, 3);
            valid_index++;
        }
    }
    
    bool peak2_valid = (peak_info->peak2_channel > 0 && peak_info->peak2_channel < 720);
    if (peak2_valid) {
        if (peak_info->peak2_data[0] != 0 || 
            peak_info->peak2_data[1] != 0 || 
            peak_info->peak2_data[2] != 0) {
            
            new_record->peak_channels[valid_index] = peak_info->peak2_channel;
            memcpy(new_record->peak_spectrums[valid_index], 
                   peak_info->peak2_data, 3);
            valid_index++;
        }
    }
    
    new_record->valid_peak_count = valid_index;
    
    if (valid_index == 0) {
        return false;
    }
    
    g_card_data_buffer.write_index = (g_card_data_buffer.write_index + 1) % 12;
    
    if (g_card_data_buffer.count < 12) {
        g_card_data_buffer.count++;
    }
    
    return true;
}

/**
 * @brief 获取指定索引的记录
 */
bool CardBuffer_GetRecord(uint8_t index, CardDataRecord *record) {
    if (index >= 12 || record == NULL) {
        return false;
    }
    
    if (index >= g_card_data_buffer.count) {
        return false;
    }
    
    memcpy(record, &g_card_data_buffer.records[index], sizeof(CardDataRecord));
    return true;
}

/**
 * @brief 获取当前存储的卡片数量
 */
uint8_t CardBuffer_GetCount(void) {
    return g_card_data_buffer.count;
}

/**
 * @brief 清空缓冲区
 */
void CardBuffer_Clear(void) {
    memset(&g_card_data_buffer, 0, sizeof(CardDataBuffer));
}

/**
 * @brief 打印所有存储的记录（简化版 - 单行格式）
 */
void CardBuffer_Print(void) {
    char msg[150];
    
    sprintf(msg, "\r\n卡片数: %d\r\n", g_card_data_buffer.count);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    
    if (g_card_data_buffer.count == 0) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"缓冲区为空\r\n\r\n", 14, 1000);
        return;
    }
    
    for (uint8_t i = 0; i < g_card_data_buffer.count; i++) {
        CardDataRecord *card = &g_card_data_buffer.records[i];
        
        // 格式: #序号 | UID:XXXXXXXX | T:时间戳 | Peaks:峰数 | Ch1:道址1
        sprintf(msg, "#%d | UID:", i+1);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
        
        for (int j = 0; j < card->uid_length; j++) {
            sprintf(msg, "%02X", card->uid[j]);
            HAL_UART_Transmit(&huart4, (uint8_t*)msg, 2, 1000);
        }
        
        sprintf(msg, " | T:%lu | Peaks:%d | Ch1:%u\r\n",
                card->timestamp,
                card->valid_peak_count,
                card->valid_peak_count > 0 ? card->peak_channels[0] : 0);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
    }
    
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 1000);
}

/**
 * @brief 读取指定块的数据（按需认证）
 */
static bool ReadBlockSelective(PN532 *pn532, uint8_t *uid, uint8_t uid_length,
                               uint8_t sector, uint8_t block, uint8_t *data) {
    uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t block_addr = sector * 4 + block;
    
    if (PN532_MifareClassicAuthenticate_UART(pn532, uid, uid_length, 
                                             sector * 4, key, 0x60) != PN532_STATUS_OK) {
        return false;
    }
    
    vTaskDelay(50);
    
    if (PN532_MifareClassicReadWithUID_UART(pn532, block_addr, data, uid, uid_length) != PN532_STATUS_OK) {
        return false;
    }
    
    return true;
}

/**
 * @brief 从块数据中提取3字节峰值数据
 */
static bool ExtractPeakData(PN532 *pn532, uint8_t *uid, uint8_t uid_length,
                           uint8_t sector, uint8_t block, uint8_t byte_offset,
                           uint8_t *block_data, uint8_t *peak_data) {
    
    if (byte_offset == 0) {
        uint8_t prev_sector = sector;
        uint8_t prev_block = block - 1;
        
        if (block == 0) {
            if (sector == 1) {
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
        uint8_t prev_sector = sector;
        uint8_t prev_block = block - 1;
        
        if (block == 0) {
            if (sector == 1) {
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
        peak_data[0] = block_data[byte_offset - 1];
        peak_data[1] = block_data[byte_offset];
        peak_data[2] = block_data[byte_offset + 1];
        
    } else if (byte_offset == 15) {
        peak_data[0] = block_data[14];
        peak_data[1] = block_data[15];
        
        uint8_t next_sector = (block == 2) ? sector + 1 : sector;
        uint8_t next_block = (block == 2) ? 0 : block + 1;
        
        if (next_sector > 15) {
            return false;
        }
        
        uint8_t next_data[16];
        if (!ReadBlockSelective(pn532, uid, uid_length, next_sector, next_block, next_data)) {
            return false;
        }
        peak_data[2] = next_data[0];
    }
    
    return true;
}

/**
 * @brief 选择性读取峰位信息（只读必要的块）
 */
bool PN532_ReadPeakInfoSelective(PN532 *pn532, uint8_t *uid, uint8_t uid_length, PeakInfo *peak_info) {
    uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t block1_data[16];
    uint8_t block2_data[16];
    
    memset(peak_info, 0, sizeof(PeakInfo));
    
    if (PN532_MifareClassicAuthenticate_UART(pn532, uid, uid_length, 0, key, 0x60) != PN532_STATUS_OK) {
        return false;
    }
    
    vTaskDelay(50);
    
    if (PN532_MifareClassicReadWithUID_UART(pn532, 2, block2_data, uid, uid_length) != PN532_STATUS_OK) {
        return false;
    }
    
    g_block2_first_byte = block2_data[0];
    
    extern volatile uint8_t system_started;
    
    if (g_block2_first_byte == 0x0A) {
        system_started = 1;
        return false;
        
    } else if (g_block2_first_byte == 0x0B) {
        system_started = 0;
        return false;
    }
    
    vTaskDelay(100);
    
    if (PN532_MifareClassicReadWithUID_UART(pn532, 1, block1_data, uid, uid_length) != PN532_STATUS_OK) {
        return false;
    }
    
    g_block1_array_size = 4;
    for (int i = 0; i < 4; i++) {
        g_block1_array[i] = block1_data[i];
    }
    
    uint16_t peak1_channel_raw = ((uint16_t)block1_data[1] << 8) | block1_data[0];
    uint16_t peak2_channel_raw = ((uint16_t)block1_data[3] << 8) | block1_data[2];
    
    peak_info->peak1_channel = peak1_channel_raw;
    peak_info->peak2_channel = peak2_channel_raw;
    
    bool peak1_valid = (peak1_channel_raw > 0 && peak1_channel_raw < 720);
    bool peak2_valid = (peak2_channel_raw > 0 && peak2_channel_raw < 720);
    
    int valid_peak_count = 0;
    if (peak1_valid) valid_peak_count++;
    if (peak2_valid) valid_peak_count++;
    
    if (valid_peak_count == 0) {
        return false;
    }
    
    vTaskDelay(100);
    
    if (peak1_valid) {
        uint8_t sector1, block1, offset1;
        if (!CalculateAddress(peak_info->peak1_channel, &sector1, &block1, &offset1)) {
            peak1_valid = false;
        } else {
            uint8_t peak1_block_data[16];
            if (!ReadBlockSelective(pn532, uid, uid_length, sector1, block1, peak1_block_data)) {
                peak1_valid = false;
            } else {
                if (!ExtractPeakData(pn532, uid, uid_length, sector1, block1, offset1, 
                                    peak1_block_data, peak_info->peak1_data)) {
                    peak1_valid = false;
                }
            }
        }
    } else {
        memset(peak_info->peak1_data, 0, sizeof(peak_info->peak1_data));
    }
    
    vTaskDelay(100);
    
    if (peak2_valid) {
        uint8_t sector2, block2, offset2;
        if (!CalculateAddress(peak_info->peak2_channel, &sector2, &block2, &offset2)) {
            peak2_valid = false;
        } else {
            uint8_t peak2_block_data[16];
            if (!ReadBlockSelective(pn532, uid, uid_length, sector2, block2, peak2_block_data)) {
                peak2_valid = false;
            } else {
                if (!ExtractPeakData(pn532, uid, uid_length, sector2, block2, offset2, 
                                    peak2_block_data, peak_info->peak2_data)) {
                    peak2_valid = false;
                }
            }
        }
    } else {
        memset(peak_info->peak2_data, 0, sizeof(peak_info->peak2_data));
    }
    
    peak_info->valid = true;
    
    return true;
}

PN532_ReadResult PN532_ReadCard_Once(void) {
  PN532_ReadResult result = {0};
  PN532 pn532;
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if (!pn532_initialized) {
    PN532_Init_Hardware();
    pn532_initialized = true;
  }

  if (!PN532_SearchCard(result.uid, &result.uid_length)) {
    return result;
  }

  vTaskDelay(50);

  pn532.uart_handle = &huart6;

  if (PN532_MifareClassicAuthenticate_UART(&pn532, result.uid,
                                           result.uid_length, 1, key,
                                           0x60) == PN532_STATUS_OK) {

    if (PN532_ReadAllSectors(&pn532, &result)) {
      result.success = true;
      result.timestamp = HAL_GetTick();
    }
  }

  return result;
}

/* ============================================================================
   FreeRTOS任务包装器
   ============================================================================
 */
void StartPN532_ICCardTask(void const *argument) {
    
    if (!pn532_initialized) {
        PN532_Init_Hardware();
        pn532_initialized = true;
    }
    
    CardBuffer_Init();
    
    // ? 新增：周期性打印的时间戳
    uint32_t last_print_time = 0;
    
    for (;;) {
        // 冷却期检查
        if (stop_flag == 1) {
            uint32_t elapsed_time = HAL_GetTick() - stop_start_time;
            
            if (elapsed_time >= STOP_DURATION) {
                stop_flag = 0;
            } else {
                vTaskDelay(1000);
                continue;
            }
        }
        
        // 寻卡
        uint8_t uid[7];
        uint8_t uid_length;
        
        if (!PN532_SearchCard(uid, &uid_length)) {
            vTaskDelay(500);
            
            // ? 新增：周期性打印（每隔10秒）
            if (HAL_GetTick() - last_print_time > 10000) {
                CardBuffer_Print();
                last_print_time = HAL_GetTick();
            }
            
            continue;
        }
        
        // 设置标志位
        stop_flag = 1;
        warn_flag = 1;
        stop_start_time = HAL_GetTick();
        
        vTaskDelay(200);
        
        // 读取峰位信息
        PN532 pn532;
        pn532.uart_handle = &huart6;
        
        if (PN532_ReadPeakInfoSelective(&pn532, uid, uid_length, &g_peak_info)) {
            if (CardBuffer_AddRecord(&g_peak_info, uid, uid_length)) {
                // ? 读卡成功后立即打印
                CardBuffer_Print();
                last_print_time = HAL_GetTick();  // 更新打印时间戳
            }
        }
        
        vTaskDelay(1000);
    }
}

/* ============================================================================
   外部调用接口
   ============================================================================
 */
bool PN532_StartReadTask(void) {
  if (PN532_ICCardTaskHandle != NULL) {
    return false;
  }

  osThreadDef(PN532_ICCardTask, StartPN532_ICCardTask, PN532_TASK_PRIORITY, 0,
              PN532_TASK_STACK_SIZE);
  PN532_ICCardTaskHandle = osThreadCreate(osThread(PN532_ICCardTask), NULL);

  if (PN532_ICCardTaskHandle == NULL) {
    return false;
  }

  return true;
}

bool PN532_IsTaskRunning(void) { 
    return (PN532_ICCardTaskHandle != NULL); 
}

void PN532_ForceReset(void) {
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(500);

  pn532_initialized = false;
}

/* ============================================================================
   私有函数实现
   ============================================================================
 */
static void PN532_Init_Hardware(void) {
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
  vTaskDelay(200);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
  vTaskDelay(3000);

  static const uint8_t fused_wakeup_sam[] = {
      0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};

  HAL_UART_Transmit(&huart6, fused_wakeup_sam, sizeof(fused_wakeup_sam), 1000);
  vTaskDelay(1000);

  uint8_t dummy[256];
  HAL_UART_Receive(&huart6, dummy, 256, 100);
}

static bool PN532_SearchCard(uint8_t *uid, uint8_t *uid_length) {
  uint8_t dummy[256];
  HAL_UART_Receive(&huart6, dummy, 256, 100);

  uart6_rx_index = 0;
  HAL_UART_Receive_IT(&huart6, &uart6_rx_buffer[0], 1);

  uint8_t card_search[] = {0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4,
                           0x4A, 0x01, 0x00, 0xE1, 0x00};
  HAL_UART_Transmit(&huart6, card_search, sizeof(card_search), 1000);
  vTaskDelay(3000);

  if (uart6_rx_index == 0) {
    return false;
  }

  for (int i = 0; i < uart6_rx_index - 1; i++) {
    if (uart6_rx_buffer[i] == 0xD5 && uart6_rx_buffer[i + 1] == 0x4B) {
      if (uart6_rx_index >= i + 13) {
        uint8_t num_targets = uart6_rx_buffer[i + 2];

        if (num_targets == 1) {
          *uid_length = uart6_rx_buffer[i + 7];

          if (*uid_length >= 4 && *uid_length <= 7 &&
              i + 8 + *uid_length <= uart6_rx_index) {
            memcpy(uid, &uart6_rx_buffer[i + 8], *uid_length);
            return true;
          }
        }
      }
    }
  }

  return false;
}

static bool PN532_ReadAllSectors(PN532 *pn532, PN532_ReadResult *result) {
  uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  int success_count = 0;
  int fail_count = 0;

  int sector = 0;

  if (PN532_MifareClassicAuthenticate_UART(pn532, result->uid,
                                           result->uid_length, sector, key,
                                           0x60) != PN532_STATUS_OK) {
    fail_count++;
  } else {
    for (int block = 0; block < 3; block++) {
      uint8_t block_addr = sector * 4 + block;

      if (PN532_MifareClassicReadWithUID_UART(
              pn532, block_addr, result->sector_data[sector][block],
              result->uid, result->uid_length) == PN532_STATUS_OK) {
        success_count++;
      } else {
        fail_count++;
      }
    }
  }

  return (fail_count == 0);
}

/**
 * @brief 获取块1数组数据
 */
bool PN532_GetBlock1Array(uint8_t *array, int *size) {
  if (array == NULL || size == NULL) {
    return false;
  }

  if (g_block1_array_size == 0) {
    return false;
  }

  for (int i = 0; i < g_block1_array_size; i++) {
    array[i] = g_block1_array[i];
  }
  *size = g_block1_array_size;

  return true;
}

/**
 * @brief 获取块2第一个字节
 */
uint8_t PN532_GetBlock2FirstByte(void) { 
    return g_block2_first_byte; 
}

/**
 * @brief 获取stop_flag状态
 */
uint8_t PN532_GetStopFlag(void) {
    return stop_flag;
}

/**
 * @brief 获取warn_flag状态
 */
uint8_t PN532_GetWarnFlag(void) {
    return warn_flag;
}

/**
 * @brief 获取剩余冷却时间（秒）
 */
uint32_t PN532_GetRemainingCooldown(void) {
    if (stop_flag == 0) {
        return 0;
    }
    
    uint32_t elapsed = HAL_GetTick() - stop_start_time;
    if (elapsed >= STOP_DURATION) {
        return 0;
    }
    
    return (STOP_DURATION - elapsed) / 1000;
}

/**
 * @brief 手动清除stop_flag和warn_flag（紧急情况使用）
 */
void PN532_ClearFlags(void) {
    stop_flag = 0;
    warn_flag = 0;
}
