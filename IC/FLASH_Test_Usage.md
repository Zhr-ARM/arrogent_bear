# Flash测试功能使用说明

## 概述
在 `flash_app.c` 中添加了完整的Flash测试功能，用于验证STM32F4的Flash存储器是否正常工作。

## 文件结构
- `Core/Inc/flash_app.h` - Flash测试接口头文件
- `Core/Src/flash_app.c` - Flash测试实现文件

## 测试功能

### 1. 基本功能测试 (`FLASH_Test_Basic`)
- 检查Flash错误标志
- 验证Flash空闲状态
- 确认Flash基本功能正常

### 2. 擦除测试 (`FLASH_Test_Erase`)
- 擦除测试扇区（扇区11）
- 验证擦除结果（检查是否为0xFF）
- 确认擦除功能正常

### 3. 读写测试 (`FLASH_Test_ReadWrite`)
- 生成测试数据模式
- 写入Flash存储器
- 从Flash读取数据
- 比较写入和读取的数据
- 验证读写功能正常

### 4. 完整测试套件 (`FLASH_Test_Complete`)
- 执行所有上述测试
- 进行多次写入测试（不同数据模式）
- 提供完整的测试报告

## 使用方法

### 方法1：在main.c中启用测试 (已配置)
测试已经在main.c中启用，会通过串口4输出结果。

### 方法2：选择测试类型
```c
// 完整测试 (推荐用于全面验证)
if (FLASH_Test_Complete()) {
    printf("Flash完整测试通过！\n");
}

// 快速测试 (用于快速验证)
if (FLASH_QuickTest()) {
    printf("Flash快速测试通过！\n");
}
```

### 方法3：在其他地方调用测试
```c
#include "flash_app.h"

void some_function() {
    // 执行完整测试
    if (FLASH_Test_Complete()) {
        printf("Flash测试通过！\n");
    } else {
        printf("Flash测试失败！\n");
    }
    
    // 或者执行单个测试
    if (FLASH_Test_Basic()) {
        printf("Flash基本功能正常\n");
    }
}
```

## 测试区域
- **测试扇区**: 扇区11 (FLASH_SECTOR_11)
- **测试地址**: 0x080E0000
- **扇区大小**: 128KB

## 注意事项
⚠️ **重要警告**：
1. 测试会擦除扇区11，请确保该扇区没有存储重要数据
2. 扇区11通常用于用户数据存储，测试前请备份重要数据
3. 测试过程中不要断电，可能导致Flash损坏

## 测试输出示例 (串口4输出)

### 完整测试输出
```
========================================
=== Starting Flash Test ===
========================================

========================================
=== FLASH Complete Test Suite ===
========================================
Test Sector: 11 (0x080E0000)
Sector Size: 128KB
Data Size: 256 bytes
========================================

[FLASH] Running Test 1/4: Basic Test...

=== FLASH Basic Test ===
[FLASH] Flash basic check passed

[FLASH] Running Test 2/4: Erase Test...

=== FLASH Erase Test ===
[FLASH] Erasing test sector...
[FLASH] Sector erased successfully
[FLASH] Erase test passed

[FLASH] Running Test 3/4: Read/Write Test...

=== FLASH Read/Write Test ===
[FLASH] Write Data (32 bytes):
AA AB AC AD AE AF B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 
BA BB BC BD BE BF C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 
[FLASH] Writing data to flash...
[FLASH] Data written successfully
[FLASH] Reading data from flash...
[FLASH] Data read successfully
[FLASH] Read Data (32 bytes):
AA AB AC AD AE AF B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 
BA BB BC BD BE BF C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 
[FLASH] Read/Write test passed

[FLASH] Running Test 4/4: Multiple Write Test...

=== FLASH Multiple Write Test ===
[FLASH] Pattern 0x00: PASS
[FLASH] Pattern 0x55: PASS
[FLASH] Pattern 0xAA: PASS
[FLASH] Pattern 0xFF: PASS

========================================
=== FLASH ALL TESTS PASSED ===
========================================

[MAIN] Flash complete test PASSED!
========================================
=== Flash Test Complete ===
========================================
```

### 快速测试输出
```
=== FLASH Quick Test ===
[FLASH] Flash basic check passed
[FLASH] Erasing test sector...
[FLASH] Sector erased successfully
[FLASH] Writing test data...
[FLASH] Data written successfully
[FLASH] Reading test data...
[FLASH] Data read successfully
[FLASH] Quick test PASSED!
[MAIN] Flash quick test PASSED!
```

## 故障排除
如果测试失败，可能的原因：
1. Flash硬件故障
2. 电源不稳定
3. 时钟配置问题
4. 测试扇区被保护

## 自定义测试
可以通过修改以下参数来自定义测试：
- `FLASH_TEST_SECTOR` - 测试扇区
- `FLASH_TEST_ADDRESS` - 测试地址
- `FLASH_TEST_DATA_SIZE` - 测试数据大小
- `FLASH_TEST_PATTERN_SIZE` - 测试模式大小
