# PN532 IC卡读取功能说明

## 功能概述

本工程已集成PN532 NFC模块的IC卡读取功能，支持Mifare Classic卡片的读取操作。

## 硬件连接

### PN532模块连接（SPI1）
- **CS (片选)**: PA4
- **RST (复位)**: PF0  
- **IRQ (中断)**: PF1
- **MOSI**: PA7
- **MISO**: PA6
- **SCK**: PA5
- **VCC**: 3.3V
- **GND**: GND

## 软件功能

### 1. 初始化流程
1. 创建PN532设备实例
2. 初始化SPI1接口
3. 复位PN532模块
4. 配置GPIO引脚

### 2. IC卡读取流程
1. **寻卡**: 检测Mifare Classic卡片
2. **获取UID**: 读取卡片的唯一标识符
3. **验证密码**: 使用默认密码0xFF进行认证
4. **读取数据**: 读取前4个数据块（Block 0-3）

### 3. 数据输出
- 通过UART4输出调试信息
- 显示卡片UID
- 显示每个数据块的内容（16字节十六进制）

## 使用方法

### 方法1: 自动测试（推荐）
系统启动后会自动执行一次IC卡读取测试：
1. 将Mifare Classic卡片靠近PN532模块
2. 观察UART4输出（波特率115200）
3. 查看卡片数据和UID

### 方法2: FreeRTOS任务
系统运行时会持续监控IC卡：
1. 检测到卡片时自动读取
2. 通过UART4输出数据
3. 支持连续读取多张卡片

## 输出示例

```
=== PN532 IC Card Reader Test ===
Please place a Mifare Classic card near the reader...
Card detected! UID: 04 5A 2B 3C 4D
Block 0: 04 5A 2B 3C 4D 88 04 00 62 63 64 65 66 67 68 69
Block 1: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
Block 2: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
Block 3: FF FF FF FF FF FF FF 07 80 69 FF FF FF FF FF FF
Card reading completed!
=== Test completed ===
```

## 技术参数

- **支持卡片类型**: Mifare Classic 1K/4K
- **通信接口**: SPI1 (4MHz)
- **认证方式**: Key A (默认密码0xFF)
- **读取范围**: Block 0-3 (前4个数据块)
- **超时时间**: 5秒（测试模式）/ 1秒（任务模式）

## 故障排除

### 1. 无法检测到卡片
- 检查PN532模块电源连接
- 确认SPI1连接正确
- 检查卡片是否支持Mifare Classic协议

### 2. 认证失败
- 确认卡片使用默认密码0xFF
- 检查卡片是否已损坏
- 尝试其他Mifare Classic卡片

### 3. 读取失败
- 检查卡片与模块距离（建议1-2cm）
- 确认卡片放置稳定
- 检查SPI通信是否正常

## 扩展功能

### 修改密码
在代码中修改`key`数组：
```c
uint8_t key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 修改为实际密码
```

### 读取更多数据块
修改循环范围：
```c
for (uint8_t block = 0; block < 16; block++) // 读取前16个块
```

### 添加写卡功能
使用现有的`PN532_MifareClassicWriteBlock_Device`函数

## 注意事项

1. 确保PN532模块与STM32F4的电压匹配（3.3V）
2. 卡片读取距离不宜过远（建议1-2cm）
3. 避免在强电磁干扰环境下使用
4. 定期检查SPI连接是否稳定

## 相关文件

- `Core/Src/pn532.c` - PN532通用驱动
- `Core/Src/pn532_stm32f4.c` - STM32F4专用驱动
- `Core/Src/freertos.c` - FreeRTOS任务实现
- `Core/Src/main.c` - 测试函数
- `Core/Inc/pn532.h` - 头文件定义