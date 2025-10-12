# PN532 IC卡读取故障排除指南

## 问题现象
- PN532模块初始化成功
- 但无法检测到IC卡
- 输出："No card detected or error occurred"

## 已添加的调试功能

### 1. **完整的PN532初始化序列**
```c
// 初始化PN532设备
pn532 = PN532_CreateDevice_STM32F4();
PN532_Init_STM32F4(&pn532);

// 等待PN532稳定
HAL_Delay(100);

// 获取固件版本（验证通信）
uint8_t version[4];
result = PN532_GetFirmwareVersion_Device(&pn532, version);
if (result == PN532_STATUS_OK) {
  my_printf(&huart4, "PN532 Firmware: %d.%d\r\n", version[1], version[2]);
} else {
  my_printf(&huart4, "PN532 Communication Error!\r\n");
  return;
}

// 配置SAM
result = PN532_SamConfiguration_Device(&pn532);
if (result == PN532_STATUS_OK) {
  my_printf(&huart4, "PN532 SAM Configured\r\n");
} else {
  my_printf(&huart4, "PN532 SAM Configuration Failed!\r\n");
}
```

### 2. **调试输出**
- 显示固件版本
- 显示SAM配置状态
- 显示寻卡结果和响应数据

## 硬件连接检查

### **SPI1连接**
- **MOSI**: PA7
- **MISO**: PA6  
- **SCK**: PA5
- **CS**: PA4 (PN532_SPI_CS_PIN)

### **控制引脚**
- **RST**: PF0 (PN532_SPI_RST_PIN)
- **IRQ**: PF1 (PN532_SPI_IRQ_PIN)

### **电源**
- **VCC**: 3.3V
- **GND**: 地

## 故障排除步骤

### 1. **检查硬件连接**
- 确认SPI1引脚连接正确
- 确认RST和IRQ引脚连接
- 确认电源供应（3.3V）
- 确认地线连接

### 2. **检查SPI配置**
- 确认SPI1已正确初始化
- 确认时钟频率合适（建议1MHz以下）
- 确认数据位8位，MSB先发送

### 3. **检查PN532模块**
- 确认模块工作电压3.3V
- 确认模块没有损坏
- 尝试更换模块

### 4. **检查IC卡**
- 确认使用Mifare Classic卡
- 确认卡片没有损坏
- 尝试不同的卡片

## 预期输出

### **正常初始化**
```
PN532 Firmware: 1.6
PN532 SAM Configured
=== PN532 IC Card Reader Test ===
Please place a Mifare Classic card near the reader...
Searching for cards...
Search result: 0, Response[0]: 1
Card detected! UID: XX XX XX XX
```

### **通信错误**
```
PN532 Communication Error!
```

### **SAM配置失败**
```
PN532 SAM Configuration Failed!
```

### **无卡片**
```
Searching for cards...
Search result: 0, Response[0]: 0
No card detected or error occurred
```

## 常见问题

### 1. **固件版本获取失败**
- 检查SPI连接
- 检查CS引脚控制
- 检查时钟频率

### 2. **SAM配置失败**
- 检查PN532模块是否正常
- 检查电源供应
- 尝试重新复位

### 3. **寻卡失败**
- 检查IRQ引脚连接
- 检查卡片类型
- 检查卡片距离（应该很近）

## 调试建议

### 1. **添加更多调试信息**
```c
// 在PN532_CallFunction_Device中添加
my_printf(&huart4, "Command: 0x%02X, Params: %d\r\n", command, params_length);
my_printf(&huart4, "Response: ");
for(int i = 0; i < response_length; i++) {
  my_printf(&huart4, "%02X ", response[i]);
}
my_printf(&huart4, "\r\n");
```

### 2. **检查IRQ引脚状态**
```c
// 在寻卡前检查IRQ状态
bool irq_state = PN532_IRQ_Read_STM32F4(&pn532);
my_printf(&huart4, "IRQ State: %s\r\n", irq_state ? "HIGH" : "LOW");
```

### 3. **检查SPI通信**
- 使用示波器检查SPI信号
- 确认时钟和数据信号正确
- 确认CS信号控制正确

## 当前状态
✅ 添加了完整的PN532初始化序列
✅ 添加了固件版本检查
✅ 添加了SAM配置检查
✅ 添加了调试输出
✅ 修复了初始化流程

现在重新编译烧录，应该能看到更详细的调试信息，帮助定位问题！