# PN532 SPI通信深度调试修复

## 🔧 **发现的问题**

### **主要问题：SPI通信协议和时序问题**
- **问题1**: PN532需要先唤醒才能正常通信
- **问题2**: SPI时钟频率可能过高，导致通信失败
- **问题3**: 缺少详细的调试信息来定位问题

### **修复内容**

#### **1. 添加PN532唤醒序列**
```c
void PN532_Init_STM32F4(PN532_Device_STM32F4 *dev) {
  // 初始化SPI
  PN532_SPI_Init_STM32F4(dev);
  
  // 复位设备
  PN532_Reset_STM32F4(dev);
  
  // 等待复位完成
  HAL_Delay(100);
  
  // 唤醒PN532模块
  PN532_SPI_Wakeup_STM32F4(dev);
  
  // 等待唤醒完成
  HAL_Delay(50);
}
```

#### **2. 降低SPI时钟频率**
```c
// 修改前
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

// 修改后
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
```

#### **3. 改进SPI测试方法**
```c
// 使用PN532状态读取命令测试SPI通信
uint8_t status = 0;
PN532_CS_Low_STM32F4(&pn532);
uint8_t cmd = 0x02; // 状态读取命令
HAL_SPI_Transmit(&hspi1, &cmd, 1, 1000);
HAL_SPI_Receive(&hspi1, &status, 1, 1000);
PN532_CS_High_STM32F4(&pn532);
```

#### **4. 添加详细调试信息**
- ✅ WriteFrame失败调试
- ✅ WaitReady超时调试
- ✅ ACK数据详细显示
- ✅ ACK匹配检查

## 🎯 **当前配置**

### **SPI1配置**
- **模式**: 主模式，软件NSS控制
- **数据位**: 8位，MSB先发送
- **时钟极性**: 低电平空闲
- **时钟相位**: 第1边沿采样
- **波特率**: 系统时钟/32 (降低8倍)
- **NSS**: 软件控制

### **PN532通信序列**
1. **复位**: 拉低RST引脚100ms
2. **等待**: 100ms稳定时间
3. **唤醒**: 发送0x55唤醒序列
4. **等待**: 50ms唤醒完成
5. **通信**: 正常SPI通信

## 📋 **预期输出**

### **修复后应该看到**
```
Testing SPI communication...
PN532 Status: 0xXX
Getting firmware version...
ACK: 00 00 FF 00 FF 00
PN532 Firmware: 1.6
PN532 SAM Configured
=== PN532 IC Card Reader Test ===
```

### **如果仍有问题**
```
Testing SPI communication...
PN532 Status: 0x00
Getting firmware version...
WriteFrame failed!
Firmware version failed!
```

## 🔍 **故障排除**

### **如果状态读取为0x00**
1. 检查SPI连接（PB3, PB4, PB5）
2. 检查PA4 CS控制
3. 检查PN532模块电源
4. 检查复位引脚PF0

### **如果WriteFrame失败**
1. 检查PN532模块是否正常
2. 检查唤醒序列是否正确
3. 检查SPI时钟频率是否合适

### **如果ACK不匹配**
1. 检查PN532模块响应
2. 检查SPI通信时序
3. 检查模块是否处于正确状态

## ✅ **修复完成**

- ✅ 添加了PN532唤醒序列
- ✅ 降低了SPI时钟频率
- ✅ 改进了SPI测试方法
- ✅ 添加了详细的调试信息
- ✅ 修复了通信协议问题

现在重新编译烧录，应该能看到更详细的调试信息，帮助定位问题！