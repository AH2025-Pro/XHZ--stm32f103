# STM32F103RET6 20kg 1g精度称重系统

这是一个使用STM32F103RET6微控制器实现的20kg量程、1g精度的称重系统项目。该项目包括以下几个主要部分：

## 1. SPI读取CS1231AD转换器结果

- **CS1231AD转换器**：用于读取L6D称重传感器的数据。
- **SPI通信**：通过SPI接口读取转换器的数据。

## 2. 改进后的卡尔曼滤波

- **卡尔曼滤波**：对从传感器读取的数据进行滤波处理，提高数据的准确性和稳定性。
- **改进后的滤波算法**：在传统卡尔曼滤波的基础上进行了改进，以适应称重系统的特殊需求。

## 3. 常用滤波效果比较

- **滤波效果比较**：比较不同滤波算法的效果，选择最适合称重系统的数据滤波方法。

## 4. I2C协议读取和存写EEPROM信息

- **I2C通信**：通过I2C协议与EEPROM进行通信。
- **EEPROM读写**：读取和存储称重系统的相关数据。
- **注意事项**：
  - I2C协议的起始格式。
  - EEPROM的页写入操作。

## 5. USART通过SP485芯片进行485总线通信

- **USART通信**：使用USART接口进行数据通信。
- **SP485芯片**：通过SP485芯片实现RS-485总线通信。
- **485总线通信**：在称重系统中实现长距离、抗干扰的数据传输。

## 硬件需求

- STM32F103RET6微控制器
- CS1231AD转换器
- L6D称重传感器
- EEPROM存储器
- SP485芯片

## 软件需求

- STM32CubeIDE或Keil MDK
- C/C++编程环境

## 使用说明

1. **硬件连接**：
   - 按照电路图连接STM32F103RET6、CS1231AD、L6D、EEPROM和SP485芯片。

2. **软件配置**：
   - 使用STM32CubeIDE或Keil MDK配置项目。
   - 配置SPI、I2C和USART接口。

3. **编译和下载**：
   - 编译项目代码。
   - 将编译后的程序下载到STM32F103RET6微控制器。

4. **运行测试**：
   - 运行程序，测试称重系统的精度和稳定性。
