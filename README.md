# ProjectFusionBoard

## 项目简介

`ProjectFusionBoard` 是一个基于 `STM32F103` 的嵌入式传感器联调与数据采集工程。  
从当前仓库内容来看，这个项目同时包含两条技术主线：

- 一条是以 `MPU6050` 为核心的 IMU / 姿态解算方向
- 另一条是以 `AD7177` 为核心的多通道高精度 ADC 采集方向

在当前版本中，主程序入口 [Core/Src/main.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/main.c:1) 运行的并不是完整的传感器融合逻辑，而是一个 **3 路 AD7177 联调测试程序**。

如果你是第一次接手这个项目，可以先把它理解成：

- 一个已经搭好基础框架的 STM32 固件工程
- 一组正在逐步接入和验证的传感器驱动
- 一个当前重点在做 ADC 采集链路 bring-up 的开发分支

这份 README 的目标是帮助新同学快速建立全局认知，知道项目现在做到哪一步、应该从哪里开始看、以及如何在本地复现当前状态。

## 当前开发阶段

### 当前主程序在做什么

当前 `main.c` 的主流程主要完成以下工作：

- 初始化 `GPIO`
- 初始化 `I2C1`
- 初始化 `SPI1`
- 初始化 `USART1`
- 初始化 3 个 `AD7177`
- 周期性读取三路 ADC 数据
- 将原始数据换算为电压值
- 通过串口持续输出，便于上位机绘图或观察

结合 [Core/Src/main.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/main.c:1) 可以确认当前几个关键参数：

- 系统采样循环频率为 `50 Hz`
- ADC 当前配置的输出数据率为 `AD7177_ODR_59SPS`
- 当前启用了 `3` 个 AD7177 设备
- 启动后会打印：

```text
frequency=50Hz
3xAD7177 test start
```

- 运行期数据输出格式为：

```text
{plotter}v1,v2,v3
```

例如：

```text
{plotter}1.234567,1.210000,1.198765
```

这说明当前版本的主要目标是把三路 ADC 的通信、采样和串口观测链路跑通。

### 当前被暂时停用的部分

仓库中仍然保留了 `MPU6050` 驱动以及 Mahony 姿态解算相关代码和公式推导，但它们目前 **没有接入当前主循环**。

在 [Core/Src/main.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/main.c:1) 中可以看到明显注释，说明 `MPU6050` 路径在当前 `3x AD7177 bring-up` 阶段是临时关闭的。这不是废弃代码，而是阶段性切换。

## 项目整体目标理解

从代码和仓库中的文档判断，这个项目更长期的目标大概率包括：

- 实现板上传感器的稳定底层通信
- 获取可靠的 IMU 姿态信息，尤其是 `roll/pitch`
- 将姿态信息与其他传感器数据结合使用
- 稳定采集多通道模拟量并进行可视化与后续处理

其中 [Mahony_Formula_Derivation.txt](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Mahony_Formula_Derivation.txt:1) 已经对 Mahony 方案进行了比较系统的推导和工程分析，说明姿态解算并不是临时想法，而是项目明确考虑过的后续方向。

## 硬件与外设概览

### MCU 与工具链目标

从 Keil 工程文件 [MDK-ARM/ProjectFusionBoard.uvprojx](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/MDK-ARM/ProjectFusionBoard.uvprojx:1) 可以看出：

- 目标芯片：`STM32F103RC`
- 开发环境：`Keil uVision`
- 编译器系列：`ARMCC 5`

### 当前主程序正在使用的外设

- `GPIO`
  - 用于 AD7177 片选控制
- `SPI1`
  - 当前 AD7177 的通信总线
- `USART1`
  - 当前调试与数据输出串口
- `I2C1`
  - 当前虽然初始化了，但主要是为 MPU6050 路径保留

### 当前工程中存在但未进入主流程的外设

- `SPI2`
  - 初始化函数存在于 `main.c`
  - 当前 `main()` 没有调用
  - 最新构建日志也提示它已声明但未使用

### 当前 AD7177 片选引脚

当前代码中的 3 个 AD7177 片选定义为：

- `AD7177_CS1`: `PC12`
- `AD7177_CS2`: `PB3`
- `AD7177_CS3`: `PB5`

### 当前串口配置

当前串口默认配置为：

- 外设：`USART1`
- 波特率：`115200`
- 模式：`TX/RX`
- 格式：`8N1`

## 软件结构说明

这个工程基本遵循 STM32Cube + Keil 的典型目录布局，在 HAL 之上叠加了自定义驱动和通信模块。

### 顶层目录

- [Core](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core:1)
  - 主程序、时钟、MSP、异常与中断处理
- [Drivers](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers:1)
  - CMSIS、STM32 HAL、自定义 I2C/SPI 驱动
- [Communication](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Communication:1)
  - UART 通信协议模块
- [MDK-ARM](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/MDK-ARM:1)
  - Keil 工程文件与相关配置
- [ProjectFusionBoard.ioc](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/ProjectFusionBoard.ioc:1)
  - STM32CubeMX 配置文件
- [Mahony_Formula_Derivation.txt](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Mahony_Formula_Derivation.txt:1)
  - Mahony 姿态解算相关推导与工程分析

### 核心应用层文件

- [Core/Src/main.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/main.c:1)
  - 当前主程序入口
  - 外设初始化
  - AD7177 采样循环
  - 串口输出

- [Core/Src/stm32f1xx_it.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/stm32f1xx_it.c:1)
  - 中断处理

- [Core/Src/stm32f1xx_hal_msp.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/stm32f1xx_hal_msp.c:1)
  - HAL 底层初始化

- [Core/Src/system_stm32f1xx.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/system_stm32f1xx.c:1)
  - 系统时钟和底层系统支持

### 自定义驱动模块

- [Drivers/I2C/i2c_hal_wrapper.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/I2C/i2c_hal_wrapper.h:1)
  和 [Drivers/I2C/i2c_hal_wrapper.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/I2C/i2c_hal_wrapper.c:1)
  - 对 HAL I2C 接口做了一层轻量封装

- [Drivers/I2C/mpu6050.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/I2C/mpu6050.h:1)
  和 [Drivers/I2C/mpu6050.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/I2C/mpu6050.c:1)
  - MPU6050 初始化
  - 原始数据读取
  - 单位换算
  - Mahony 更新逻辑

- [Drivers/SPI/ad7177.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/ad7177.h:1)
  和 [Drivers/SPI/ad7177.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/ad7177.c:1)
  - AD7177 寄存器级驱动
  - SPI 复位、读写寄存器、等待 ready、数据换算

- [Drivers/SPI/spi_adc.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/spi_adc.h:1)
  和 [Drivers/SPI/spi_adc.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/spi_adc.c:1)
  - 较早期的通用 SPI ADC 抽象

- [Drivers/SPI/spi_flash.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/spi_flash.h:1)
  和 [Drivers/SPI/spi_flash.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/spi_flash.c:1)
  - SPI Flash 驱动

### 通信模块

- [Communication/uart_protocol/uart_proto.h](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Communication/uart_protocol/uart_proto.h:1)
  和 [Communication/uart_protocol/uart_proto.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Communication/uart_protocol/uart_proto.c:1)
  - 自定义 UART 帧协议
  - 命令解析与数据包封装

需要注意的是，这个 UART 协议模块虽然已经编入工程，但当前 `AD7177` 联调主流程主要还是直接调用 `HAL_UART_Transmit()` 输出纯文本数据，并没有把协议层作为主接口使用。

## 当前固件行为详解

### 启动流程

当前 `main()` 的启动顺序大致是：

1. `HAL_Init()`
2. `SystemClock_Config()`
3. 初始化 GPIO
4. 初始化 I2C1
5. 初始化 SPI1
6. 初始化 USART1
7. 复位软件内部 tick 变量
8. 初始化 3 个 AD7177
9. 打印启动信息
10. 进入周期性采样与上报循环

### 当前 AD7177 配置

每个 AD7177 目前使用的配置基本一致：

- SPI 总线：`SPI1`
- CRC：关闭
- 超时：`100 ms`
- 参考电压：`5000.0 mV`
- 使用外部参考：开启
- 使用逻辑通道：`0`
- 输入对：`AIN3` 正端，`AIN4` 负端
- 双极性模式：开启
- 滤波阶数：`0`
- 输出数据率：`AD7177_ODR_59SPS`

### 运行时循环逻辑

当前主循环非常直接，主要为了方便 bring-up：

- 每 `20 ms` 读一次 ADC，也就是 `50 Hz`
- 更新 3 路电压缓存值
- 每轮都通过串口输出一次
- 预留了后续状态管理和事件处理接口

这样的结构非常适合当前板级联调，但还不是最终项目架构。

## 构建环境

### 当前已验证的工具链

根据仓库中的最新构建日志：

- IDE：`Keil uVision V5.24.2`
- 工具链：`MDK-ARM Plus 5.24.1`
- 编译器：`Armcc V5.06 update 5`

当前记录的最新构建结果为：

- `0` 个错误
- `3` 个警告

对应日志文件：
[MDK-ARM/ProjectFusionBoard/ProjectFusionBoard.build_log.htm](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/MDK-ARM/ProjectFusionBoard/ProjectFusionBoard.build_log.htm:1)

### 如何打开工程

如果使用 Keil：

- 打开 [MDK-ARM/ProjectFusionBoard.uvprojx](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/MDK-ARM/ProjectFusionBoard.uvprojx:1)

如果需要回看或重新生成 CubeMX 配置：

- 打开 [ProjectFusionBoard.ioc](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/ProjectFusionBoard.ioc:1)

### 推荐的本地构建流程

1. 使用 Keil 打开 `MDK-ARM/ProjectFusionBoard.uvprojx`
2. 选择目标 `ProjectFusionBoard`
3. 执行 Rebuild
4. 使用 ST-Link 或 Keil 内置下载方式烧录
5. 打开串口工具，连接 `USART1`
6. 设置波特率 `115200`
7. 观察启动信息与周期输出

## 串口联调说明

### 正常启动时应看到的内容

如果初始化成功，串口应看到类似输出：

```text
frequency=50Hz
3xAD7177 test start
```

### 初始化失败时的输出

如果驱动初始化失败，程序会进入错误循环，并持续输出：

```text
INIT_ERROR:<code>
```

当前 `main.c` 里定义的错误码有：

- `2`：SPI / ADC 初始化失败
- `3`：UART 初始化失败

### 严重错误时的输出

如果程序进入 `Error_Handler()`，会持续输出：

```text
FATAL_ERROR
```

### 推荐的串口观察工具

可使用任意串口终端或带绘图能力的上位机工具，例如：

- SSCOM
- Serial Studio
- VS Code 串口插件
- 自写 Python / MATLAB 绘图脚本

由于当前输出是固定格式的三路浮点数文本流，所以非常适合做实时波形观察。

## 当前需要特别注意的几点

### 1. 当前主程序是联调程序，不是最终应用

虽然现在仓库主分支已经建立并上传，但从功能角度看，当前固件仍然属于 `3x AD7177` 联调状态，不应误认为这是最终产品逻辑。

### 2. MPU6050 / Mahony 已有基础，但尚未重新接回主循环

驱动和理论分析都已经在仓库中，但当前 `main()` 没有真正执行这一条链路。

### 3. `SPI2` 目前未使用

`MX_SPI2_Init()` 还在源码中，但当前主程序没有调用，Keil 的构建日志也因此给出未使用警告。

### 4. 驱动层文件会比当前实际使用的多

这是 STM32Cube/Keil 工程的常见情况。很多 HAL 文件和驱动接口是保留在工程里的，但并不代表当前版本都在实际运行。

### 5. 涉及时序和采样率时需要回到实板验证

代码中已经明确了循环频率和 ODR 关系，但任何真正敏感的时序结论仍然要以实板测试为准。

## 新同学建议接手路径

如果你是第一次接手这个项目，建议按下面顺序进入：

1. 先完整读完这份 README
2. 打开 `main.c`，理解当前三路 AD7177 的主循环
3. 阅读 `ad7177.h/.c`，弄清楚当前 ADC 驱动框架
4. 在 Keil 中重新编译，确认本地环境可以复现当前构建结果
5. 烧录到板子上，通过串口确认输出是否正常
6. 在实板上确认三路 ADC 数据是否稳定、是否符合预期
7. 如果你后续要接 IMU / 姿态方向，再继续读 `Mahony_Formula_Derivation.txt`

## 建议的下一步工作

基于当前仓库状态，最自然的后续工作包括：

- 验证三路 AD7177 的电气连接与采样稳定性
- 对照数据手册复查 AD7177 寄存器配置
- 把当前 `main.c` 中的 bring-up 逻辑整理成更清晰的采集层
- 在 ADC 路径稳定后重新接回 MPU6050 逻辑
- 比较当前数据链路和未来 Mahony 姿态链路的整合方式
- 持续补充硬件说明和联调记录

## 仓库管理说明

当前仓库已经完成 Git 初始化，并已上传到 GitHub。  
仓库中的 `.gitignore` 已经过滤常见的 Keil 构建产物、用户私有配置和临时文件。

如果你后续新增了本地调试脚本、二进制文件、日志或测试导出数据，请确认它们是否应该加入 `.gitignore`。

## 这份 README 暂未覆盖的内容

这份文档是基于当前仓库内容整理出来的，因此暂时还没有覆盖以下内容：

- 完整硬件原理图
- 更详细的板级接线说明
- 标定流程
- 量产或部署流程
- 测试工装说明

如果这些资料存在于仓库外部，建议后续把路径或说明补充到 README 中。

## 快速索引

- 主程序入口：[Core/Src/main.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Core/Src/main.c:1)
- Keil 工程：[MDK-ARM/ProjectFusionBoard.uvprojx](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/MDK-ARM/ProjectFusionBoard.uvprojx:1)
- CubeMX 配置：[ProjectFusionBoard.ioc](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/ProjectFusionBoard.ioc:1)
- AD7177 驱动：[Drivers/SPI/ad7177.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/SPI/ad7177.c:1)
- MPU6050 驱动：[Drivers/I2C/mpu6050.c](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Drivers/I2C/mpu6050.c:1)
- Mahony 分析文档：[Mahony_Formula_Derivation.txt](/C:/Users/lllfunway/Desktop/Fusion%20board/ProjectFusionBoard/Mahony_Formula_Derivation.txt:1)
