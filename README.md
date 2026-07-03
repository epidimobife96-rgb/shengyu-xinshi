# 声语信使

电子系统课程设计项目：使用声波在两个终端之间传输短信息。当前方案采用 4-FSK，把每个码元映射为 2 bit，通过话筒模拟前端、STM32 接收算法和扬声器发送端完成短距离通信验证。

## 当前状态

- 接收端：STM32F411RET6，ADC + DMA 以 16 kHz 采样，每 20 ms 处理 320 点，Goertzel 检测 1500/2500/3500/4500 Hz。
- 发送端：STM32F411CEU6 黑丸调试工程，串口输入数字或 `00/01/10/11`，输出对应 4-FSK 声音/PWM 波形。
- 协议：已实现一位数字接收，包含前导、起始、数据、反码校验和结束验证帧。
- 调试输出：OLED 和 USART2 均可显示接收状态；静音串口日志已降频，避免有效帧被刷掉。

## 4-FSK 映射

| bits | 频率 |
| --- | --- |
| `00` | 1500 Hz |
| `01` | 2500 Hz |
| `10` | 3500 Hz |
| `11` | 4500 Hz |

## 一位数字协议

帧结构：

`00 01 10 11` + `01 10` + `data0 data1` + `~data0 ~data1` + `10 11`

数字编码：

| 数字 | 码元 |
| --- | --- |
| 0 | `00 01` |
| 1 | `00 10` |
| 2 | `00 11` |
| 3 | `01 00` |
| 4 | `01 10` |
| 5 | `01 11` |
| 6 | `10 00` |
| 7 | `10 01` |
| 8 | `10 11` |
| 9 | `11 00` |

## 目录

- `docs/`：课程任务书、器件资料、硬件检查记录、调试记录汇总。
- `hardware/filter/`：接收端模拟滤波器 Multisim 文件和网表。
- `hardware/mcu_board/`：单片机核心板原理图、网表和工程资料。
- `hardware/syxs_board/`：声语信使整板原理图和网表。
- `hardware/speaker_filter/`：发送端喇叭/功放前滤波器网表。
- `logs/`：课程工作日志和串口调试原始记录。
- `software/stm32_receive/`：STM32F411 接收端 CubeMX/HAL/Keil 工程。
- `software/stm32_send/`：STM32F411 发送端 CubeMX/HAL/Keil 工程。
- `software/python_tools/`：PC 端声音播放和协议测试工具。

## 引脚版本说明

当前有两个硬件阶段，烧录前必须按实际板子确认引脚。

| 模块 | 面包板/调试工程 | 整板原理图 |
| --- | --- | --- |
| 发送 PWM | `PA8 / TIM1_CH1` | `PA0` 进入发送滤波/功放链路 |
| 接收 ADC | `PA1 / ADC1_IN1` | `PC4 / ADC1_IN14` |
| 串口调试 | `USART2 PA2/PA3` | 保持按工程配置连接 |
| OLED | I2C 接口 | 按整板原理图连接 |

如果使用整板，应把发送端工程从 `PA8/TIM1_CH1` 改到 `PA0` 可用的定时器通道，并把接收端 ADC 从 `ADC_CHANNEL_1` 改为 `ADC_CHANNEL_14 / PC4`。

## 构建

- IDE：Keil MDK-ARM V5。
- 配置工具：STM32CubeMX。
- HAL/CMSIS 驱动：接收端工程保留完整 `Drivers`；发送端 Keil 工程复用 `software/stm32_receive/Drivers`，避免重复提交一份 HAL/CMSIS。
- 烧录调试：ST-Link 烧录；串口调试使用 USB 转 TTL，115200 8N1。

## 关键文档

- [硬件检查记录](docs/硬件检查记录.md)
- [调试记录汇总](docs/调试记录汇总.md)
- [发送端工程说明](software/stm32_send/README.md)
