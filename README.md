# 声语信使

电子系统课程设计项目：使用声波在终端之间传输固定 48 字符短信息。当前主工程基于 STM32F411RET6 第二版收发一体板，采用半双工 4-FSK、Goertzel 检测、RS 纠错和 CRC12 校验，并支持标准、多机和高频隐蔽三种通信模式。

> **当前答辩版本：`v1.0.0-demo`（2026-07-20）**
>
> 当前代码入口：[`software/stm32_second_board/`](software/stm32_second_board/)；第一版板卡和一位数字工程均标记为历史资料。

## 快速导航

| 内容 | 状态 | 入口 |
| --- | --- | --- |
| 第二版三模式固件 | **当前主工程** | [`software/stm32_second_board/`](software/stm32_second_board/) |
| 第二版原理图与 PCB | **当前硬件** | [`hardware/syxs_board/`](hardware/syxs_board/) |
| 7 月 13 日起非代码日志 | **当前整理版** | [`logs/工作日志-9组-范嘉毅-24271097.md`](logs/工作日志-9组-范嘉毅-24271097.md) |
| 课程任务书和设计资料 | 正式/阶段资料 | [`docs/`](docs/) |
| 第一版与分立收发工程 | 历史测试工程 | [`software/README.md`](software/README.md) |
| 早期串口与一位数字记录 | 历史调试记录 | [`logs/serial/`](logs/serial/) |

## 当前主工程

主工程位于 `software/stm32_second_board/`，主要功能如下：

- 接收：`PC4 / ADC1_IN14` 经 MCP6S21 PGA 输入；普通模式 16 kHz 采样，隐蔽模式 20 kHz 采样。
- 发送：MCP4921 SPI DAC 输出，32 kHz 更新率，经滤波和 PAM8302 功放驱动扬声器。
- 时钟：25 MHz HSE 经 PLL 产生 96 MHz 系统时钟。
- 协议：48 字符负载 + CRC12，使用两块 RS(35,25) GF(64) 编码；完整消息帧约 `19.941 s`。
- 交互：4×4 键盘编辑，OLED 显示模式、消息和状态，Flash 保存最近 5 条消息。
- 指示灯：LED3 在校准或消息发送期间点亮，蓝灯表示接收完成。
- 开机画面：显示“声语信使”及全体成员姓名、学号，持续 2 秒后进入模式选择。

最新可烧录文件：`software/stm32_second_board/MDK-ARM/test2/test2.hex`。

## 通信模式

开机画面结束后按键选择模式；通信双方必须选择相同模式。

| 按键 | 模式 | 数据频率 | 同步频率 | PGA/校准 |
| --- | --- | --- | --- | --- |
| `A` | STANDARD | 1500/2500/3500/4500 Hz | 2000 Hz | 启动校准，自动增益控制 |
| `B` | MULTI-NODE | 1500/2500/3500/4500 Hz | 2000 Hz | 不发送校准，PGA 固定 x2 |
| `C` | HIDDEN | 6400/6550/6850/7000 Hz | 6700 Hz | 启动校准，自动增益控制 |

多机模式支持 3 个节点：接收状态按 `1/2/3` 设置本机编号，按 `D` 在广播和 N1/N2/N3 目的地址间切换。发送帧携带源地址、目的地址，CRC 同时保护地址和文本。

## 目录说明

- [`docs/`](docs/)：课程资料、器件规格书和历史专题报告，详细状态见目录内索引。
- [`hardware/`](hardware/)：第二版整板、滤波器和单片机板资料，当前入口为 `syxs_board/`。
- [`logs/`](logs/)：成员工作日志、7 月 13 日起非代码总结和早期串口原始记录。
- [`software/`](software/)：当前第二版主工程、共享驱动及历史测试工程。

## 构建与烧录

- IDE：Keil MDK-ARM V5。
- 配置工具：STM32CubeMX。
- 工程入口：`software/stm32_second_board/MDK-ARM/test2.uvprojx`。
- HAL/CMSIS：第二版工程复用 `software/stm32_receive/Drivers`，克隆完整仓库后即可解析相对路径。
- 调试串口：USART2，115200 8N1。
- 烧录：使用 ST-Link 烧录工程生成的 HEX，或直接使用仓库内已提交的 `test2.hex`。

## 答辩版校验

- 版本记录：[`CHANGELOG.md`](CHANGELOG.md)
- 发布页面：[`v1.0.0-demo`](https://github.com/epidimobife96-rgb/shengyu-xinshi/releases/tag/v1.0.0-demo)
- Keil 构建：`0 Error(s), 3 Warning(s)`，警告均来自 HAL 库未使用参数。
- 程序大小：Code=77100，RO-data=10192，RW-data=120，ZI-data=7384。
- HEX SHA-256：`80D15BDB51B6F944610941480073471BF544B315394B95FF26E95740F8AD2A9D`

## 关键文档

- **当前：** [第二版板卡工程说明](software/stm32_second_board/README.md)
- **当前：** [第二版硬件资料与实物调整](hardware/syxs_board/README.md)
- **当前：** [7 月 13 日起非代码工作日志](logs/工作日志-9组-范嘉毅-24271097.md)
- **历史：** [硬件检查记录](docs/硬件检查记录.md)
- **历史：** [一位数字调试记录汇总](docs/调试记录汇总.md)
- **历史：** [前导码与同步问题总结报告](docs/前导码与同步相关代码问题总结报告.docx)
