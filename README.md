# shengyuxinshi

电子系统课程设计项目：声语信使。

目标是使用声波在两个终端之间传输短信息，当前阶段重点为 STM32F411 4-FSK 声学通信接收端、短信息编辑器硬件资料和上位机测试工具整理。

## 目录

- `docs/`：课程设计任务书等项目文档。
- `hardware/`：硬件原理图、网表、滤波器和单片机核心板资料。
- `hardware/message_editor/`：短信息编辑器相关原理图和网表。
- `software/stm32_receive/`：STM32F411 接收端 CubeMX/HAL/Keil 工程。
- `software/python_tools/`：Python 声音测试与发送工具。
- `images/`：截图、测试图片和展示素材。
- `logs/`：工作日志等过程记录。

## 当前实现

- 4-FSK 频率：1500 Hz、2500 Hz、3500 Hz、4500 Hz。
- STM32 接收端使用 ADC + DMA 采样，Goertzel 算法检测频率。
- OLED 显示接收状态和识别结果。
- Python 工具用于生成和播放测试声波。

