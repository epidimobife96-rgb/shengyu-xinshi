# 第一版板卡接收端

> **状态：历史测试工程。** 当前答辩主工程位于 [`../stm32_second_board/`](../stm32_second_board/)。本目录用于保留第一版板卡验证过程。

STM32F411 第一版板卡 4-FSK 接收端测试工程，包含接收解码、同步频点检测、OLED/串口调试等逻辑。

- 采样率：`16 kHz`
- 数据频点：`1500 / 2500 / 3500 / 4500 Hz`
- 同步频点：`2000 Hz`
- 工程来源：`D:\stm32\test1`

Keil 工程复用 `software/stm32_receive/Drivers` 下的 HAL/CMSIS 驱动，避免重复提交驱动库。
