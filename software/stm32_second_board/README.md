# 第二版板卡收发工程

STM32F411 第二版板卡 4-FSK 收发测试工程，面向带 SPI DAC 和 PGA 的新版硬件。

- 采样率：`16 kHz`
- 数据频点：`1500 / 2500 / 3500 / 4500 Hz`
- 同步频点：`2000 Hz`
- 支持 SPI DAC/PGA 相关控制逻辑
- 工程来源：`D:\stm32\test2`

Keil 工程复用 `software/stm32_receive/Drivers` 下的 HAL/CMSIS 驱动，避免重复提交驱动库。
