# 第一版板卡发送端

STM32F411 第一版板卡 4-FSK 发送端测试工程。

- PWM 输出：`PA0 / TIM2_CH1`
- 功放静音控制：`PB12`
- 串口调试：`USART2 115200 8N1`
- 工程来源：`D:\stm32\test1_tx`

Keil 工程复用 `software/stm32_receive/Drivers` 下的 HAL/CMSIS 驱动，避免重复提交驱动库。
