# 软件工程索引

## 当前答辩主工程

[`stm32_second_board/`](stm32_second_board/) 是第二版 STM32F411RET6 收发一体板的当前固件，包含标准、多机和高频隐蔽三种模式。源码、Keil 工程、CubeMX 配置和可直接烧录的 HEX 均以该目录为准。

| 入口 | 路径 |
| --- | --- |
| 工程说明 | [`stm32_second_board/README.md`](stm32_second_board/README.md) |
| 主源码 | `stm32_second_board/Core/Src/main.c` |
| Keil 工程 | `stm32_second_board/MDK-ARM/test2.uvprojx` |
| CubeMX 配置 | `stm32_second_board/test2.ioc` |
| 答辩版 HEX | `stm32_second_board/MDK-ARM/test2/test2.hex` |

## 共享依赖

`stm32_second_board` 的 Keil 工程通过相对路径复用 `stm32_receive/Drivers` 中的 STM32 HAL/CMSIS。虽然 `stm32_receive` 本身属于早期接收工程，但其 `Drivers` 目录仍是当前主工程的构建依赖，不能单独删除。

## 历史与辅助工程

| 目录 | 用途 | 状态 |
| --- | --- | --- |
| `stm32_first_board_rx/` | 第一版整板接收测试 | 历史工程 |
| `stm32_first_board_tx/` | 第一版整板发送测试 | 历史工程 |
| `stm32_receive/` | 早期分立接收端及共享驱动 | 历史工程 / 当前依赖 |
| `stm32_send/` | 黑丸开发板一位数字发送测试 | 历史工程 |
| `python_tools/` | PC 声音播放和原型验证脚本 | 辅助工具 |
