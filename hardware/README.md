# 硬件资料索引

## 当前第二版整板

当前实物和答辩使用的硬件资料位于 [`syxs_board/`](syxs_board/)。其中：

- `声语信使.pdf`：第二版整板原理图，查看和答辩说明优先使用。
- `声语信使网表.net`：第二版整板网表，用于核对网络连接。
- `声语信使第二版_含PCB.epro2`：EasyEDA Pro 原理图与 PCB 工程。
- `Altium_声语信使第二版_含PCB.zip`：Altium 导出工程包。

实物调试中发生的器件值变化见 [`syxs_board/README.md`](syxs_board/README.md)，排障时应同时核对原理图标称值和板上实装值。

## 其他目录

| 目录 | 内容 | 状态 |
| --- | --- | --- |
| `filter/` | 接收模拟滤波器 Multisim 设计与网表 | 早期/辅助仿真资料 |
| `mcu_board/` | 单片机核心板、网表及短信息编辑器资料 | 分板设计资料 |
| `speaker_filter/` | 功放前喇叭滤波器设计与网表 | 辅助仿真资料 |
| `syxs_board/` | 第二版收发一体板原理图、网表和 PCB | **当前硬件** |

> `filter/滤波器设计.ms14 (Security copy)` 是 Multisim 自动安全副本，仅用于工程恢复，不作为当前设计入口。
