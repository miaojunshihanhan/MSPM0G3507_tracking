# MSPM0G3507 Tracking Car

## 项目介绍

本项目是基于 TI MSPM0G3507 的智能车循迹控制程序，支持双电机速度闭环、8 路灰度传感器循迹、直线与曲线跟踪、直角转弯识别、全白延时停车保护，以及 UART 串口调参和 VOFA 数据输出。

系统主控为 MSPM0G3507，电机由 TB6612 驱动，编码器用于速度反馈，灰度传感器用于外环循迹。当前主程序采用裸机循环结构，1ms 定时中断负责按键、循迹控制节拍和状态灯调度，主循环负责串口命令处理与速度环任务执行。

## 目录结构

```text
.
├── empty.c                         # 程序入口、系统初始化、中断分发与主循环
├── board.c / board.h               # 板级初始化、延时、串口基础接口
├── ti_msp_dl_config.c / .h          # SysConfig 生成的外设配置
├── Hardware/
│   ├── tracking.c / tracking.h      # 速度闭环、循迹外环、串口调参与状态输出
│   ├── status_display.c / .h        # OLED 状态显示初始化
│   ├── motor.c / motor.h            # TB6612 电机驱动与 PWM 输出
│   ├── encoder.c / encoder.h        # 编码器计数接口
│   ├── key.c / key.h                # 按键扫描
│   ├── oled.c / oled.h              # OLED 驱动
│   └── w25q128.c / w25q128.h        # 外部 Flash 参数保存
├── keil/
│   └── empty_LP_MSPM0G3507_nortos_keil.uvprojx
└── source/                          # TI SDK、CMSIS 与相关依赖
```

## 主要功能

- 10ms 速度闭环控制，支持左右轮独立目标速度。
- 灰度传感器循迹外环，输出左右轮差速目标。
- 中线稳定跟踪，支持较大偏差时的连续修正。
- 直角元素识别，朝检测到循迹线的一侧转弯。
- 全白状态延时保护，连续全白超过 5s 后停车。
- UART 命令调参，支持速度环、循迹环和 VOFA 输出配置。
- W25Q128 保存速度环 PID、前馈和 PWM 参数。

## 构建环境

- IDE：Keil MDK 5
- 编译器：ARM Compiler 6 / armclang
- 工程文件：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- 目标芯片：MSPM0G3507
- 下载调试：DAPLink / SWD
- 串口：115200 baud，8N1

## 构建与烧录

1. 使用 Keil 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
2. 选择目标 `MSPM0G3507_Project`。
3. 执行 Build 或 Rebuild 生成 `.axf` 和 `.hex`。
4. 通过 DAPLink 连接目标板 SWD 接口。
5. 点击 Download 烧录程序。
6. 使用串口工具连接调试串口，波特率设置为 `115200`。

## 串口命令

```text
RUN base       启动循迹，base 为基础速度
TRK 0/1/2      设置循迹模式：关闭 / 原地校正 / 前进循迹
TBASE n        设置循迹基础速度
TMAX n         设置循迹最大转向输出
TMIN n         设置循迹最小转向输出
TDB n          设置循迹死区
TKP/TKI/TKD n  设置循迹环 PID 参数
S l r          设置左右轮速度目标
L n / R n      单独设置左轮或右轮速度目标
KP/KI/KD/FF n  设置速度环参数
MINPWM n       设置最小 PWM
SLEW n         设置 PWM 斜率限制
VOFA 0/1       关闭或开启 VOFA 数据输出
RATE ms        设置 VOFA 输出周期
LINE           输出当前灰度传感器状态
STAT           输出当前控制状态
STOP           停车
```
