## MSPM0G3507 TB6612 双电机PID速度控制系统

### 项目概述

本项目是基于TI MSPM0G3507 LaunchPad开发的双电机速度闭环控制系统，使用TB6612电机驱动芯片实现电机控制。系统采用增量式PI算法实现速度闭环控制，通过编码器反馈电机转速，实时调整PWM输出。

### 硬件平台

| 组件   | 型号            | 说明                |
| ---- | ------------- | ----------------- |
| MCU  | MSPM0G3507    | TI MSPM0系列32位微控制器 |
| 开发板  | LP-MSPM0G3507 | TI LaunchPad开发板   |
| 电机驱动 | TB6612        | 双H桥电机驱动芯片         |
| 编码器  | -             | 正交编码器（外部中断计数）     |

### 引脚分配

#### 电机控制引脚

| 信号   | 引脚             | 功能说明      |
| ---- | -------------- | --------- |
| AIN1 | PA12           | 电机A正向控制   |
| AIN2 | PA13           | 电机A反向控制   |
| BIN1 | PA14           | 电机B正向控制   |
| BIN2 | PA15           | 电机B反向控制   |
| PWMA | PB0 (TIMA1 C0) | 电机A PWM信号 |
| PWMB | PB1 (TIMA1 C1) | 电机B PWM信号 |

#### 编码器引脚

| 信号  | 引脚   | 功能说明      |
| --- | ---- | --------- |
| E1A | PA8  | 电机A编码器通道A |
| E1B | PA9  | 电机A编码器通道B |
| E2A | PB14 | 电机B编码器通道A |
| E2B | PB15 | 电机B编码器通道B |

#### 其他外设引脚

| 信号    | 引脚   | 功能说明     |
| ----- | ---- | -------- |
| KEY   | PB21 | 按键输入     |
| LED   | PB22 | LED指示灯输出 |
| SWCLK | PA20 | SWD调试时钟  |
| SWDIO | PA19 | SWD调试数据  |

#### 灰度传感器引脚

| 信号 | 引脚   | 功能说明       |
| -- | ---- | ---------- |
| S1 | PA27 | 灰度传感器1（上拉） |
| S2 | PA26 | 灰度传感器2（上拉） |
| S3 | PA25 | 灰度传感器3（上拉） |
| S4 | PA24 | 灰度传感器4（上拉） |

### 软件架构

#### 模块说明

| 模块   | 文件                     | 功能说明                     |
| ---- | ---------------------- | ------------------------ |
| 主程序  | `empty.c`              | 系统初始化、主循环、定时器中断处理、速度闭环控制 |
| 板级支持 | `board.c/h`            | 延时函数（us/ms级）、UART相关（已注释） |
| 电机驱动 | `Hardware/motor.c/h`   | 增量式PI速度控制算法、PWM输出设置      |
| 编码器  | `Hardware/encoder.c/h` | 正交编码器外部中断计数（当前中断处理已注释）   |
| 按键   | `Hardware/key.c/h`     | 按键扫描与消抖                  |
| LED  | `Hardware/led.c/h`     | LED亮灭、闪烁控制               |

#### 核心数据结构

```c
int32_t Get_Encoder_countA, encoderA_cnt, PWMA;  // 电机A编码器计数、速度、PWM值
int32_t Get_Encoder_countB, encoderB_cnt, PWMB;  // 电机B编码器计数、速度、PWM值
float Velocity_Kp = 1.0, Velocity_Ki = 0.5;      // PI控制器参数
```

### PID控制算法

本系统采用**增量式PI算法**，公式如下：

```
Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
u(k) = u(k-1) + Δu(k)
```

其中：

- `e(k)` = 目标速度 - 当前速度（编码器计数）
- 输出限幅：`[-3600, 3600]`（对应Timer Compare值范围）
- 目标速度：硬编码为 `-15`（编码器计数值/周期）

#### 控制流程

1. **定时器中断**（TIMA0，周期5ms）：定时读取编码器计数值
2. **速度计算**：`encoderA_cnt = -Get_Encoder_countA`（负号表示电机A安装方向相反）
3. **PI控制**：调用 `Velocity_A()` / `Velocity_B()` 计算PWM值
4. **PWM输出**：调用 `Set_PWM()` 设置电机方向和占空比（TIMA1输出PWM）
5. **串口输出**：主循环中通过printf输出编码器计数值

### 项目目录结构

```
MSPM0G3507_TB6612/
├── empty.c                    # 主程序入口
├── board.c/h                  # 板级支持包
├── ti_msp_dl_config.c/h       # SysConfig自动生成配置
├── Hardware/
│   ├── motor.c/h              # 电机驱动与PID控制
│   ├── encoder.c/h            # 编码器接口
│   ├── key.c/h                # 按键处理
│   └── led.c/h                # LED控制
├── keil/
│   ├── empty_LP_MSPM0G3507_nortos_keil.uvprojx  # Keil工程文件
│   ├── empty_LP_MSPM0G3507_nortos_keil.uvoptx   # Keil工程配置
│   ├── Objects/               # 编译输出目标文件
│   └── Listings/              # 编译列表文件
├── source/
│   ├── third_party/           # 第三方库（CMSIS、mcuboot）
│   └── ti/                    # TI官方驱动库（driverlib）
└── empty.syscfg               # SysConfig配置文件
```

### 构建与烧录

#### 开发环境

- **IDE**: Keil MDK v5.x
- **编译器**: ARM Compiler (AC5/AC6)
- **调试器**: XDS-110（LaunchPad内置）

#### 编译步骤

1. 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
2. 选择目标配置（Debug/Release）
3. 点击"Build"或"Rebuild"编译项目
4. 编译成功后生成 `.axf` 和 `.hex` 文件

#### 烧录步骤

1. 将LaunchPad通过USB连接到电脑
2. 确保SWD跳线设置正确（J101 13:14 ON, J101 15:16 ON）
3. 在Keil中点击"Download"烧录程序
4. 点击"Start/Stop Debug Session"开始调试

### 使用说明

1. **硬件连接**：将TB6612驱动板与LaunchPad按引脚分配表连接
2. **供电**：为电机和驱动板提供适当电源
3. **运行**：烧录程序后，电机将以目标速度运行
4. **调试**：通过UART串口（默认配置）查看编码器计数值输出

### 已知问题与限制

1. **编码器中断未启用**：`Hardware/encoder.c` 中的 `GROUP1_IRQHandler` 函数体被完全注释，当前编码器计数功能未实现
2. **UART代码已注释**：`board.c` 中的UART收发函数、`fputc`重定向等代码已被注释，printf输出需确保SysConfig已配置UART
3. **PID参数固定**：速度环PI参数（Kp=1.0, Ki=0.5）为固定值，未提供运行时调整接口
4. **目标速度硬编码**：目标速度 `-15` 硬编码在定时器中断处理函数中

### 代码修改建议

#### 启用编码器中断

取消 `Hardware/encoder.c` 中 `GROUP1_IRQHandler` 函数体的注释，并在 `empty.c` 中启用相关中断：

```c
// 编码器A使用GPIOA中断，编码器B使用GPIOB中断
NVIC_EnableIRQ(GPIOA_INT_IRQn);
NVIC_EnableIRQ(GPIOB_INT_IRQn);
```

#### 调整PID参数

修改 `Hardware/motor.c` 中的参数值：

```c
float Velocity_Kp = 2.0, Velocity_Ki = 0.8;  // 根据实际系统调整
```

#### 修改目标速度

修改 `empty.c` 中定时器中断处理函数的目标速度值：

```c
PWMA = Velocity_A(20, encoderA_cnt);  // 修改第一个参数为目标速度
```

### 参考文档

- [LP-MSPM0G3507 LaunchPad User's Guide](https://www.ti.com/lit/slau873)
- [MSPM0G350x Datasheet](https://www.ti.com/product/MSPM0G3507)
- [TB6612 Datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20150318.pdf)
- [TI MSPM0 SDK Documentation](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/)

### 版本信息

- **项目版本**: V1.0
- **更新日期**: 2024-07-19
- **源码来源**: WHEELTEC (wheeltec.net)

***

*All rights reserved.*
