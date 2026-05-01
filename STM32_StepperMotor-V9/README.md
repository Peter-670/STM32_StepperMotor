# STM32 Stepper Motor Control System V9

> 基于 STM32F103C8T6 + DRV8825 的步进电机精密位移控制系统，面向激光器谐振腔调谐应用，支持串口指令控制、微米级定位、限位保护、位移累计等功能。

---

## 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [硬件详解](#硬件详解)
- [软件详解](#软件详解)
- [串口通信协议](#串口通信协议)
- [编译与烧录](#编译与烧录)
- [使用说明](#使用说明)
- [自定义修改指南](#自定义修改指南)
- [文件结构](#文件结构)
- [附录](#附录)

---

## 项目概述

本项目为**激光自动调谐装置**的核心运动控制单元，通过步进电机驱动电动滑台实现精密位移，控制激光器谐振腔长与 PPLN 晶体极化周期的同步匹配。

### 核心功能

- 串口指令控制：3 位数字快捷指令 + 微米级精确位移指令（A/B 方向）
- 三档速度可调（Slow / Mid / Fast）
- 6400 脉冲/圈细分，导程 1mm/圈，单步 0.15625μm
- 双限位（A/B 方向独立锁定）+ 原点检测
- 步进中快速限位检测 + 双样本去抖滤波
- 位移累计追踪（int32_t 步数），0.1μm 显示精度
- 限位触发后反向自动可用，无需手动复位
- 报警信息、位置查询等实时串口反馈

---

## 系统架构

```text
+--------------+    UART (115200)    +------------------+
|  PC / 串口助手 | <-----------------> | STM32F103C8T6    |
+--------------+                     |                  |
                                     | +--------------+ |
                                     | | Cmd Parser   | |
                                     | | 3-digit / AB | |
                                     | +------+-------+ |
                                     |        |          |
                                     | +------v-------+ |
                                     | | Motor Ctrl   | |
                                     | | Step/Dir/Pos | |
                                     | +------+-------+ |
                                     |        |          |
                                     | +------v-------+ |
                                     | | Limit Check  | |
                                     | | Fast+Debounce| |
                                     | +--------------+ |
                                     +----+---+----+----+
                                          |   |    |
                                    DIR   STEP LIMIT
                                     |     |     |
                              +-------v-----v-----v----+
                              |       DRV8825          |
                              |   + Motor + Sensors    |
                              +------------------------+
```

### 关键设计决策

| 决策 | 原因 |
|------|------|
| GPIO 位带产生脉冲（非定时器 PWM） | 步数精确可控，`Step(N, T)` 发完即停 |
| 软件忙等待延时（非中断驱动） | 简单可靠，无中断嵌套风险 |
| 步数累加位移（非 float 累积） | 消除浮点舍入误差 |
| 方向独立锁定（非全局 alarm_lock） | 限位触发后反向立即可用，不卡死 |
| 两层限位检测（快速 + 去抖） | 快速检测保安全，去抖滤波防误触 |

---

## 硬件详解

### 硬件组成

| 组件 | 型号/规格 | 用途 |
|------|-----------|------|
| 主控芯片 | STM32F103C8T6 (Cortex-M3, 72MHz) | 指令解析、脉冲生成、逻辑控制 |
| 驱动芯片 | DRV8825 | 步进电机功率驱动，2.5A 峰值 |
| 电动滑台 | 鑫飞 FEX4015-LBN | 执行机构，滚珠丝杆传动 |
| 串口模块 | USB-TTL (CH340) | PC 与主控通信 |

### 关键硬件参数

| 参数 | 数值 |
|------|------|
| 滑台行程 | +/-7.5mm |
| 细分设置 | 1/6400 |
| 导程 | 1mm/圈 |
| 单步位移 | 0.15625um |
| 最大速度 | 20mm/s |
| 重复定位精度 | <=+/-1.5um |

### 引脚分配

| 引脚 | 功能 | 模式 | 说明 |
|------|------|------|------|
| PA9 | USART1_TX | AF Push-Pull | 串口发送 |
| PA10 | USART1_RX | Input Floating | 串口接收 |
| PA0 | TIM2_CH1 | AF Push-Pull | PWM 输出（备用） |
| PB10 | DIR | Output Push-Pull | A/B 方向控制 |
| PB11 | STEP | Output Push-Pull | 脉冲输出 |
| PB3 | A 限位 (CW) | Input Pull-Down | 高电平有效 |
| PB4 | B 限位 (CCW) | Input Pull-Down | 高电平有效 |
| PB5 | 原点 (ORG) | Input Pull-Down | 高电平有效 |

### 时钟配置

- HSE: 8MHz 晶振 -> PLL x9 -> SYSCLK = 72MHz
- AHB (HCLK) = 72MHz, APB1 = 36MHz, APB2 = 72MHz
- SysTick: 72MHz / 1000 = 72kHz -> HAL_Delay(1) = 1ms

---

## 软件详解

### 速度档位

| 档位 | half_T 延时值 | 近似脉冲周期 | 适用场景 |
|------|---------------|-------------|----------|
| 1 (Slow) | 500 | ~56us | 精确定位 |
| 2 (Mid) | 250 | ~28us | 常规移动（A/B 指令默认） |
| 3 (Fast) | 180 | ~20us | 快速移动 |

### 位移换算

```text
6400 脉冲 = 1 圈 = 1 mm = 1000 um

1 步 = 1000/6400 = 0.15625 um

um -> 步数: steps = microns * 32 / 5
步数 -> 0.1um: 0.1um = steps * 25 / 16
```

### 限位保护模型

```text
                    +-------------+
          +-------->|   NORMAL    |<--------+
          |         | (双向可用)   |         |
          |         +------+------+         |
          |                |                |
     A限位释放          A限位触发         B限位释放
          |                |                |
          |         +------v------+         |
          +---------|  A-LOCKED   |         |
                    | (仅B向可用)  |         |
                    +-------------+         |
                                            |
          +-------------+                   |
          |  B-LOCKED   |<------------------+
          | (仅A向可用)  |
          +-------------+
```

**检测层级：**

| 层级 | 位置 | 去抖 | 延迟 | 作用 |
|------|------|------|------|------|
| 快速检测 | `Step()` 每 100 步 | 2 次连续读取 | ~200ns | 运动中紧急停止 |
| 去抖检测 | `Limit_Switch_Check()` 主循环 | 2ms 间隔 x3 次 | ~30ms | 准确判定并锁方向 |

**原点 (ORG)：** 仅信息提示 + 位置清零，不停止电机，不锁定方向。

### 关键全局变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `motor_position_steps` | `int32_t` | 当前位移（步数，A+ B-） |
| `motor_speed` | `uint8_t` | 速度 1-3 |
| `motor_dir` | `uint8_t` | 0=A 方向, 1=B 方向 |
| `limit_a_locked` | `uint8_t` | A 方向锁（限位触发后置 1，释放自动清 0） |
| `limit_b_locked` | `uint8_t` | B 方向锁 |
| `motor_stop_flag` | `uint8_t` | 紧急停止标志（阻止位置更新） |
| `motor_busy` | `uint8_t` | 电机运动中标志（保护原点位置重置） |

---

## 串口通信协议

### 基础配置

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | None |
| 流控 | None |
| 结尾符 | `\r\n`（回车换行） |

### 指令表

| 指令 | 格式 | 示例 | 说明 |
|------|------|------|------|
| 快捷控制 | `<S><D><T>` | `201` | S=速度1-3, D=方向0-A/1-B, T=圈数1-整/2-半 |
| A 微米位移 | `A<距离>` | `A100` | A 方向移动 100um |
| B 微米位移 | `B<距离>` | `B1000` | B 方向移动 1000um |
| 复位 | `R` | `R` | 清除报警状态（方向锁保持，释放后自动解除） |
| 查询 | `?` | `?` | 返回当前位移（0.1um 精度） |

### 指令示例

```text
101     -> Slow, A-dir, Full turn (6400 steps = 1000.0um)
321     -> Fast, B-dir, Half turn (3200 steps = 500.0um)
A100    -> A direction, 100um (640 steps)
B5000   -> B direction, 5000um = 5mm (32000 steps)
R       -> Clear alarm state
?       -> Query position
```

### 反馈格式

```text
正常执行:  "A move 100 um (640 steps)"  ->  "Done  Pos: +100.0 um"
限位报警:  "ALARM: A-limit triggered, reverse dir (B) available"
方向阻止:  "Blocked: limit active, reverse dir only"
位置查询:  "Pos: +1500.0 um"
```

---

## 编译与烧录

### 前置条件

- Keil MDK 5.38+（需 ARM Compiler 5 或 ARMCLANG 6）
- STM32F1xx HAL 库（已包含在 STM32CubeF1 中）
- ST-LINK/V2 调试器

### 编译步骤

1. 用 Keil MDK 打开 `MDK-ARM/STM32_StepperMotor.uvprojx`
2. 确认 Project -> Options -> Target 中芯片型号为 `STM32F103C8`
3. 确认 Options -> C/C++ -> Include Paths 包含：
   - `..\Core\Inc`
   - `..\Drivers\STM32F1xx_HAL_Driver\Inc`
   - `..\Drivers\CMSIS\Include`
   - `..\Drivers\CMSIS\Device\ST\STM32F1xx\Include`
4. 确认 Options -> C/C++ -> Define：`USE_HAL_DRIVER,STM32F103xB`
5. 按 `F7` 编译

### 烧录步骤

1. ST-LINK 连接 SWDIO (PA13)、SWCLK (PA14)、GND、3.3V
2. Keil 中按 `F8`（Download）烧录
3. 复位后通过串口助手连接 PA9/PA10（115200-8-N-1）

---

## 使用说明

### 典型操作流程

```text
1. 上电 -> 查看初始化信息 -> 确认系统就绪
2. 发送 ? -> 确认当前位置为 +0.0 um
3. 发送 A4500 -> 电机正转 4.5mm
4. 发送 B2000 -> 电机反转 2.0mm
5. 触发限位 -> 电机自动停止 + 报警提示
6. 发送反向指令 -> 电机反向移动离开限位
7. 限位释放 -> 方向锁自动清除
```

### 方向约定

- **A 方向**（dir=0）：对应原 CW / 正转 / 远离旋钮
- **B 方向**（dir=1）：对应原 CCW / 反转 / 靠近旋钮

### 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 串口无输出 | 波特率不匹配 / 接线反 | 确认 115200，TX/RX 交叉 |
| 电机只响不转 | 驱动电流不足 | 调整 DRV8825 限流电位器 |
| 位置不更新 | 运动中限位触发 | 发反向指令离开限位 |
| 限位反复报警 | 传感器噪声 | 检查传感器接线和供电 |
| 发送指令无反应 | 方向被锁定 | 检查报警提示，发反向指令 |

---

## 自定义修改指南

### 修改速度档位

编辑 `Core/Src/main.c` 开头的宏定义：

```c
#define DEFAULT_HALF_T_LOW   500   // 低速档半周期（越大越慢）
#define DEFAULT_HALF_T_MID   250   // 中速档
#define DEFAULT_HALF_T_HIGH  180   // 高速档
```

### 修改限位去抖参数

编辑 `Core/Inc/main.h`：

```c
#define LIMIT_FILTER_COUNT  3      // 去抖连续确认次数（x10ms = 总去抖时间）
#define STEP_CHECK_INTERVAL 100    // 步进中限位检测间隔（步数）
```

### 修改行程范围

编辑 `Core/Src/main.c` 中 `parse_micron_value()` 的数值上限：

```c
if (digit_count > 0 && value > 0 && value <= 15000)  // 15mm 最大位移
```

### 添加新指令

在 `Core/Src/main.c` 的 `UART_Parse_Cmd()` 中添加新的 `if` 分支：

```c
if (cmd == 'Z')  // 自定义回零指令
{
    // ... 回零逻辑 ...
    return;
}
```

---

## 文件结构

```text
STM32_StepperMotor-V9/
|-- README.md                          <- 本文件
|-- StepperMotorCtrol.md               <- 详细中文说明文档
|-- STM32_StepperMotor.ioc             <- CubeMX 项目配置
|-- .mxproject                         <- CubeMX 辅助配置
|-- keilkill.bat                       <- Keil 编译产物清理脚本
|
|-- Core/
|   |-- Inc/
|   |   |-- main.h                     <- 主头文件（引脚、宏、换算公式）
|   |   |-- stepper.h                  <- 步进电机驱动接口
|   |   |-- gpio.h                     <- GPIO 初始化
|   |   |-- tim.h                      <- TIM2 PWM 初始化
|   |   |-- usart.h                    <- USART1 初始化
|   |   |-- stm32f1xx_it.h            <- 中断处理声明
|   |   `-- stm32f1xx_hal_conf.h      <- HAL 模块配置
|   |
|   `-- Src/
|       |-- main.c                     <- 主程序（指令解析、限位逻辑、位移追踪）
|       |-- stepper.c                  <- 脉冲生成 + 步进中限位检测
|       |-- gpio.c                     <- GPIO 引脚初始化
|       |-- tim.c                      <- TIM2 定时器初始化
|       |-- usart.c                    <- USART1 初始化
|       |-- stm32f1xx_it.c            <- 中断服务函数（UART RX + SysTick）
|       |-- stm32f1xx_hal_msp.c       <- HAL MSP 初始化
|       `-- system_stm32f1xx.c        <- 系统时钟初始化
|
`-- MDK-ARM/
    |-- startup_stm32f103xb.s          <- 启动文件（向量表 + 栈/堆）
    |-- STM32_StepperMotor.uvprojx     <- Keil 项目文件
    |-- STM32_StepperMotor.uvoptx      <- Keil 项目选项
    |-- STM32_StepperMotor/
    |   `-- STM32_StepperMotor.sct     <- 链接脚本（Scatter File）
    `-- RTE/
        `-- _STM32_StepperMotor/
            `-- RTE_Components.h       <- RTE 配置
```

---

## 附录

### 速查表

| 微米 -> 步数 | 步数 -> 微米 |
|-------------|-------------|
| 1um = 6 步 | 6 步 ~= 0.9um |
| 10um = 64 步 | 64 步 = 10.0um |
| 100um = 640 步 | 640 步 = 100.0um |
| 500um = 3200 步 | 3200 步 = 500.0um |
| 1000um = 6400 步 | 6400 步 = 1000.0um |
| 5000um = 32000 步 | 32000 步 = 5000.0um |

### 版本历史

| 版本 | 日期 | 要点 |
|------|------|------|
| V9 | 2026-05-01 | A/B 微米指令、方向独立锁定、步数追踪、快速限位去抖、UART 中断修复 |
| V1.7 | 2026-04-16 | 串口指令解析重构、限位去抖、位移精度提升 |
| V1.4 | 2026-04-07 | 基础控制功能（正反转/三档速度/半圈整圈） |

### 换算公式速查

```c
// um -> 步数（整数，最大量化误差 < 1 步）
#define UM_TO_STEPS(um)       ((uint32_t)(um) * 32 / 5)

// 步数 -> 0.1um（用于显示）
#define STEPS_TO_0P1UM(steps) ((int32_t)(steps) * 25 / 16)
```
