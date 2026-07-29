# mainboard 农业机器人嵌入式控制平台

## 个人工作量报告

---

**作者**：唐  
**开发周期**：2026 年 5 月 2 日 — 7 月 5 日（9 周）  
**平台**：STM32F407VET6 + FreeRTOS（CMSIS_V2）  

---

## 一、项目概述

mainboard 是一款面向农业场景的智能机器人嵌入式控制平台，从零构建完整固件，实现差速轮式底盘运动控制、左右双路播撒装置、割草装置、远程指令接收、Xbox 手柄操控五大功能。

---

## 二、工作量量化

### 2.1 代码规模

| 类别 | 文件数 | 代码行数 |
|------|--------|----------|
| 手写应用代码（mainboard_FW/） | 40 | **3,930** |
| STM32 CubeMX 配置代码（Core/） | 18 | **3,791** |
| 设计文档 | 5 | **~1,700** |

Git 统计：**净新增 +5,331 行**（44 个文件变更），**14 次提交**。

### 2.2 模块划分

```mermaid
%%{init: {"pie": {"textPosition": 0.6}, "themeVariables": {"pie1": "#4e79a7", "pie2": "#f28e2b", "pie3": "#e15759", "pie4": "#76b7b2", "pie5": "#59a14f", "pie6": "#edc948", "pie7": "#b07aa1"}}}%%
pie title 各模块代码量占比（共 3,930 行）
    "Driver 设备驱动 1197" : 1197
    "App 应用层 755" : 755
    "BSP 板级支持包 748" : 748
    "Algorithm 算法库 432" : 432
    "Utils 工具库 349" : 349
    "Device 设备抽象 326" : 326
    "OSAL 操作系统抽象 123" : 123
```

### 2.3 硬件外设配置

| 外设类型 | 数量 | 说明 |
|----------|------|------|
| CAN 总线 | 1 路 | M3508 电机通信（1 Mbps） |
| UART 串口 | 6 路 | 调试/手柄/远程命令/预留 |
| 定时器（编码器模式） | 4 路 | 正交编码器硬件解码 |
| 定时器（PWM 输出） | 10 路 | 舵机/电机/电调驱动 |
| GPIO 引脚 | 44+ | — |

> 共配置 **23 个 MCU 内部 IP**，使用了 STM32F407VET6 几乎全部定时器资源。

---

## 三、系统架构设计

### 3.1 总体结构

```mermaid
flowchart TB
    PC["🖥 PC 上位机<br/>下发控制指令<br/>（线速度/角速度/开关）"]

    PC -->|"UART5 115200bps<br/>19字节帧 + CRC16"| UART5

    subgraph MCU["mainboard 主控板 · STM32F407VET6 · 168MHz"]

        subgraph TASKS["FreeRTOS 多任务调度"]
            direction LR
            T1["控制任务<br/>500 Hz"]
            T2["通信任务<br/>ISR 驱动"]
            T3["打印任务<br/>2 Hz"]
        end

        subgraph COMM["通信层"]
            direction LR
            UART5["UART5<br/>CmdPort 接收"]
            USART2["USART2<br/>Xbox 手柄"]
            USART3["USART3<br/>调试打印"]
        end

        subgraph EXEC["执行层"]
            direction LR
            CAN1["CAN1<br/>M3508 ×2"]
            PWM_A["PWM 舵机 ×2<br/>TIM9 CH1+2"]
            PWM_B["PWM 编码电机 ×2<br/>TIM10/11"]
            PWM_C["PWM 电调 ×1<br/>TIM1 CH1"]
        end

    end

    subgraph ACT["执行器"]
        direction LR
        CHASSIS["🏗 差速底盘<br/>双轮差速驱动"]
        SEEDER["🌱 播撒器<br/>左右双路"]
        MOWING["⚙ 割草装置<br/>无刷电机"]
    end

    UART5 --> T1
    USART2 --> T2
    T1 --> CAN1 --> CHASSIS
    T1 --> PWM_A --> SEEDER
    T1 --> PWM_B --> SEEDER
    T1 --> PWM_C --> MOWING
```

### 3.2 软件五层架构

```mermaid
flowchart TB
    subgraph APP["App 应用层"]
        A1["app_main<br/>系统初始化 · 控制主循环"]
        A2["test<br/>模块独立测试框架"]
    end

    subgraph DEVICE["Device 设备抽象层"]
        D1["diff_chassis<br/>运动学正逆解 · CAN 打包"]
    end

    subgraph DRIVER["Driver 设备驱动层"]
        direction LR
        R1["M3508"]
        R2["pwm_motor"]
        R3["pwm_servo"]
        R4["pwm_esc"]
        R5["xbox"]
        R6["cmd_port"]
        R7["debug_printer"]
    end

    subgraph BSP["BSP 板级支持包"]
        direction LR
        B1["can_bsp"]
        B2["pwm_enc"]
        B3["pwm_servo"]
        B4["pwm_esc"]
        B5["serial_device"]
    end

    subgraph HAL["HAL 硬件抽象层"]
        H1["STM32F4xx_HAL + CMSIS"]
    end

    subgraph CROSS["跨层模块"]
        direction LR
        C1["Algorithm<br/>PID / SuperPID"]
        C2["OSAL<br/>TaskManager"]
        C3["Utils<br/>CRC / Vector2D"]
    end

    APP --> DEVICE --> DRIVER --> BSP --> HAL
    CROSS -.-> APP
    CROSS -.-> DRIVER
```

### 3.3 设计要点

**面向对象抽象**
- `SerialDevice` 抽象基类：UART 中断统一分发，新增协议只需继承并重写一个方法
- `ITaskProcessor` 纯虚接口：业务模块与 FreeRTOS 调度解耦，通过 `TaskManager` 声明式注册
- `diff_chassis` 双重构造模式：外部注入（便于测试）或指定 ID 内部创建（便于集成）

**设计模式**
- 策略模式：`ITaskProcessor` 多态注册到不同任务槽
- 模板方法：`SerialDevice` 定义中断接收流程，子类实现协议解析
- RAII：所有驱动对象禁用拷贝，析构自动释放资源

**双闭环控制**

```mermaid
flowchart LR
    subgraph LOOP1["M3508 CAN 电机（电流闭环）"]
        direction LR
        A1["🎯 目标电流"] --> A2["PID"] --> A3["CAN 发送"] --> A4["M3508"] --> A5["📊 转速/位置反馈"] --> A1
    end

    subgraph LOOP2["PWM 编码电机（速度闭环）"]
        direction LR
        B1["🎯 目标转速"] --> B2["SuperPID<br/>增量式 · 积分分离<br/>死区 · 输出限幅"] --> B3["PWM 输出"] --> B4["直流减速电机"] --> B5["编码器"] --> B6["转速计算"] --> B1
    end
```

---

## 四、通信协议设计

### 4.1 CmdPort 协议 v2.0

```mermaid
flowchart LR
    subgraph FRAME["完整帧 · 19 字节固定"]
        direction LR
        H0["Head0<br/>0xFC"] --> H1["Head1<br/>0xFB"] --> ID["Frame ID<br/>1 Byte"] --> LEN["Data Len<br/>0x0B"]
        LEN --> D0["linear_speed<br/>float32 (4B)"] --> D1["angular_speed<br/>float32 (4B)"] --> D2["左播撒<br/>uint8"] --> D3["右播撒<br/>uint8"] --> D4["割草<br/>uint8"]
        D4 --> CRC["CRC16<br/>2 Bytes"] --> E0["End0<br/>0xFD"] --> E1["End1<br/>0xFE"]
    end
```

**特性**
- 数据段 CRC-16/XMODEM 校验，多项式 0x1021
- 9 状态有限状态机解析，任何异常自动回退重新同步
- 设计手册完整（470 行），含 Python 上位机参考实现

---

## 五、Git 开发记录

### 5.1 提交历史（14 次）

| 日期 | 提交信息 | 说明 |
|------|----------|------|
| 5/2 | `init: first commit` | 工程初始化（CubeMX 生成 + FreeRTOS 集成） |
| 5/3 | `refactor: 重构设备驱动模块并添加调试打印功能` | 代码结构整理、DebugPrinter 框架 |
| 5/3 | `fix(xbox): 修正手柄按键映射顺序和数据解析注释` | Xbox 手柄驱动纠错 |
| 5/3 | `feat(test): 添加 Xbox 控制器支持到 M3508 测试应用` | 手柄 + 电机联调 |
| 5/4 | `feat(device): 新增差速底盘设备类及测试程序` | diff_chassis 模块 |
| 5/4 | `feat(pwm_motor): 新增PWM电机驱动与测试框架` | 编码器 + PID 闭环 |
| 5/4 | `refactor(test): 移除重复的全局变量声明并调整头文件包含` | 代码质量优化 |
| 5/4 | `fix(BSP): 修正电机的PWM定时器通道和方向` | 硬件映射校验 |
| 5/24 | `feat(pwm/tim): 完善TIM8 PWM配置并优化电机测试功能` | TIM 扩展 |
| 6/21 | `feat: add PWM servo driver and incremental test for servo A` | 舵机驱动 |
| 6/21 | `feat(esc): 新增PWM电调驱动与测试程序` | 电调/割草驱动 |
| 7/2 | `feat(app): 新增应用主程序及命令端口类` | 系统集成 + CmdPort v1.0 |
| 7/2 | `feat(cmd_port): 扩展数据结构，新增右侧播撒器和割草` | 协议 v2.0 扩展 |
| 7/5 | `feat(app): 添加右侧播撒器和割草功能，重构控制逻辑` | 功能完成 |

### 5.2 开发时间线

```mermaid
flowchart LR
    subgraph W1["第 1 周 · 5/2"]
        direction TB
        S1["环境搭建<br/>➊ CubeMX 配置<br/>➋ FreeRTOS 集成<br/>➌ TaskManager"]
    end

    subgraph W2["第 2 周 · 5/3"]
        direction TB
        S2["通信框架<br/>➊ Xbox 手柄驱动<br/>➋ M3508 基础驱动<br/>➌ DebugPrinter"]
    end

    subgraph W3["第 3 周 · 5/4"]
        direction TB
        S3["底盘 + 电机<br/>➊ diff_chassis 运动学<br/>➋ PWM 编码电机闭环<br/>➌ 硬件校准"]
    end

    subgraph W4["第 4~6 周 · 5/24"]
        direction TB
        S4["定时器扩展<br/>➊ TIM8 PWM 配置<br/>➋ 电机测试优化"]
    end

    subgraph W5["第 7~8 周 · 6/21"]
        direction TB
        S5["舵机 + 电调<br/>➊ PWM 舵机驱动<br/>➋ PWM 电调驱动<br/>➌ 独立测试程序"]
    end

    subgraph W6["第 9 周 · 7/2~7/5"]
        direction TB
        S6["系统集成<br/>➊ CmdPort v1.0 → v2.0<br/>➋ 应用主程序<br/>➌ 全系统联调"]
    end

    W1 --> W2 --> W3 --> W4 --> W5 --> W6
```

### 5.3 代码变更

44 个文件变更，+5,331 行新增，-209 行删除。

| 变更类型 | 文件 |
|----------|------|
| 新增模块 | `app_main` · `cmd_port` · `diff_chassis` · `pwm_motor` · `pwm_servo` · `pwm_esc` · `xbox` · `debug_printer` · `pwm_enc_bsp` · `pwm_esc_bsp` · `pwm_servo_bsp` |
| 重构模块 | `M3508`（CAN 电机驱动） · `Serial_device`（串口抽象基类） · `test`（测试框架） |
| 配置扩展 | `Core/tim.c`（+364 行 TIM 配置） · `Core/freertos.c`（任务调度调整） |

---

## 六、产出物汇总

| 类别 | 数量 |
|------|------|
| 手写代码 | 3,930 行 / 40 文件 |
| 配置代码 | 3,791 行 / 18 文件 |
| 软件模块 | 7 大模块 / 20+ 子模块 |
| 硬件外设 | 23 个 IP / 44+ 引脚 |
| Git 提交 | 14 次 |
| 设计文档 | 5 份 |
| 协议设计 | CmdPort v2.0（19 字节帧 + CRC16）+ Python 参考实现 |
| 测试覆盖 | 7 项独立测试 + 全系统模拟注入 |

---

**日期**：2026 年 7 月 10 日
