# Servo A Incremental Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为当前工程增量加入仅针对 A 口的舵机测试，同时保留现有 PWM 电机测试入口继续运行。

**Architecture:** 在 `test.h/test.cpp` 中新增独立的 `AppTest_ServoA_Init/TaskStep`，内部创建一个绑定 `SERVO_A` 的 `PwmServo` 对象，并用简单状态机执行 `回中 -> 正角度 -> 回中 -> 负角度`。在 `freertos.c` 中保留现有 `AppTest_PwmMotor_*` 调用，同时追加舵机 A 测试调用。

**Tech Stack:** C++、STM32 HAL、FreeRTOS 任务循环、现有 `DebugPrinter`、Trae diagnostics

---

### Task 1: 扩展测试接口声明

**Files:**
- Modify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\App\test.h`

- [ ] **Step 1: 写出会失败的接口引用目标**

```c
void AppTest_ServoA_Init(void);
void AppTest_ServoA_TaskStep(void);
```

- [ ] **Step 2: 通过源码检查确认当前头文件尚未声明这些接口**

Run: 在 `test.h` 中搜索 `AppTest_ServoA`
Expected: 当前不存在 `AppTest_ServoA_Init` 和 `AppTest_ServoA_TaskStep`

- [ ] **Step 3: 在 `test.h` 中加入新声明**

```c
void AppTest_M3508_Init(void);
void AppTest_M3508_TaskStep(void);
void AppTest_PwmMotor_Init(void);
void AppTest_PwmMotor_TaskStep(void);
void AppTest_ServoA_Init(void);
void AppTest_ServoA_TaskStep(void);
```

- [ ] **Step 4: 运行诊断确认头文件无语法问题**

Run: `GetDiagnostics`
Expected: 无新增错误

### Task 2: 在测试实现中增加舵机 A 状态机

**Files:**
- Modify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\App\test.cpp`

- [ ] **Step 1: 写出会失败的新增依赖引用**

```cpp
#include "../Driver/pwm_servo.h"
```

```cpp
extern "C" void AppTest_ServoA_Init(void);
extern "C" void AppTest_ServoA_TaskStep(void);
```

- [ ] **Step 2: 通过源码检查确认当前 `test.cpp` 尚未包含舵机测试实现**

Run: 在 `test.cpp` 中搜索 `AppTest_ServoA`
Expected: 当前不存在舵机 A 初始化与循环函数

- [ ] **Step 3: 新增舵机 A 测试对象与配置辅助代码**

```cpp
#include "../Driver/pwm_servo.h"

namespace
{
static const uint32_t kServoATestPhaseMs = 3000u;
static const float kServoATargetCenterDeg = 0.0f;
static const float kServoATargetPositiveDeg = 30.0f;
static const float kServoATargetNegativeDeg = -30.0f;

alignas(PwmServo) static unsigned char g_servo_a_storage[sizeof(PwmServo)];
PwmServo *g_servo_a = 0;
bool g_servo_a_test_ready = false;
uint32_t g_servo_a_test_start_tick = 0u;

PwmServo::HardwareConfig build_servo_a_hardware_config()
{
    PwmServo::HardwareConfig config;
    config.servo_id = PwmServoBsp::SERVO_A;
    config.max_angle_deg = 180.0f;
    config.center_compare = 1825.0f;
    config.compare_delta = 115.0f;
    return config;
}

const char *get_servo_a_phase_name(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return "center_1";
    case 1u:
        return "positive";
    case 2u:
        return "center_2";
    default:
        return "negative";
    }
}

float get_servo_a_target_angle(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return kServoATargetCenterDeg;
    case 1u:
        return kServoATargetPositiveDeg;
    case 2u:
        return kServoATargetCenterDeg;
    default:
        return kServoATargetNegativeDeg;
    }
}
} // namespace
```

- [ ] **Step 4: 新增舵机 A 初始化与周期执行函数**

```cpp
extern "C" void AppTest_ServoA_Init(void)
{
    if (g_servo_a == 0)
    {
        const PwmServo::HardwareConfig hardware_config = build_servo_a_hardware_config();
        g_servo_a = new (g_servo_a_storage) PwmServo(hardware_config);
    }

    if (g_servo_a == 0)
    {
        g_servo_a_test_ready = false;
        g_debug.printf("[servo_a] init failed\n");
        return;
    }

    (void)g_servo_a->reset();
    g_servo_a_test_start_tick = HAL_GetTick();
    g_servo_a_test_ready = true;

    g_debug.printf("[servo_a] init ok. phase: center -> positive -> center -> negative\n");
}

extern "C" void AppTest_ServoA_TaskStep(void)
{
    if (!g_servo_a_test_ready || g_servo_a == 0)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t phase = ((now_tick - g_servo_a_test_start_tick) / kServoATestPhaseMs) % 4u;
    const float target_angle = get_servo_a_target_angle(phase);
    const char *phase_name = get_servo_a_phase_name(phase);

    (void)g_servo_a->set_angle(target_angle);

    static uint32_t last_print_tick = 0u;
    if ((now_tick - last_print_tick) >= 500u)
    {
        last_print_tick = now_tick;
        g_debug.printf("[servo_a] %s target=%.1f deg current=%.1f deg\n",
                       phase_name,
                       target_angle,
                       g_servo_a->get_current_angle());
    }
}
```

- [ ] **Step 5: 运行诊断确认 `test.cpp` 无新增语法问题**

Run: `GetDiagnostics`
Expected: 无新增错误

### Task 3: 把舵机 A 测试挂到任务循环

**Files:**
- Modify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\Core\Src\freertos.c`

- [ ] **Step 1: 写出会失败的新调用目标**

```c
AppTest_ServoA_Init();
AppTest_ServoA_TaskStep();
```

- [ ] **Step 2: 通过源码检查确认当前 `freertos.c` 尚未调用舵机 A 测试**

Run: 在 `freertos.c` 中搜索 `AppTest_ServoA`
Expected: 当前不存在舵机 A 调用

- [ ] **Step 3: 在保留现有 PWM 电机测试的基础上追加舵机 A 调用**

```c
  AppTest_PwmMotor_Init();
  AppTest_ServoA_Init();
  for(;;)
  {
    AppTest_PwmMotor_TaskStep();
    AppTest_ServoA_TaskStep();
    osDelay(10);
  }
```

- [ ] **Step 4: 运行全局诊断并检查工作区变更**

Run: `GetDiagnostics`
Expected: 无新增错误

Run:

```bash
git status --short
```

Expected:
- 修改 `TerraMind_FW/App/test.h`
- 修改 `TerraMind_FW/App/test.cpp`
- 修改 `Core/Src/freertos.c`
- 保留已有舵机驱动文件和文档变更
