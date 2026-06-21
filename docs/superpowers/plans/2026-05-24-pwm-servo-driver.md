# PWM Servo Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 TerraMind 当前工程新增与现有 `pwm_motor` 风格一致的 PWM 舵机驱动，支持 A/B/C/D 固定口位与参考工程一致的角度映射。

**Architecture:** 新增 `PwmServoBsp` 负责定时器和通道映射、PWM 启动与 compare 输出；新增 `PwmServo` 负责角度限幅、参考公式换算与回中。工程文件 `MDK-ARM/TerraMind.uvprojx` 同步加入新文件，最后用诊断检查和最小编译验证保证接入正确。

**Tech Stack:** C++、STM32 HAL TIM、Keil uVision `.uvprojx`、Trae diagnostics

---

### Task 1: 实现舵机 BSP 层

**Files:**
- Create: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.h`
- Create: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.cpp`
- Reference: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_enc_bsp.h`
- Reference: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_enc_bsp.cpp`

- [ ] **Step 1: 写出 BSP 头文件骨架**

```cpp
#ifndef PWM_SERVO_BSP_H
#define PWM_SERVO_BSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class PwmServoBsp
{
public:
    enum ServoId
    {
        SERVO_A = 0,
        SERVO_B,
        SERVO_C,
        SERVO_D
    };

    explicit PwmServoBsp(ServoId servo_id);

    void set_compare(uint32_t compare_value);
    bool is_valid() const;

private:
    void bind_servo_config(ServoId servo_id);
    void start_hardware();
    uint32_t clamp_compare_value(uint32_t compare_value) const;

private:
    TIM_HandleTypeDef *tim_;
    uint32_t channel_;
    uint32_t period_;
    bool is_valid_;
};

#endif

#endif
```

- [ ] **Step 2: 实现固定口位映射与 PWM 启动**

```cpp
#include "pwm_servo_bsp.h"

PwmServoBsp::PwmServoBsp(ServoId servo_id)
    : tim_(0),
      channel_(0u),
      period_(0u),
      is_valid_(false)
{
    bind_servo_config(servo_id);
    if (!is_valid_)
    {
        return;
    }

    period_ = __HAL_TIM_GET_AUTORELOAD(tim_);
    start_hardware();
}

void PwmServoBsp::bind_servo_config(ServoId servo_id)
{
    tim_ = 0;
    channel_ = 0u;
    is_valid_ = true;

    switch (servo_id)
    {
    case SERVO_A:
        tim_ = &htim12;
        channel_ = TIM_CHANNEL_2;
        break;
    case SERVO_B:
        tim_ = &htim12;
        channel_ = TIM_CHANNEL_1;
        break;
    case SERVO_C:
        tim_ = &htim8;
        channel_ = TIM_CHANNEL_4;
        break;
    case SERVO_D:
        tim_ = &htim8;
        channel_ = TIM_CHANNEL_3;
        break;
    default:
        is_valid_ = false;
        break;
    }
}

void PwmServoBsp::start_hardware()
{
    if (!is_valid_)
    {
        return;
    }

    (void)HAL_TIM_PWM_Start(tim_, channel_);
}
```

- [ ] **Step 3: 实现 compare 输出与限幅**

```cpp
void PwmServoBsp::set_compare(uint32_t compare_value)
{
    if (!is_valid_)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(tim_, channel_, clamp_compare_value(compare_value));
}

bool PwmServoBsp::is_valid() const
{
    return is_valid_;
}

uint32_t PwmServoBsp::clamp_compare_value(uint32_t compare_value) const
{
    if (compare_value >= period_)
    {
        return period_;
    }

    return compare_value;
}
```

- [ ] **Step 4: 运行诊断检查 BSP 新文件**

Run: 使用 `GetDiagnostics` 检查
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.h`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.cpp`

Expected: 无新增语法或包含错误

- [ ] **Step 5: 提交本任务**

```bash
git add TerraMind_FW/BSP/pwm_servo_bsp.h TerraMind_FW/BSP/pwm_servo_bsp.cpp
git commit -m "feat: add pwm servo bsp"
```

### Task 2: 实现舵机 Driver 层

**Files:**
- Create: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.h`
- Create: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.cpp`
- Reference: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_motor.h`
- Reference: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_motor.cpp`

- [ ] **Step 1: 写出 Driver 头文件接口**

```cpp
#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "../BSP/pwm_servo_bsp.h"

class PwmServo
{
public:
    struct HardwareConfig
    {
        PwmServoBsp::ServoId servo_id;
        float max_angle_deg;
        float center_compare;
        float compare_delta;

        HardwareConfig();
    };

    explicit PwmServo(const HardwareConfig &hardware_config);

    bool set_angle(float angle_deg);
    bool reset();
    float get_current_angle() const;

private:
    float limit_angle(float angle_deg) const;
    bool hardware_config_is_valid() const;

private:
    PwmServoBsp bsp_;
    HardwareConfig hardware_config_;
    float current_angle_deg_;
};

#endif

#endif
```

- [ ] **Step 2: 实现默认配置与构造行为**

```cpp
#include "pwm_servo.h"

PwmServo::HardwareConfig::HardwareConfig()
    : servo_id(PwmServoBsp::SERVO_A),
      max_angle_deg(180.0f),
      center_compare(1825.0f),
      compare_delta(115.0f)
{
}

PwmServo::PwmServo(const HardwareConfig &hardware_config)
    : bsp_(hardware_config.servo_id),
      hardware_config_(hardware_config),
      current_angle_deg_(0.0f)
{
    if (hardware_config_is_valid())
    {
        (void)reset();
    }
}
```

- [ ] **Step 3: 实现参考兼容的角度换算**

```cpp
bool PwmServo::set_angle(float angle_deg)
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    const float limited_angle_deg = limit_angle(angle_deg);
    const float half_angle_deg = hardware_config_.max_angle_deg * 0.5f;
    const float compare_value =
        hardware_config_.center_compare -
        (limited_angle_deg / half_angle_deg) * hardware_config_.compare_delta;

    bsp_.set_compare(static_cast<uint32_t>(compare_value));
    current_angle_deg_ = limited_angle_deg;
    return true;
}

bool PwmServo::reset()
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    bsp_.set_compare(static_cast<uint32_t>(hardware_config_.center_compare));
    current_angle_deg_ = 0.0f;
    return true;
}

float PwmServo::get_current_angle() const
{
    return current_angle_deg_;
}
```

- [ ] **Step 4: 实现配置校验与限幅**

```cpp
float PwmServo::limit_angle(float angle_deg) const
{
    const float half_angle_deg = hardware_config_.max_angle_deg * 0.5f;
    if (angle_deg > half_angle_deg)
    {
        return half_angle_deg;
    }

    if (angle_deg < -half_angle_deg)
    {
        return -half_angle_deg;
    }

    return angle_deg;
}

bool PwmServo::hardware_config_is_valid() const
{
    if (!bsp_.is_valid())
    {
        return false;
    }

    if (hardware_config_.max_angle_deg <= 0.0f)
    {
        return false;
    }

    if (hardware_config_.compare_delta < 0.0f)
    {
        return false;
    }

    return true;
}
```

- [ ] **Step 5: 运行诊断检查 Driver 新文件**

Run: 使用 `GetDiagnostics` 检查
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.h`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.cpp`

Expected: 无新增语法或包含错误

- [ ] **Step 6: 提交本任务**

```bash
git add TerraMind_FW/Driver/pwm_servo.h TerraMind_FW/Driver/pwm_servo.cpp
git commit -m "feat: add pwm servo driver"
```

### Task 3: 接入 Keil 工程并完成最小验证

**Files:**
- Modify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\MDK-ARM\TerraMind.uvprojx`
- Verify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.h`
- Verify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.cpp`
- Verify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.h`
- Verify: `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.cpp`

- [ ] **Step 1: 将新文件加入 `TerraMind.uvprojx`**

```xml
<File>
  <FileName>pwm_servo.cpp</FileName>
  <FileType>8</FileType>
  <FilePath>..\TerraMind_FW\Driver\pwm_servo.cpp</FilePath>
</File>
<File>
  <FileName>pwm_servo.h</FileName>
  <FileType>5</FileType>
  <FilePath>..\TerraMind_FW\Driver\pwm_servo.h</FilePath>
</File>
```

```xml
<File>
  <FileName>pwm_servo_bsp.cpp</FileName>
  <FileType>8</FileType>
  <FilePath>..\TerraMind_FW\BSP\pwm_servo_bsp.cpp</FilePath>
</File>
<File>
  <FileName>pwm_servo_bsp.h</FileName>
  <FileType>5</FileType>
  <FilePath>..\TerraMind_FW\BSP\pwm_servo_bsp.h</FilePath>
</File>
```

- [ ] **Step 2: 运行全量诊断**

Run: 使用 `GetDiagnostics` 检查以下文件
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\MDK-ARM\TerraMind.uvprojx`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.h`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\BSP\pwm_servo_bsp.cpp`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.h`
- `c:\Users\Tang\Desktop\TerraMind\TerraMind\TerraMind\TerraMind_FW\Driver\pwm_servo.cpp`

Expected: 无新增错误；如果 `uvprojx` 无语言诊断，则其余 C++ 文件应无报错

- [ ] **Step 3: 检查工作区变更**

Run:

```bash
git status --short
```

Expected:
- 新增 4 个舵机文件
- 修改 `MDK-ARM/TerraMind.uvprojx`
- 保留与本任务无关的现有日志文件变更，不做回退

- [ ] **Step 4: 提交本任务**

```bash
git add MDK-ARM/TerraMind.uvprojx TerraMind_FW/BSP/pwm_servo_bsp.h TerraMind_FW/BSP/pwm_servo_bsp.cpp TerraMind_FW/Driver/pwm_servo.h TerraMind_FW/Driver/pwm_servo.cpp
git commit -m "feat: add pwm servo driver to keil project"
```
