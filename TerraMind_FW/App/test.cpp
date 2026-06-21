#include "test.h"

#include "../Driver/debug_printer.h"
#include "../Driver/pwm_motor.h"
#include "../Driver/pwm_servo.h"
#include "../Driver/pwm_esc.h"
#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/debug_printer.h"

#include <new>

DebugPrinter g_debug(&huart3);

//M3508差速底盘测试
namespace
{
diff_chassis::MechanicalConfig g_chassis_cfg;
}

diff_chassis g_chassis(1, 2, g_chassis_cfg);
bool g_test_ready = false;
uint32_t g_test_start_tick = 0;

extern "C" void AppTest_M3508_Init(void)
{
    g_chassis_cfg.wheel_track_m = 0.32f;
    g_chassis_cfg.wheel_diameter_m = 0.10f;
    g_chassis_cfg.max_linear_speed_mps = 0.35f;
    g_chassis_cfg.max_angular_speed_rad = 2.0f;

    if (!CAN_BUS.init_default(&hcan1))
    {
        g_test_ready = false;
        g_debug.printf("diff_chassis test init failed: CAN init error\n");
        return;
    }

    g_chassis.set_mechanical_config(g_chassis_cfg);

    // 先使用保守的速度环参数验证差速底盘控制链路。
    g_chassis.left_motor()->set_pid(10.0f, 0.0f, 0.5f);
    g_chassis.right_motor()->set_pid(10.0f, 0.0f, 0.5f);
    g_chassis.left_motor()->reset_controller();
    g_chassis.right_motor()->reset_controller();
    g_chassis.stop();

    g_test_start_tick = HAL_GetTick();
    g_test_ready = true;
    g_debug.printf("diff_chassis test init ok. track=%.2fm, diameter=%.2fm\n",
                   g_chassis_cfg.wheel_track_m,
                   g_chassis_cfg.wheel_diameter_m);
    g_debug.printf("phase: stop -> forward -> backward -> turn left -> turn right\n");
}

extern "C" void AppTest_M3508_TaskStep(void)
{
    if (!g_test_ready)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t phase_ms = 4000u;
    const uint32_t phase = ((now_tick - g_test_start_tick) / phase_ms) % 5u;

    float cmd_v = 0.0f;
    float cmd_w = 0.0f;
    const char *phase_name = "stop";

    switch (phase)
    {
    case 0u:
        phase_name = "stop";
        break;
    case 1u:
        cmd_v = 0.20f;
        phase_name = "forward";
        break;
    case 2u:
        cmd_v = -0.20f;
        phase_name = "backward";
        break;
    case 3u:
        cmd_w = 1.00f;
        phase_name = "turn_left";
        break;
    default:
        cmd_w = -1.00f;
        phase_name = "turn_right";
        break;
    }

    (void)g_chassis.set_cmd_vel(cmd_v, cmd_w);

    static uint32_t last_print_tick = 0;
    if ((now_tick - last_print_tick) >= 500u)
    {
        last_print_tick = now_tick;

        const diff_chassis::WheelTarget target = g_chassis.calc_wheel_target_rpm(cmd_v, cmd_w);
        const M3508::State &left_state = g_chassis.left_motor()->get_state();
        const M3508::State &right_state = g_chassis.right_motor()->get_state();

        g_debug.printf("[diff] %s v=%.2f w=%.2f | target(L,R)=%.1f, %.1f rpm\n",
                       phase_name,
                       cmd_v,
                       cmd_w,
                       target.left_rpm,
                       target.right_rpm);
        g_debug.printf("[diff] left out=%.1f rpm cmd=%d temp=%d | right out=%.1f rpm cmd=%d temp=%d\n",
                       left_state.output_rpm,
                       left_state.command,
                       left_state.temperature,
                       right_state.output_rpm,
                       right_state.command,
                       right_state.temperature);
    }
}

//PWM电机测试
float pwm_kp = 1200.0f;
float pwm_kd = 5.0f;
namespace
{
// PWM 电机测试任务周期。
// 这里与 freertos.c 中的 osDelay(10) 保持一致，必须同步修改。
static const uint32_t kPwmMotorTestPeriodMs = 10u;
static const float kPwmMotorTestPeriodS = 0.01f;

// 下面两项是和实物强相关的核心参数。
// 如果当前接的 A 口电机型号不是这一组默认值，请优先修改这里。
// 1. gear_ratio: 输出轴相对电机本体的减速比
// 2. encoder_counts_per_rev: 电机本体旋转一圈对应的编码器计数值
static const float kMotorGearRatio = 30.0f;
static const uint32_t kMotorEncoderCountsPerRev = 52u;

// 速度测试要求：
// 1. 上电后前 5s 保持停转
// 2. 5s 后开始按当前目标转速持续旋转
static const float kTargetStopRpm = 0.0f;
static const float kTargetForwardRpm = 200.0f;
static const uint32_t kPwmMotorStartDelayMs = 5000u;

alignas(PwmMotor) static unsigned char g_pwm_motor_storage[sizeof(PwmMotor)];
PwmMotor *g_pwm_motor = 0;
bool g_pwm_test_ready = false;
uint32_t g_pwm_test_start_tick = 0u;

PwmMotor::HardwareConfig build_pwm_motor_hardware_config()
{
    PwmMotor::HardwareConfig config;
    config.motor_id = PwmEncBsp::MOTOR_A;
    config.gear_ratio = kMotorGearRatio;
    config.encoder_counts_per_rev = kMotorEncoderCountsPerRev;
    config.control_period_s = kPwmMotorTestPeriodS;
    // 当前 A 口实测为“PWM 正方向”和“编码器正方向”相反，因此这里取 -1。
    config.direction_sign = 1.0f;
    return config;
}

PwmMotor::SpeedPidConfig build_pwm_motor_pid_config()
{
    PwmMotor::SpeedPidConfig config;

    // 测试阶段先使用较保守的速度环参数，避免第一次上电就给出过猛输出。
    config.kp = pwm_kp;
    config.ki = 0.0f;
    config.kd = pwm_kd;
    config.integral_limit = 3000.0f;
    config.output_limit = 65535.0f;
    config.deadzone = 0.5f;
    config.integral_separation_threshold = 50.0f;
    return config;
}

const char *get_test_phase_name(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return "stop";
    default:
        return "forward";
    }
}

float get_target_rpm_by_phase(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return kTargetStopRpm;
    default:
        return kTargetForwardRpm;
    }
}
} // namespace

namespace
{
static const uint32_t kServoASwitchDelayMs = 10000u;
static const float kServoATargetPositiveDeg = -80.0f;
static const float kServoATargetNegativeDeg = 135.0f;

alignas(PwmServo) static unsigned char g_servo_a_storage[sizeof(PwmServo)];
PwmServo *g_servo_a = 0;
bool g_servo_a_test_ready = false;
uint32_t g_servo_a_test_start_tick = 0u;

PwmServo::HardwareConfig build_servo_a_hardware_config()
{
    PwmServo::HardwareConfig config;
    config.servo_id = PwmServoBsp::SERVO_A;
    config.max_angle_deg = 270.0f;
    config.center_compare = 1825.0f;
    config.compare_delta = 115.0f;
    return config;
}

const char *get_servo_a_phase_name(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return "negative_hold";
    default:
        return "positive_hold";
    }
}

float get_servo_a_target_angle(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return kServoATargetNegativeDeg;
    default:
        return kServoATargetPositiveDeg;
    }
}
} // namespace

extern "C" void AppTest_PwmMotor_Init(void)
{
    if (g_pwm_motor == 0)
    {
        const PwmMotor::HardwareConfig hardware_config = build_pwm_motor_hardware_config();
        const PwmMotor::SpeedPidConfig pid_config = build_pwm_motor_pid_config();
        g_pwm_motor = new (g_pwm_motor_storage) PwmMotor(hardware_config, pid_config);
    }

    g_pwm_test_start_tick = HAL_GetTick();
    g_pwm_test_ready = true;

    g_debug.printf("[pwm_motor] init ok. port=A period=%lums gear=%.2f enc=%lu dir=%.1f\n",
                   (unsigned long)kPwmMotorTestPeriodMs,
                   kMotorGearRatio,
                   (unsigned long)kMotorEncoderCountsPerRev,
                   -1.0f);
    g_debug.printf("[pwm_motor] phase: stop(5s) -> forward(keep)\n");
}

extern "C" void AppTest_PwmMotor_TaskStep(void)
{
    if (!g_pwm_test_ready || g_pwm_motor == 0)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t elapsed_ms = now_tick - g_pwm_test_start_tick;
    const uint32_t phase = (elapsed_ms < kPwmMotorStartDelayMs) ? 0u : 1u;
    const float target_rpm = get_target_rpm_by_phase(phase);
    const char *phase_name = get_test_phase_name(phase);

    // 固定周期执行速度环控制。
    g_pwm_motor->control_speed(target_rpm);

    static uint32_t last_print_tick = 0u;
    if ((now_tick - last_print_tick) >= 200u)
    {
        last_print_tick = now_tick;
        g_debug.printf("[pwm_motor] %s target=%.1f rpm current=%.1f rpm\n",
                       phase_name,
                       target_rpm,
                       g_pwm_motor->get_current_rpm());
    }
}

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

    (void)g_servo_a->set_angle(kServoATargetNegativeDeg);
    g_servo_a_test_start_tick = HAL_GetTick();
    g_servo_a_test_ready = true;

    g_debug.printf("[servo_a] init ok. phase: negative(10s) -> positive(keep)\n");
}

extern "C" void AppTest_ServoA_TaskStep(void)
{
    if (!g_servo_a_test_ready || g_servo_a == 0)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t elapsed_ms = now_tick - g_servo_a_test_start_tick;
    const uint32_t phase = (elapsed_ms < kServoASwitchDelayMs) ? 0u : 1u;
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

// ==================== 电调测试代码 ====================
// 测试目标：验证 PWM 电调驱动是否正常工作
// 测试流程：空转(5秒) → 20%油门(5秒) → 停止(保持)
// 电调A使用 TIM13 (PA6)，电调B使用 TIM14 (PA7)
// ======================================================

namespace
{
// 测试参数配置
static const uint32_t kEscSwitchDelayMs = 5000u;      // 每个阶段持续时间：5秒
static const float kEscTargetIdlePercent = 0.0f;      // 空转油门：0%（1ms脉宽，电调不转）
static const float kEscTargetForwardPercent = 20.0f;   // 正转油门：20%（1.2ms脉宽，电机慢速转动）
static const float kEscTargetStopPercent = 0.0f;       // 停止油门：0%（1ms脉宽，电调停止）

// 电调A的全局变量
// 使用 placement new 在静态内存上分配对象，避免动态内存分配
alignas(PwmEsc) static unsigned char g_esc_a_storage[sizeof(PwmEsc)];
PwmEsc *g_esc_a = 0;                    // 电调A对象指针
bool g_esc_a_test_ready = false;        // 电调A测试是否就绪
uint32_t g_esc_a_test_start_tick = 0u;  // 电调A测试开始时间戳

// 电调B的全局变量（与电调A结构相同）
alignas(PwmEsc) static unsigned char g_esc_b_storage[sizeof(PwmEsc)];
PwmEsc *g_esc_b = 0;                    // 电调B对象指针
bool g_esc_b_test_ready = false;        // 电调B测试是否就绪
uint32_t g_esc_b_test_start_tick = 0u;  // 电调B测试开始时间戳

// 构建电调A的硬件配置
PwmEsc::HardwareConfig build_esc_a_hardware_config()
{
    PwmEsc::HardwareConfig config;
    config.esc_id = PwmEscBsp::ESC_A;   // 使用ESC_A接口（TIM13, PA6）
    config.min_throttle_compare = 1000u; // 0%油门对应1ms脉宽（1000个计数）
    config.max_throttle_compare = 2000u; // 100%油门对应2ms脉宽（2000个计数）
    return config;
}

// 构建电调B的硬件配置
PwmEsc::HardwareConfig build_esc_b_hardware_config()
{
    PwmEsc::HardwareConfig config;
    config.esc_id = PwmEscBsp::ESC_B;   // 使用ESC_B接口（TIM14, PA7）
    config.min_throttle_compare = 1000u; // 0%油门对应1ms脉宽
    config.max_throttle_compare = 2000u; // 100%油门对应2ms脉宽
    return config;
}

// 获取测试阶段名称（用于调试输出）
const char *get_esc_phase_name(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return "idle";      // 阶段0：空转
    case 1u:
        return "forward";   // 阶段1：正转
    default:
        return "stop";      // 阶段2：停止
    }
}

// 根据测试阶段获取目标油门值
float get_esc_target_throttle(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return kEscTargetIdlePercent;      // 阶段0：0%油门
    case 1u:
        return kEscTargetForwardPercent;   // 阶段1：20%油门
    default:
        return kEscTargetStopPercent;      // 阶段2：0%油门
    }
}
} // namespace

// ==================== 电调A测试函数 ====================

// 电调A初始化函数
// 在FreeRTOS任务启动时调用一次
extern "C" void AppTest_EscA_Init(void)
{
    // 如果电调A对象还未创建，则创建它
    if (g_esc_a == 0)
    {
        const PwmEsc::HardwareConfig hardware_config = build_esc_a_hardware_config();
        // 使用placement new在预分配的内存上创建对象
        g_esc_a = new (g_esc_a_storage) PwmEsc(hardware_config);
    }

    // 记录测试开始时间
    g_esc_a_test_start_tick = HAL_GetTick();
    g_esc_a_test_ready = true;

    // 打印测试流程说明
    g_debug.printf("[esc_a] init ok. phase: idle(5s) -> forward(5s) -> stop(keep)\n");
}

// 电调A任务循环函数
// 在FreeRTOS任务中周期性调用（每2ms调用一次）
extern "C" void AppTest_EscA_TaskStep(void)
{
    // 安全检查：如果测试未就绪或对象未创建，直接返回
    if (!g_esc_a_test_ready || g_esc_a == 0)
    {
        return;
    }

    // 计算当前时间和已过去的时间
    const uint32_t now_tick = HAL_GetTick();
    const uint32_t elapsed_ms = now_tick - g_esc_a_test_start_tick;

    // 根据已过去的时间判断当前处于哪个测试阶段
    // 阶段0（0-5秒）：空转
    // 阶段1（5-10秒）：20%油门正转
    // 阶段2（10秒后）：停止
    const uint32_t phase = (elapsed_ms < kEscSwitchDelayMs) ? 0u : ((elapsed_ms < kEscSwitchDelayMs * 2u) ? 1u : 2u);

    // 获取当前阶段的目标油门和阶段名称
    const float target_throttle = get_esc_target_throttle(phase);
    const char *phase_name = get_esc_phase_name(phase);

    // 设置电调油门
    (void)g_esc_a->set_throttle(target_throttle);

    // 每500ms打印一次调试信息
    static uint32_t last_print_tick = 0u;
    if ((now_tick - last_print_tick) >= 500u)
    {
        last_print_tick = now_tick;
        g_debug.printf("[esc_a] %s target=%.1f%% current=%.1f%%\n",
                       phase_name,
                       target_throttle,
                       g_esc_a->get_current_throttle());
    }
}

// ==================== 电调B测试函数 ====================
// 电调B的测试逻辑与电调A完全相同，只是使用不同的硬件接口

// 电调B初始化函数
extern "C" void AppTest_EscB_Init(void)
{
    if (g_esc_b == 0)
    {
        const PwmEsc::HardwareConfig hardware_config = build_esc_b_hardware_config();
        g_esc_b = new (g_esc_b_storage) PwmEsc(hardware_config);
    }

    g_esc_b_test_start_tick = HAL_GetTick();
    g_esc_b_test_ready = true;

    g_debug.printf("[esc_b] init ok. phase: idle(5s) -> forward(5s) -> stop(keep)\n");
}

// 电调B任务循环函数
extern "C" void AppTest_EscB_TaskStep(void)
{
    if (!g_esc_b_test_ready || g_esc_b == 0)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t elapsed_ms = now_tick - g_esc_b_test_start_tick;
    const uint32_t phase = (elapsed_ms < kEscSwitchDelayMs) ? 0u : ((elapsed_ms < kEscSwitchDelayMs * 2u) ? 1u : 2u);
    const float target_throttle = get_esc_target_throttle(phase);
    const char *phase_name = get_esc_phase_name(phase);

    (void)g_esc_b->set_throttle(target_throttle);

    static uint32_t last_print_tick = 0u;
    if ((now_tick - last_print_tick) >= 500u)
    {
        last_print_tick = now_tick;
        g_debug.printf("[esc_b] %s target=%.1f%% current=%.1f%%\n",
                       phase_name,
                       target_throttle,
                       g_esc_b->get_current_throttle());
    }
}
