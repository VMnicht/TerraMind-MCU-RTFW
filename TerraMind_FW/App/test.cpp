#include "test.h"

#include "../Driver/debug_printer.h"
#include "../Driver/pwm_motor.h"
#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/debug_printer.h"

#include <new>

DebugPrinter g_debug(&huart1);

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

// 速度测试采用单方向旋转后停转的方式循环，便于观察起转、稳态和刹停效果。
static const float kTargetStopRpm = 0.0f;
static const float kTargetForwardRpm = 200.0f;

alignas(PwmMotor) static unsigned char g_pwm_motor_storage[sizeof(PwmMotor)];
PwmMotor *g_pwm_motor = 0;
bool g_pwm_test_ready = false;
uint32_t g_pwm_test_start_tick = 0u;

PwmMotor::HardwareConfig build_pwm_motor_hardware_config()
{
    PwmMotor::HardwareConfig config;
    config.motor_id = PwmEncBsp::MOTOR_D;
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
        return "forward";
    default:
        return "stop";
    }
}

float get_target_rpm_by_phase(uint32_t phase)
{
    switch (phase)
    {
    case 0u:
        return kTargetForwardRpm;
    default:
        return kTargetStopRpm;
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
    g_debug.printf("[pwm_motor] phase: forward(20s) -> stop(20s)\n");
}

extern "C" void AppTest_PwmMotor_TaskStep(void)
{
    if (!g_pwm_test_ready || g_pwm_motor == 0)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    const uint32_t phase_ms = 20000u;
    const uint32_t phase = ((now_tick - g_pwm_test_start_tick) / phase_ms) % 2u;
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
