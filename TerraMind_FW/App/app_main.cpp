#include "app_main.h"

#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/cmd_port.h"
#include "../Driver/pwm_motor.h"
#include "../Driver/pwm_servo.h"
#include "../Driver/debug_printer.h"
#include "../Driver/pwm_esc.h"
#include "../Driver/pc_port.h"
#include "../Driver/pwm_open_loop_motor.h"

#include <new>

static DebugPrinter g_debug(&huart6);

alignas(PcPort) static unsigned char g_pc_port_buf[sizeof(PcPort)];
static PcPort *g_pc_port = nullptr;
static PcProtocol::Command g_applied = {};
static bool g_can_tx_failed = false;

static diff_chassis::MechanicalConfig g_chassis_cfg;
static diff_chassis *g_chassis = nullptr;

alignas(cmd_port) static unsigned char g_cmd_port_buf[sizeof(cmd_port)];
static cmd_port *g_cmd_port = nullptr;

alignas(PwmServo) static unsigned char g_servo_buf[sizeof(PwmServo)];
static PwmServo *g_seeder_servo = nullptr;

alignas(PwmMotor) static unsigned char g_motor_buf[sizeof(PwmMotor)];
static PwmMotor *g_seeder_motor = nullptr;

alignas(PwmServo) static unsigned char g_servo_r_buf[sizeof(PwmServo)];
static PwmServo *g_seeder_servo_r = nullptr;
alignas(PwmMotor) static unsigned char g_motor_r_buf[sizeof(PwmMotor)];
static PwmMotor *g_seeder_motor_r = nullptr;

alignas(PwmEsc) static unsigned char g_esc_buf[sizeof(PwmEsc)];
static PwmEsc *g_mowing_esc = nullptr;

alignas(PwmOpenLoopMotor) static unsigned char g_spray_buf[sizeof(PwmOpenLoopMotor)];
static PwmOpenLoopMotor *g_spray_pump = nullptr;

static void init_chassis()
{
    g_chassis_cfg.wheel_track_m = 0.32f;
    g_chassis_cfg.wheel_diameter_m = 0.10f;
    g_chassis_cfg.max_linear_speed_mps = 0.35f;
    g_chassis_cfg.max_angular_speed_rad = 2.0f;
		g_chassis_cfg.right_reversed = true;

    if (!CAN_BUS.init_default(&hcan1))
    {
        g_debug.printf("[app] CAN init failed\n");
        return;
    }

    g_chassis = new diff_chassis(1, 2, g_chassis_cfg);
    g_chassis->left_motor()->set_pid(10.0f, 0.0f, 0.5f);
    g_chassis->right_motor()->set_pid(10.0f, 0.0f, 0.5f);
    g_chassis->left_motor()->reset_controller();
    g_chassis->right_motor()->reset_controller();
    g_chassis->stop();

    g_debug.printf("[app] chassis ok. track=%.2fm dia=%.2fm\n",
                   g_chassis_cfg.wheel_track_m,
                   g_chassis_cfg.wheel_diameter_m);
}

static void init_cmd_port()
{
    g_cmd_port = new (g_cmd_port_buf) cmd_port(&huart5);
    g_cmd_port->startUartReceiveIT();
    g_debug.printf("[app] cmd_port ok (UART5)\n");
}

static void init_pc_port()
{
    g_pc_port = new (g_pc_port_buf) PcPort(&huart3);
    if (g_pc_port->init_status)
    {
        g_pc_port->startUartReceiveIT();
        g_debug.printf("[app] PC port ok (USART3)\n");
    }
    else
    {
        g_debug.printf("[app] PC port init failed\n");
    }
}

static void init_seeder()
{
    PwmServo::HardwareConfig servo_cfg;
    servo_cfg.servo_id = PwmServoBsp::SERVO_A;
    servo_cfg.max_angle_deg = 270.0f;
    servo_cfg.center_compare = 1825.0f;
    servo_cfg.compare_delta = 115.0f;
    g_seeder_servo = new (g_servo_buf) PwmServo(servo_cfg);
    g_seeder_servo->set_angle(135.0f);

    PwmMotor::HardwareConfig motor_hw;
    motor_hw.motor_id = PwmEncBsp::MOTOR_A;
    motor_hw.gear_ratio = 30.0f;
    motor_hw.encoder_counts_per_rev = 52u;
    motor_hw.control_period_s = 0.002f;
    motor_hw.direction_sign = 1.0f;

    PwmMotor::SpeedPidConfig motor_pid;
    motor_pid.kp = 1200.0f;
    motor_pid.ki = 0.0f;
    motor_pid.kd = 5.0f;
    motor_pid.integral_limit = 3000.0f;
    motor_pid.output_limit = 65535.0f;
    motor_pid.deadzone = 0.5f;
    motor_pid.integral_separation_threshold = 50.0f;

    g_seeder_motor = new (g_motor_buf) PwmMotor(motor_hw, motor_pid);

    // 右侧舵机：SERVO_B
    PwmServo::HardwareConfig servo_r_cfg;
    servo_r_cfg.servo_id = PwmServoBsp::SERVO_B;
    servo_r_cfg.max_angle_deg = 270.0f;
    servo_r_cfg.center_compare = 1825.0f;
    servo_r_cfg.compare_delta = 115.0f;
    g_seeder_servo_r = new (g_servo_r_buf) PwmServo(servo_r_cfg);
    g_seeder_servo_r->set_angle(135.0f);

    // 右侧电机：MOTOR_B（方向与左相反）
    PwmMotor::HardwareConfig motor_r_hw;
    motor_r_hw.motor_id = PwmEncBsp::MOTOR_D;
    motor_r_hw.gear_ratio = 30.0f;
    motor_r_hw.encoder_counts_per_rev = 52u;
    motor_r_hw.control_period_s = 0.002f;
    motor_r_hw.direction_sign = 1.0f;

    PwmMotor::SpeedPidConfig motor_r_pid;
    motor_r_pid.kp = 1200.0f;
    motor_r_pid.ki = 0.0f;
    motor_r_pid.kd = 5.0f;
    motor_r_pid.integral_limit = 3000.0f;
    motor_r_pid.output_limit = 65535.0f;
    motor_r_pid.deadzone = 0.5f;
    motor_r_pid.integral_separation_threshold = 50.0f;

    g_seeder_motor_r = new (g_motor_r_buf) PwmMotor(motor_r_hw, motor_r_pid);

    g_debug.printf("[app] seeder ok (L:A+L:A + R:B+R:B)\n");
}

static void init_mowing()
{
    PwmEsc::HardwareConfig esc_cfg;
    esc_cfg.esc_id = PwmEscBsp::ESC_A;
    esc_cfg.min_throttle_compare = 1000u;
    esc_cfg.max_throttle_compare = 2000u;
    g_mowing_esc = new (g_esc_buf) PwmEsc(esc_cfg);
    g_mowing_esc->emergency_stop();
    g_debug.printf("[app] mowing ok (ESC_A)\n");
}

static void init_spray()
{
    PwmOpenLoopMotor::HardwareConfig cfg;
    cfg.motor_id = PwmEncBsp::MOTOR_B;
    cfg.direction = PwmOpenLoopMotor::Direction::Forward; // B 口正向已实测确认。
    cfg.min_throttle_percent = 70.0f; // 实测起转油门，非零 PC 油门映射到 70..100%。
    g_spray_pump = new (g_spray_buf) PwmOpenLoopMotor(cfg);
    g_debug.printf("[app] spray %s (B, forward, PC throttle -> %.1f..%.1f%% PWM)\n",
                   g_spray_pump->is_valid() ? "ok" : "init failed",
                   cfg.min_throttle_percent, cfg.max_throttle_percent);
}

extern "C" void App_ControlInit(void)
{
    init_chassis();
    init_cmd_port();
    init_pc_port();
    init_seeder();
    init_mowing();
    init_spray();
}

static void run_control_loop(uint32_t now)
{
    PcProtocol::Command c = {};
    if (g_pc_port != nullptr && g_pc_port->has_control())
    {
        if (g_pc_port->command_fresh(now))
        {
            c = g_pc_port->command();
            if (!c.enable || c.stop) c = PcProtocol::Command{};
        }
        // Once USART3 has submitted a command, timeout holds a safe stop.
        // UART5 cannot silently resume stale targets.
    }
    else if (g_cmd_port != nullptr)
    {
        const CmdData &legacy = g_cmd_port->cmd;
        c.linear_mps = legacy.linear_speed;
        c.angular_radps = legacy.angular_speed;
        c.left_on = legacy.left_seeder;
        c.left_rpm = 200.0f;
        c.right_on = legacy.right_seeder;
        c.right_rpm = -200.0f;
        c.mowing_on = legacy.mowing;
        c.mowing_percent = 20.0f;
    }
    if (g_spray_pump != nullptr)
    {
        (void)g_spray_pump->set_throttle(c.spray_on ? c.spray_percent : 0.0f);
        // 状态回报逻辑油门，映射后的 PWM 留在驱动中，避免二次映射。
        c.spray_percent = g_spray_pump->get_target_throttle();
        c.spray_on = c.spray_percent > 0.0f;
    }
    else
    {
        c.spray_on = false;
        c.spray_percent = 0.0f;
    }
    g_applied = c;

    // 水泵独立执行；其余装置保留原有底盘初始化检查。
    if (g_chassis == nullptr) return;

    g_can_tx_failed = !g_chassis->set_cmd_vel(c.linear_mps, c.angular_radps);

    if (g_seeder_servo != nullptr)
    {
        if (c.left_on)
        {
            g_seeder_servo->set_angle(-80.0f);
        }
        else
        {
            g_seeder_servo->set_angle(135.0f);
        }
    }

    if (g_seeder_motor != nullptr)
    {
        if (c.left_on)
        {
            g_seeder_motor->control_speed(c.left_rpm);
        }
        else
        {
            g_seeder_motor->control_speed(0.0f);
        }
    }

    // right_seeder
    if (g_seeder_servo_r != nullptr)
    {
        if (c.right_on)
        {
            g_seeder_servo_r->set_angle(-80.0f);
        }
        else
        {
            g_seeder_servo_r->set_angle(135.0f);
        }
    }

    if (g_seeder_motor_r != nullptr)
    {
        if (c.right_on)
        {
            g_seeder_motor_r->control_speed(c.right_rpm);
        }
        else
        {
            g_seeder_motor_r->control_speed(0.0f);
        }
    }

    // mowing (ESC_A)
    if (g_mowing_esc != nullptr)
    {
        if (c.mowing_on)
        {
            g_mowing_esc->set_throttle(c.mowing_percent);
        }
        else
        {
            g_mowing_esc->set_throttle(0.0f);
        }
    }
}

static void send_pc_status(uint32_t now)
{
    if (g_pc_port == nullptr || !g_pc_port->init_status) return;
    static uint32_t last_status = 0u;
    if (now - last_status < 50u) return;
    last_status = now;

    PcProtocol::Status s = {};
    s.uptime_ms = now;
    s.last_command_seq = g_pc_port->last_command_seq();
    s.result = g_pc_port->result();
    s.mode = g_pc_port->has_control() ?
        (g_pc_port->command_fresh(now) && g_pc_port->command().enable && !g_pc_port->command().stop ? 1u : 2u) : 0u;
    s.capabilities = PcProtocol::CAP_CHASSIS | PcProtocol::CAP_LEFT_SEEDER |
                     PcProtocol::CAP_RIGHT_SEEDER | PcProtocol::CAP_MOWING;
    if (g_spray_pump != nullptr && g_spray_pump->is_valid()) s.capabilities |= PcProtocol::CAP_SPRAY;
    if (g_pc_port->has_control() && !g_pc_port->command_fresh(now)) s.faults |= PcProtocol::FAULT_PC_TIMEOUT;
    if (g_can_tx_failed) s.faults |= PcProtocol::FAULT_CAN_TX;
    if (g_pc_port->rx_overflow()) s.faults |= PcProtocol::FAULT_RX_OVERFLOW;
    const uint32_t age = g_pc_port->command_age_ms(now);
    s.command_age_ms = age > 65535u ? 65535u : static_cast<uint16_t>(age);
    s.rx_error_count = g_pc_port->error_count();
    s.linear_mps = g_applied.linear_mps;
    s.angular_radps = g_applied.angular_radps;
    if (g_chassis != nullptr)
    {
        const M3508::State &ls = g_chassis->left_motor()->get_state();
        const M3508::State &rs = g_chassis->right_motor()->get_state();
        s.left_target_rpm = ls.target_output_rpm;
        s.right_target_rpm = rs.target_output_rpm;
        s.left_actual_rpm = ls.output_rpm;
        s.right_actual_rpm = rs.output_rpm;
    }
    s.left_seeder_on = g_applied.left_on;
    s.left_seeder_target_rpm = g_applied.left_on ? g_applied.left_rpm : 0.0f;
    if (g_seeder_motor != nullptr) s.left_seeder_actual_rpm = g_seeder_motor->get_current_rpm();
    if (g_seeder_servo != nullptr) s.left_servo_angle_deg = g_seeder_servo->get_current_angle();
    s.right_seeder_on = g_applied.right_on;
    s.right_seeder_target_rpm = g_applied.right_on ? g_applied.right_rpm : 0.0f;
    if (g_seeder_motor_r != nullptr) s.right_seeder_actual_rpm = g_seeder_motor_r->get_current_rpm();
    if (g_seeder_servo_r != nullptr) s.right_servo_angle_deg = g_seeder_servo_r->get_current_angle();
    s.mowing_on = g_applied.mowing_on;
    if (g_mowing_esc != nullptr) s.mowing_percent = g_mowing_esc->get_current_throttle();
    s.spray_on = g_applied.spray_on;
    s.spray_target_percent = g_applied.spray_percent;
    // 无传感器反馈，不能将 PWM 油门冒充实测值。
    s.spray_actual_percent = 0.0f;
    s.spray_feedback_valid = false;
    // Lift is reserved; no actuator or feedback is connected yet.
    (void)g_pc_port->send_status(s);
}

extern "C" void App_ControlStep(void)
{
    const uint32_t now = HAL_GetTick();
    if (g_pc_port != nullptr) g_pc_port->poll(now);
    run_control_loop(now);
    send_pc_status(now);

    if (g_chassis == nullptr)
    {
        return;
    }

    static uint32_t last_print = 0u;
    if ((now - last_print) >= 500u)
    {
        last_print = now;

        const M3508::State &ls = g_chassis->left_motor()->get_state();
        const M3508::State &rs = g_chassis->right_motor()->get_state();

        g_debug.printf("[ctrl] v=%.2f w=%.2f l=%d r=%d m=%d | L:%.0frpm %d | R:%.0frpm %d\n",
                       g_applied.linear_mps, g_applied.angular_radps,
                       g_applied.left_on, g_applied.right_on, g_applied.mowing_on,
                       ls.output_rpm, ls.command,
                       rs.output_rpm, rs.command);
    }
}

// ==================== Test Injection ====================

#if APP_TEST_MODE

static uint32_t g_test_start_tick = 0u;

extern "C" void App_TestInjectStep(void)
{
    if (g_cmd_port == nullptr)
    {
        return;
    }

    const uint32_t now = HAL_GetTick();

    if (g_test_start_tick == 0u)
    {
        g_test_start_tick = now;
        g_debug.printf("[test] start. phases:\n");
        g_debug.printf("  stop -> forward -> backward -> turn_left -> turn_right -> fwd+seeder -> stop\n");
    }

    const uint32_t elapsed = now - g_test_start_tick;

    float v = 0.0f;
    float w = 0.0f;
    bool seeder = false;

    if (elapsed < 3000u)
    {
        // stop
    }
    else if (elapsed < 6000u)
    {
        v = 0.20f;
    }
    else if (elapsed < 9000u)
    {
        v = -0.20f;
    }
    else if (elapsed < 12000u)
    {
        w = 1.00f;
    }
    else if (elapsed < 15000u)
    {
        w = -1.00f;
    }
    else if (elapsed < 19000u)
    {
        v = 0.20f;
        seeder = true;
    }
    else if (elapsed < 22000u)
    {
        // stop + seeder off
    }
    else
    {
        g_test_start_tick = now;
        return;
    }

    g_cmd_port->cmd.linear_speed = v;
    g_cmd_port->cmd.angular_speed = w;
    g_cmd_port->cmd.left_seeder = seeder;
    g_cmd_port->cmd.right_seeder = seeder;
    g_cmd_port->cmd.mowing = seeder;
}

#else  // !APP_TEST_MODE

extern "C" void App_TestInjectStep(void)
{
    // UART ISR writes to g_cmd_port->cmd directly.
    // This function is a no-op in real mode.
}

#endif
