#include "app_main.h"

#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/cmd_port.h"
#include "../Driver/pwm_motor.h"
#include "../Driver/pwm_servo.h"
#include "../Driver/debug_printer.h"
#include "../Driver/pwm_esc.h"

#include <new>

static DebugPrinter g_debug(&huart3);

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

extern "C" void App_ControlInit(void)
{
    init_chassis();
    init_cmd_port();
    init_seeder();
    init_mowing();
}

static void run_control_loop()
{
    if (g_cmd_port == nullptr || g_chassis == nullptr)
    {
        return;
    }

    const CmdData &c = g_cmd_port->cmd;

    g_chassis->set_cmd_vel(c.linear_speed, c.angular_speed);

    if (g_seeder_servo != nullptr)
    {
        if (c.left_seeder)
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
        if (c.left_seeder)
        {
            g_seeder_motor->control_speed(200.0f);
        }
        else
        {
            g_seeder_motor->control_speed(0.0f);
        }
    }

    // right_seeder
    if (g_seeder_servo_r != nullptr)
    {
        if (c.right_seeder)
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
        if (c.right_seeder)
        {
            g_seeder_motor_r->control_speed(-200.0f);
        }
        else
        {
            g_seeder_motor_r->control_speed(0.0f);
        }
    }

    // mowing (ESC_A)
    if (g_mowing_esc != nullptr)
    {
        if (c.mowing)
        {
            g_mowing_esc->set_throttle(20.0f);
        }
        else
        {
            g_mowing_esc->set_throttle(0.0f);
        }
    }
}

extern "C" void App_ControlStep(void)
{
    run_control_loop();

    if (g_chassis == nullptr)
    {
        return;
    }

    static uint32_t last_print = 0u;
    const uint32_t now = HAL_GetTick();
    if ((now - last_print) >= 500u)
    {
        last_print = now;

        const CmdData &c = g_cmd_port->cmd;
        const M3508::State &ls = g_chassis->left_motor()->get_state();
        const M3508::State &rs = g_chassis->right_motor()->get_state();

        g_debug.printf("[ctrl] v=%.2f w=%.2f l=%d r=%d m=%d | L:%.0frpm %d | R:%.0frpm %d\n",
                       c.linear_speed, c.angular_speed,
                       c.left_seeder, c.right_seeder, c.mowing,
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
