#include "app_main.h"

#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/cmd_port.h"
#include "../Driver/pwm_motor.h"
#include "../Driver/pwm_servo.h"
#include "../Driver/debug_printer.h"

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

static void init_chassis()
{
    g_chassis_cfg.wheel_track_m = 0.32f;
    g_chassis_cfg.wheel_diameter_m = 0.10f;
    g_chassis_cfg.max_linear_speed_mps = 0.35f;
    g_chassis_cfg.max_angular_speed_rad = 2.0f;

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

    g_debug.printf("[app] seeder ok (servo_A + motor_A)\n");
}

extern "C" void App_Init(void)
{
    init_chassis();
    init_cmd_port();
    init_seeder();
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
}

extern "C" void App_TaskStep(void)
{
    run_control_loop();
}

// ==================== Test Mode ====================

static bool g_test_active = false;
static uint32_t g_test_start_tick = 0u;

extern "C" void App_TestInit(void)
{
    App_Init();
    g_test_active = true;
    g_test_start_tick = HAL_GetTick();
    g_debug.printf("[test] start. phases:\n");
    g_debug.printf("  0-3s: stop\n");
    g_debug.printf("  3-6s: forward v=0.20\n");
    g_debug.printf("  6-9s: backward v=-0.20\n");
    g_debug.printf("  9-12s: turn_left w=1.0\n");
    g_debug.printf("  12-15s: turn_right w=-1.0\n");
    g_debug.printf("  15-19s: forward + seeder ON\n");
    g_debug.printf("  19-22s: stop + seeder OFF\n");
}

extern "C" void App_TestStep(void)
{
    if (!g_test_active || g_cmd_port == nullptr)
    {
        return;
    }

    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed = now - g_test_start_tick;

    float v = 0.0f;
    float w = 0.0f;
    bool seeder = false;
    const char *phase = "stop";

    if (elapsed < 3000u)
    {
        phase = "stop";
    }
    else if (elapsed < 6000u)
    {
        v = 0.20f;
        phase = "forward";
    }
    else if (elapsed < 8000u)
    {
        v = -0.20f;
        phase = "backward";
    }
    else if (elapsed < 10000u)
    {
        w = 1.00f;
        phase = "turn_left";
    }
    else if (elapsed < 12000u)
    {
        w = -1.00f;
        phase = "turn_right";
    }
    else if (elapsed < 14000u)
    {
        seeder = true;
        phase = "fwd+seeder";
    }
    else if (elapsed < 22000u)
    {
        phase = "stop+seeder_off";
    }
    else
    {
        g_test_start_tick = now;
        return;
    }

    g_cmd_port->cmd.linear_speed = v;
    g_cmd_port->cmd.angular_speed = w;
    g_cmd_port->cmd.left_seeder = seeder;

    run_control_loop();

    static uint32_t last_print = 0u;
    if ((now - last_print) >= 500u)
    {
        last_print = now;

        const M3508::State &ls = g_chassis->left_motor()->get_state();
        const M3508::State &rs = g_chassis->right_motor()->get_state();

        g_debug.printf("[%s] v=%.2f w=%.2f seed=%d | L:%.0f rpm %d | R:%.0f rpm %d\n",
                       phase, v, w, seeder,
                       ls.output_rpm, ls.command,
                       rs.output_rpm, rs.command);
    }
}
