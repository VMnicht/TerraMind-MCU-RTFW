#include "test.h"

#include "../BSP/can_bsp.h"
#include "../Device/diff_chassis.h"
#include "../Driver/debug_printer.h"

namespace
{
diff_chassis::MechanicalConfig g_chassis_cfg;
}

diff_chassis g_chassis(1, 2, g_chassis_cfg);
DebugPrinter g_debug(&huart1);
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
