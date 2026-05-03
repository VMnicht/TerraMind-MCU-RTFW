#include "test.h"

#include "../BSP/can_bsp.h"
#include "../Driver/M3508.h"
#include "../Driver/debug_printer.h"

M3508 g_motor_3508_1(1);
DebugPrinter g_debug(&huart1); // 假设使用 USART1 进行调试打印
uint32_t g_last_switch_tick = 0;
float g_target_output_rpm = 50.0f;
float g_cmd_target_rpm = 0.0f;
bool g_test_ready = false;


extern "C" void AppTest_M3508_Init(void)
{
    if (!CAN_BUS.init_default(&hcan1))
    {
        g_test_ready = false;
        return;
    }

    // 使用更保守的测试参数，优先保证运行平稳。
    g_motor_3508_1.set_pid(10.0f, 0.0f, 0.5f);
    g_motor_3508_1.reset_controller();
    g_motor_3508_1.set_target_rpm(0.0f);
    g_cmd_target_rpm = 0.0f;
    g_last_switch_tick = HAL_GetTick();
    g_test_ready = true;

    g_debug.printf("M3508 Test Initialized. Target: %.2f RPM\n", g_target_output_rpm);
}

extern "C" void AppTest_M3508_TaskStep(void)
{
    if (!g_test_ready)
    {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
//    if ((now_tick - g_last_switch_tick) >= 5000u)
//    {
//        g_last_switch_tick = now_tick;
//        g_target_output_rpm = -g_target_output_rpm;
//    }

    // 目标转速斜坡，避免突变目标引发抖动。
    const float ramp_step_rpm = 0.08f;
    if (g_cmd_target_rpm < g_target_output_rpm)
    {
        g_cmd_target_rpm += ramp_step_rpm;
        if (g_cmd_target_rpm > g_target_output_rpm)
        {
            g_cmd_target_rpm = g_target_output_rpm;
        }
    }
    else if (g_cmd_target_rpm > g_target_output_rpm)
    {
        g_cmd_target_rpm -= ramp_step_rpm;
        if (g_cmd_target_rpm < g_target_output_rpm)
        {
            g_cmd_target_rpm = g_target_output_rpm;
        }
    }

    g_motor_3508_1.set_target_rpm(g_cmd_target_rpm);

    // 每隔 500ms 打印一次电机状态
    static uint32_t last_print_tick = 0;
    if (now_tick - last_print_tick >= 500u)
    {
        last_print_tick = now_tick;
        const M3508::State &state = g_motor_3508_1.get_state();
        g_debug.printf("[M3508] Cmd: %.1f, Output: %.1f, Rotor: %.1f, Temp: %d\n",
                       g_cmd_target_rpm, state.output_rpm, state.rotor_rpm, state.temperature);
    }
}
