// Compile the real application in this translation unit to inspect its internal
// applied commands. Only HAL and the CAN transport are replaced by host fakes.
#include "../TerraMind_FW/App/app_main.cpp"
#include <assert.h>
#include <math.h>
#include <string.h>

static uint8_t last_can[8] = {};
CanBsp::TxHeader::TxHeader() : id(0), is_extended_id(false), is_remote_frame(false),
    dlc(8), transmit_global_time(false) {}
CanBsp::CanBsp() : hcan_(nullptr), instances_{} {}
CanBsp &CanBsp::instance() { static CanBsp bus; return bus; }
bool CanBsp::init_default(CAN_HandleTypeDef *hcan) { hcan_ = hcan; return true; }
void CanBsp::register_m3508(M3508 *) {}
bool CanBsp::send_raw(const TxHeader &, uint8_t data[8])
{
    memcpy(last_can, data, 8);
    return true;
}

static void append_float(uint8_t *&p, float v) { memcpy(p, &v, 4); p += 4; }
static void submit_pc(bool enable, bool stop, bool spray_on, float percent,
                      float linear = 0.1f, float angular = 0.3f)
{
    uint8_t frame[60] = {0xfc, 0xfb, 1, 1, 1, 0, 48, 0};
    uint8_t *p = frame + 8;
    *p++ = 0x01; *p++ = 1; *p++ = (enable ? 1 : 0) | (stop ? 2 : 0);
    *p++ = 0x10; *p++ = 8; append_float(p, linear); append_float(p, angular);
    *p++ = 0x20; *p++ = 5; *p++ = 1; append_float(p, 125.0f);
    *p++ = 0x21; *p++ = 5; *p++ = 1; append_float(p, -150.0f);
    *p++ = 0x30; *p++ = 5; *p++ = 1; append_float(p, 30.0f);
    *p++ = 0x40; *p++ = 5; *p++ = 0; append_float(p, 0.0f);
    *p++ = 0x50; *p++ = 5; *p++ = spray_on ? 1 : 0; append_float(p, percent);
    const uint16_t crc = PcProtocol::crc16(frame + 2, 54);
    *p++ = static_cast<uint8_t>(crc); *p++ = static_cast<uint8_t>(crc >> 8);
    *p++ = 0xfd; *p++ = 0xfe;
    assert(p == frame + sizeof(frame));
    for (uint8_t byte : frame) g_pc_port->handleReceiveData(byte);
    App_ControlStep();
    assert(g_pc_port->result() == PcProtocol::RESULT_OK);
}

static void expect_pump(float percent)
{
    const float duty = percent > 0.0f ? 70.0f + 0.3f * percent : 0.0f;
    assert(fabsf(g_spray_pump->get_applied_throttle() - duty) < 0.001f);
    assert(g_spray_pump->get_target_throttle() == percent);
    assert(g_applied.spray_percent == percent);
    const uint32_t expected = static_cast<uint32_t>(duty * htim9.period / 100.0f + 0.5f);
    assert(htim9.compare[0] == expected && htim9.compare[1] == 0u);
}

static void expect_pc_others()
{
    assert(g_applied.linear_mps == 0.1f && g_applied.angular_radps == 0.3f);
    assert(g_applied.left_on && g_applied.left_rpm == 125.0f);
    assert(g_applied.right_on && g_applied.right_rpm == -150.0f);
    assert(g_applied.mowing_on && g_mowing_esc->get_current_throttle() == 30.0f);
    assert(g_seeder_servo->get_current_angle() == -80.0f);
    assert(g_seeder_servo_r->get_current_angle() == -80.0f);
    const auto expected = g_chassis->calc_wheel_target_rpm(0.1f, 0.3f);
    assert(g_chassis->left_motor()->get_state().target_output_rpm == expected.left_rpm);
    assert(g_chassis->right_motor()->get_state().target_output_rpm == expected.right_rpm);
}

static void other_outputs(uint32_t out[15])
{
    out[0] = htim11.compare[0]; out[1] = htim10.compare[0];
    out[2] = htim1.compare[2]; out[3] = htim1.compare[3];
    out[4] = htim12.compare[0]; out[5] = htim12.compare[1];
    out[6] = htim13.compare[0];
    for (unsigned i = 0; i < 8; ++i) out[7 + i] = last_can[i];
}

static void check_chassis_direction(bool pc)
{
    // Independently calculated motor-ID RPMs for the calibrated physical robot:
    // straight motion stays the same; turns reverse relative to the old firmware.
    const float cases[][4] = {
        {0.2f, 0.0f, 38.197186f, -38.197186f},
        {-0.2f, 0.0f, -38.197186f, 38.197186f},
        {0.0f, 1.0f, 30.557749f, 30.557749f},
        {0.0f, -1.0f, -30.557749f, -30.557749f},
        {0.2f, 1.0f, 68.754936f, -7.639437f},
        {0.2f, -1.0f, 7.639437f, -68.754936f},
        {0.0f, 0.0f, 0.0f, 0.0f},
    };
    for (const auto &c : cases)
    {
        if (pc)
            submit_pc(true, false, false, 0.0f, c[0], c[1]);
        else
        {
            g_cmd_port->cmd.linear_speed = c[0];
            g_cmd_port->cmd.angular_speed = c[1];
            App_ControlStep();
        }
        assert(fabsf(g_chassis->left_motor()->get_state().target_output_rpm - c[2]) < 0.001f);
        assert(fabsf(g_chassis->right_motor()->get_state().target_output_rpm - c[3]) < 0.001f);
        // Protocol/status retains the caller's sign; compensation happens once.
        assert(g_applied.linear_mps == c[0] && g_applied.angular_radps == c[1]);
        expect_pump(0.0f);
    }
}

int main()
{
    TIM_HandleTypeDef *timers[] = {&htim1, &htim2, &htim3, &htim4, &htim5, &htim8,
                                  &htim9, &htim10, &htim11, &htim12, &htim13, &htim14};
    for (auto tim : timers) tim->period = 65535u;
    App_ControlInit();
    assert(htim3.encoder_starts == 0u);
    assert(htim2.encoder_starts == 2u && htim5.encoder_starts == 2u);
    expect_pump(0.0f); // Initialization itself never applies throttle.
    test_tick = 10u;
    App_ControlStep();
    expect_pump(0.0f);
    check_chassis_direction(false);

    // UART5 still drives the original devices; it never starts the PC-only pump.
    g_cmd_port->cmd.linear_speed = 0.15f;
    g_cmd_port->cmd.angular_speed = -0.4f;
    g_cmd_port->cmd.left_seeder = true;
    g_cmd_port->cmd.right_seeder = false;
    g_cmd_port->cmd.mowing = true;
    test_tick = 20u;
    App_ControlStep();
    assert(g_applied.linear_mps == 0.15f && g_applied.angular_radps == -0.4f);
    assert(g_applied.left_on && !g_applied.right_on && g_applied.mowing_on);
    assert(g_seeder_servo->get_current_angle() == -80.0f);
    assert(g_seeder_servo_r->get_current_angle() == 135.0f);
    assert(g_mowing_esc->get_current_throttle() == 20.0f);
    expect_pump(0.0f);

    test_tick = 100u;
    submit_pc(true, false, true, 35.0f);
    expect_pc_others();
    expect_pump(35.0f);
    // Inspect the actual status frame emitted by App_ControlStep().
    assert(test_uart_tx_size == 117u);
    const uint8_t *spray = nullptr;
    for (unsigned pos = 8; pos < 113; pos += 2u + test_uart_tx[pos + 1])
    {
        const uint8_t *data = test_uart_tx + pos + 2;
        if (test_uart_tx[pos] == 0x01) assert(data[10] & PcProtocol::CAP_SPRAY);
        if (test_uart_tx[pos] == 0x50) spray = data;
    }
    assert(spray && spray[0] == 1u && spray[9] == 0u);
    float target = 0.0f, actual = -1.0f;
    memcpy(&target, spray + 1, 4); memcpy(&actual, spray + 5, 4);
    assert(target == g_applied.spray_percent && actual == 0.0f);

    // Let the existing PID derivative settle, then verify changing only spray
    // leaves all other PWM channels and the complete chassis CAN frame unchanged.
    App_ControlStep();
    uint32_t baseline[15], current[15];
    other_outputs(baseline);
    const float commands[] = {0.0f, 1.0f, 20.0f, 50.0f, 100.0f};
    for (float throttle : commands)
    {
        ++test_tick;
        submit_pc(true, false, true, throttle);
        expect_pump(throttle);
        expect_pc_others();
        other_outputs(current);
        assert(memcmp(baseline, current, sizeof(baseline)) == 0);
    }

    test_tick = 110u;
    submit_pc(true, false, false, 35.0f);
    expect_pc_others();
    expect_pump(0.0f);
    test_tick = 120u;
    submit_pc(true, true, true, 35.0f);
    expect_pump(0.0f);
    assert(g_applied.linear_mps == 0.0f && !g_applied.left_on && !g_applied.mowing_on);
    test_tick = 130u;
    submit_pc(false, false, true, 35.0f);
    expect_pump(0.0f);
    test_tick = 140u;
    submit_pc(true, false, true, 35.0f);
    test_tick = 390u; App_ControlStep();
    expect_pump(35.0f);
    test_tick = 391u; App_ControlStep();
    expect_pump(0.0f); // Timeout must not fall back to stale UART5 commands.
    assert(g_applied.linear_mps == 0.0f && !g_applied.left_on);

    test_tick = 395u;
    check_chassis_direction(true);

    // Pump control and stop still run when no chassis object is available.
    g_chassis = nullptr;
    test_tick = 400u; submit_pc(true, false, true, 35.0f);
    expect_pump(35.0f);
    test_tick = 651u; App_ControlStep();
    expect_pump(0.0f);
}
