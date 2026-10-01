#include "../TerraMind_FW/Driver/pwm_open_loop_motor.h"
#include <assert.h>
#include <limits>
#include <math.h>

int main()
{
    htim9.period = 2000u; // Verify scaling uses ARR, not a hard-coded 65535.
    htim9.compare[0] = htim9.compare[1] = 123u;
    htim3.counter = 77u;
    PwmOpenLoopMotor::HardwareConfig cfg;
    PwmOpenLoopMotor pump(cfg);
    assert(pump.is_valid());
    assert(htim9.pwm_starts == 2u && htim9.nonzero_starts == 0u);
    assert(htim3.encoder_starts == 0u && htim3.counter == 77u);
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    assert(pump.set_throttle(50.0f));
    assert(htim9.compare[0] == 1000u && htim9.compare[1] == 0u);
    assert(pump.set_throttle(150.0f) && pump.get_applied_throttle() == 100.0f);
    assert(htim9.compare[0] == 2000u);
    assert(pump.set_throttle(-20.0f) && pump.get_applied_throttle() == 0.0f);
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    const float invalid[] = {std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()};
    for (float value : invalid)
    {
        assert(pump.set_throttle(50.0f));
        assert(!pump.set_throttle(value));
        assert(pump.get_applied_throttle() == 0.0f);
        assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    }
    cfg.direction = PwmOpenLoopMotor::Direction::Reverse;
    cfg.max_throttle_percent = 40.0f;
    PwmOpenLoopMotor reverse(cfg);
    assert(reverse.set_throttle(80.0f));
    assert(reverse.get_target_throttle() == 80.0f && reverse.get_applied_throttle() == 32.0f);
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 640u);
    reverse.stop();
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);

    // Pump calibration: zero remains stopped; every positive request exceeds
    // the measured 70% starting threshold, and full logical throttle is 100%.
    cfg.direction = PwmOpenLoopMotor::Direction::Forward;
    cfg.min_throttle_percent = 70.0f;
    cfg.max_throttle_percent = 100.0f;
    PwmOpenLoopMotor mapped(cfg);
    const float logical[] = {0.0f, 0.01f, 1.0f, 20.0f, 50.0f, 100.0f, 150.0f, -1.0f};
    const float physical[] = {0.0f, 70.003f, 70.3f, 76.0f, 85.0f, 100.0f, 100.0f, 0.0f};
    for (unsigned i = 0; i < sizeof(logical) / sizeof(logical[0]); ++i)
    {
        assert(mapped.set_throttle(logical[i]));
        assert(fabsf(mapped.get_applied_throttle() - physical[i]) < 0.001f);
        assert(htim9.compare[0] == static_cast<uint32_t>(physical[i] * 20.0f + 0.5f));
        assert(htim9.compare[1] == 0u);
    }
    for (float value : invalid)
    {
        assert(mapped.set_throttle(50.0f));
        assert(!mapped.set_throttle(value));
        assert(mapped.get_target_throttle() == 0.0f && mapped.get_applied_throttle() == 0.0f);
        assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    }
    assert(mapped.set_throttle(50.0f));
    mapped.stop();
    assert(mapped.get_target_throttle() == 0.0f && mapped.get_applied_throttle() == 0.0f);
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    cfg.direction = PwmOpenLoopMotor::Direction::Reverse;
    PwmOpenLoopMotor mapped_reverse(cfg);
    assert(mapped_reverse.set_throttle(50.0f));
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 1700u);
    mapped_reverse.stop();
    cfg.min_throttle_percent = 101.0f;
    PwmOpenLoopMotor inverted_range(cfg);
    assert(!inverted_range.is_valid() && !inverted_range.set_throttle(20.0f));
    cfg.min_throttle_percent = 70.0f;

    htim9.fail_start = 1;
    PwmOpenLoopMotor failed(cfg);
    assert(!failed.is_valid() && !failed.set_throttle(20.0f));
    assert(htim9.compare[0] == 0u && htim9.compare[1] == 0u);
    htim9.fail_start = 0;
    cfg.max_throttle_percent = std::numeric_limits<float>::quiet_NaN();
    PwmOpenLoopMotor bad_config(cfg);
    assert(!bad_config.is_valid() && !bad_config.set_throttle(20.0f));

    // Existing motor clients still start and read their encoder by default.
    htim10.period = htim11.period = 65535u;
    PwmEncBsp closed_loop(PwmEncBsp::MOTOR_A);
    assert(htim2.encoder_starts == 2u);
    htim2.period = 65535u;
    htim2.counter = 65534u;
    assert(closed_loop.get_encoder_count() == -2 && htim2.counter == 0u);
    closed_loop.set_pwm_output(1234);
    assert(htim11.compare[0] == 1234u && htim10.compare[0] == 0u);
    closed_loop.set_pwm_output(-1234);
    assert(htim11.compare[0] == 0u && htim10.compare[0] == 1234u);
}
