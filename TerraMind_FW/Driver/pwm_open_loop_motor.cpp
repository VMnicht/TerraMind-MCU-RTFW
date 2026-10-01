#include "pwm_open_loop_motor.h"
#include <float.h>

PwmOpenLoopMotor::PwmOpenLoopMotor(const HardwareConfig &config)
    : bsp_(config.motor_id, false), config_(config), target_throttle_(0.0f), applied_throttle_(0.0f)
{
    stop();
}

bool PwmOpenLoopMotor::is_valid() const
{
    return bsp_.is_valid() &&
           (config_.direction == Direction::Forward || config_.direction == Direction::Reverse) &&
           config_.min_throttle_percent >= 0.0f &&
           config_.min_throttle_percent <= config_.max_throttle_percent &&
           config_.max_throttle_percent <= 100.0f;
}

bool PwmOpenLoopMotor::set_throttle(float percent)
{
    if (!is_valid() || !(percent >= -FLT_MAX && percent <= FLT_MAX))
    {
        stop();
        return false;
    }
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    const float duty = percent > 0.0f ? config_.min_throttle_percent +
        (config_.max_throttle_percent - config_.min_throttle_percent) * percent / 100.0f : 0.0f;
    const float signed_percent = config_.direction == Direction::Reverse ? -duty : duty;
    if (!bsp_.set_duty_percent(signed_percent))
    {
        stop();
        return false;
    }
    target_throttle_ = percent;
    applied_throttle_ = duty;
    return true;
}

void PwmOpenLoopMotor::stop()
{
    bsp_.set_pwm_output(0);
    target_throttle_ = 0.0f;
    applied_throttle_ = 0.0f;
}

float PwmOpenLoopMotor::get_applied_throttle() const { return applied_throttle_; }
float PwmOpenLoopMotor::get_target_throttle() const { return target_throttle_; }
