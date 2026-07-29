#include "pwm_servo.h"

PwmServo::HardwareConfig::HardwareConfig()
    : servo_id(PwmServoBsp::SERVO_A),
      max_angle_deg(180.0f),
      center_compare(1825.0f),
      compare_delta(115.0f)
{
}

PwmServo::PwmServo(const HardwareConfig &hardware_config)
    : bsp_(hardware_config.servo_id),
      hardware_config_(hardware_config),
      current_angle_deg_(0.0f)
{
    if (hardware_config_is_valid())
    {
        (void)reset();
    }
}

bool PwmServo::set_angle(float angle_deg)
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    const float limited_angle_deg = limit_angle(angle_deg);
    const float half_angle_deg = hardware_config_.max_angle_deg * 0.5f;
    float compare_value =
        hardware_config_.center_compare -
        (limited_angle_deg / half_angle_deg) * hardware_config_.compare_delta;

    if (compare_value < 0.0f)
    {
        compare_value = 0.0f;
    }

    compare_value_ = compare_value;
    bsp_.set_compare(static_cast<uint32_t>(compare_value));
    current_angle_deg_ = limited_angle_deg;
    return true;
}

bool PwmServo::reset()
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    float compare_value = hardware_config_.center_compare;
    if (compare_value < 0.0f)
    {
        compare_value = 0.0f;
    }

    bsp_.set_compare(static_cast<uint32_t>(compare_value));
    current_angle_deg_ = 0.0f;
    compare_value_ = compare_value;
    return true;
}

float PwmServo::get_current_angle() const
{
    return current_angle_deg_;
}

float PwmServo::limit_angle(float angle_deg) const
{
    const float half_angle_deg = hardware_config_.max_angle_deg * 0.5f;
    if (angle_deg > half_angle_deg)
    {
        return half_angle_deg;
    }

    if (angle_deg < -half_angle_deg)
    {
        return -half_angle_deg;
    }

    return angle_deg;
}

bool PwmServo::hardware_config_is_valid() const
{
    if (!bsp_.is_valid())
    {
        return false;
    }

    if (hardware_config_.max_angle_deg <= 0.0f)
    {
        return false;
    }

    if (hardware_config_.center_compare < 0.0f)
    {
        return false;
    }

    if (hardware_config_.compare_delta < 0.0f)
    {
        return false;
    }

    return true;
}
