#include "pwm_esc.h"

PwmEsc::HardwareConfig::HardwareConfig()
    : esc_id(PwmEscBsp::ESC_A),
      min_throttle_compare(1000u),
      max_throttle_compare(2000u)
{
}

PwmEsc::PwmEsc(const HardwareConfig &hardware_config)
    : bsp_(hardware_config.esc_id),
      hardware_config_(hardware_config),
      current_throttle_percent_(0.0f)
{
    if (hardware_config_is_valid())
    {
        (void)emergency_stop();
    }
}

bool PwmEsc::set_throttle(float throttle_percent)
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    const float limited_throttle = limit_throttle(throttle_percent);
    const uint32_t compare_value = throttle_to_compare(limited_throttle);

    bsp_.set_compare(compare_value);
    current_throttle_percent_ = limited_throttle;
    return true;
}

bool PwmEsc::emergency_stop()
{
    if (!hardware_config_is_valid())
    {
        return false;
    }

    const uint32_t compare_value = throttle_to_compare(0.0f);
    bsp_.set_compare(compare_value);
    current_throttle_percent_ = 0.0f;
    return true;
}

float PwmEsc::get_current_throttle() const
{
    return current_throttle_percent_;
}

float PwmEsc::limit_throttle(float throttle_percent) const
{
    if (throttle_percent > 100.0f)
    {
        return 100.0f;
    }

    if (throttle_percent < 0.0f)
    {
        return 0.0f;
    }

    return throttle_percent;
}

bool PwmEsc::hardware_config_is_valid() const
{
    if (!bsp_.is_valid())
    {
        return false;
    }

    if (hardware_config_.min_throttle_compare >= hardware_config_.max_throttle_compare)
    {
        return false;
    }

    return true;
}

uint32_t PwmEsc::throttle_to_compare(float throttle_percent) const
{
    const float range = static_cast<float>(hardware_config_.max_throttle_compare - hardware_config_.min_throttle_compare);
    const float compare_value = static_cast<float>(hardware_config_.min_throttle_compare) + (throttle_percent / 100.0f) * range;
    return static_cast<uint32_t>(compare_value);
}