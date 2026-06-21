#include "pwm_esc_bsp.h"

PwmEscBsp::PwmEscBsp(EscId esc_id)
    : tim_(0),
      channel_(0u),
      period_(0u),
      is_valid_(false)
{
    bind_esc_config(esc_id);
    if (!is_valid_)
    {
        return;
    }

    period_ = __HAL_TIM_GET_AUTORELOAD(tim_);
    start_hardware();
}

void PwmEscBsp::set_compare(uint32_t compare_value)
{
    if (!is_valid_)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(tim_, channel_, clamp_compare_value(compare_value));
}

bool PwmEscBsp::is_valid() const
{
    return is_valid_;
}

void PwmEscBsp::bind_esc_config(EscId esc_id)
{
    tim_ = 0;
    channel_ = 0u;
    period_ = 0u;
    is_valid_ = true;

    switch (esc_id)
    {
    case ESC_A:
        tim_ = &htim13;
        channel_ = TIM_CHANNEL_1;
        break;

    case ESC_B:
        tim_ = &htim14;
        channel_ = TIM_CHANNEL_1;
        break;

    default:
        is_valid_ = false;
        break;
    }
}

void PwmEscBsp::start_hardware()
{
    if (!is_valid_)
    {
        return;
    }

    (void)HAL_TIM_PWM_Start(tim_, channel_);
}

uint32_t PwmEscBsp::clamp_compare_value(uint32_t compare_value) const
{
    if (compare_value >= period_)
    {
        return period_;
    }

    return compare_value;
}