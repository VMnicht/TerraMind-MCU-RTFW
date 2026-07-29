#include "pwm_servo_bsp.h"

PwmServoBsp::PwmServoBsp(ServoId servo_id)
    : tim_(0),
      channel_(0u),
      period_(0u),
      is_valid_(false)
{
    bind_servo_config(servo_id);
    if (!is_valid_)
    {
        return;
    }

    period_ = __HAL_TIM_GET_AUTORELOAD(tim_);
    start_hardware();
}

void PwmServoBsp::set_compare(uint32_t compare_value)
{
    if (!is_valid_)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(tim_, channel_, clamp_compare_value(compare_value));
}

bool PwmServoBsp::is_valid() const
{
    return is_valid_;
}

void PwmServoBsp::bind_servo_config(ServoId servo_id)
{
    tim_ = 0;
    channel_ = 0u;
    period_ = 0u;
    is_valid_ = true;

    switch (servo_id)
    {
    case SERVO_A:
        tim_ = &htim12;
        channel_ = TIM_CHANNEL_2;
        break;

    case SERVO_B:
        tim_ = &htim12;
        channel_ = TIM_CHANNEL_1;
        break;

    case SERVO_C:
        tim_ = &htim8;
        channel_ = TIM_CHANNEL_4;
        break;

    case SERVO_D:
        tim_ = &htim8;
        channel_ = TIM_CHANNEL_3;
        break;

    default:
        is_valid_ = false;
        break;
    }
}

void PwmServoBsp::start_hardware()
{
    if (!is_valid_)
    {
        return;
    }

    (void)HAL_TIM_PWM_Start(tim_, channel_);
}

uint32_t PwmServoBsp::clamp_compare_value(uint32_t compare_value) const
{
    if (compare_value >= period_)
    {
        return period_;
    }

    return compare_value;
}
