#include "pwm_enc_bsp.h"

PwmEncBsp::PwmEncBsp(MotorId motor_id)
    : config_{0},
      pwm_period_ch1_(0u),
      pwm_period_ch2_(0u),
      is_valid_(false)
{
    bind_motor_config(motor_id);
    if (!is_valid_)
    {
        return;
    }

    // 读取 PWM 定时器当前自动重装载值，后续用于输出限幅。
    pwm_period_ch1_ = __HAL_TIM_GET_AUTORELOAD(config_.pwm_tim_ch1);
    pwm_period_ch2_ = __HAL_TIM_GET_AUTORELOAD(config_.pwm_tim_ch2);

    start_hardware();

    // 构造完成后先关闭两路输出，避免对象刚创建时电机误动作。
    set_pwm_output(0);

    // 将编码器计数器清零，保证第一次读取获得的是从对象创建后开始的增量值。
    __HAL_TIM_SET_COUNTER(config_.encoder_tim, 0u);
}

void PwmEncBsp::set_pwm_output(int32_t pwm_output)
{
    if (!is_valid_)
    {
        return;
    }

    if (pwm_output > 0)
    {
        // 正转时仅 CH1 输出 PWM，CH2 保持关闭。
        __HAL_TIM_SET_COMPARE(
            config_.pwm_tim_ch1,
            config_.pwm_channel_ch1,
            clamp_compare_value(static_cast<int64_t>(pwm_output), pwm_period_ch1_));
        __HAL_TIM_SET_COMPARE(config_.pwm_tim_ch2, config_.pwm_channel_ch2, 0u);
        return;
    }

    if (pwm_output < 0)
    {
        // 反转时仅 CH2 输出 PWM，CH1 保持关闭。
        __HAL_TIM_SET_COMPARE(config_.pwm_tim_ch1, config_.pwm_channel_ch1, 0u);
        __HAL_TIM_SET_COMPARE(
            config_.pwm_tim_ch2,
            config_.pwm_channel_ch2,
            clamp_compare_value(static_cast<int64_t>(-pwm_output), pwm_period_ch2_));
        return;
    }

    // 目标为 0 时关闭两路输出，避免双边同时导通导致电机制动或蜂鸣。
    __HAL_TIM_SET_COMPARE(config_.pwm_tim_ch1, config_.pwm_channel_ch1, 0u);
    __HAL_TIM_SET_COMPARE(config_.pwm_tim_ch2, config_.pwm_channel_ch2, 0u);
}

int32_t PwmEncBsp::get_encoder_count()
{
    if (!is_valid_)
    {
        return 0;
    }

    // 读取当前编码器计数后立即清零，返回值表示“自上次调用以来”的增量。
    const uint32_t raw_count = __HAL_TIM_GET_COUNTER(config_.encoder_tim);
    const int32_t signed_count = convert_encoder_count(raw_count);
    __HAL_TIM_SET_COUNTER(config_.encoder_tim, 0u);
    return signed_count;
}

void PwmEncBsp::bind_motor_config(MotorId motor_id)
{
    config_.pwm_tim_ch1 = 0;
    config_.pwm_tim_ch2 = 0;
    config_.pwm_channel_ch1 = 0u;
    config_.pwm_channel_ch2 = 0u;
    config_.encoder_tim = 0;
    config_.encoder_channel_a = 0u;
    config_.encoder_channel_b = 0u;
    is_valid_ = true;

    switch (motor_id)
    {
    case MOTOR_A:
        // 电机 A:
        // PWM: PB8 -> TIM10_CH1, PB9 -> TIM11_CH1
        // ENC: PA15/PB3 -> TIM2_CH1/TIM2_CH2
        config_.pwm_tim_ch1 = &htim10;
        config_.pwm_tim_ch2 = &htim11;
        config_.pwm_channel_ch1 = TIM_CHANNEL_1;
        config_.pwm_channel_ch2 = TIM_CHANNEL_1;
        config_.encoder_tim = &htim2;
        config_.encoder_channel_a = TIM_CHANNEL_1;
        config_.encoder_channel_b = TIM_CHANNEL_2;
        break;

    case MOTOR_B:
        // 电机 B:
        // PWM: PE5/PE6 -> TIM9_CH1/TIM9_CH2
        // ENC: PB4/PB5 -> TIM3_CH1/TIM3_CH2
        config_.pwm_tim_ch1 = &htim9;
        config_.pwm_tim_ch2 = &htim9;
        config_.pwm_channel_ch1 = TIM_CHANNEL_1;
        config_.pwm_channel_ch2 = TIM_CHANNEL_2;
        config_.encoder_tim = &htim3;
        config_.encoder_channel_a = TIM_CHANNEL_1;
        config_.encoder_channel_b = TIM_CHANNEL_2;
        break;

    case MOTOR_C:
        // 电机 C:
        // PWM: PE9/PE11 -> TIM1_CH1/TIM1_CH2
        // ENC: PB6/PB7 -> TIM4_CH1/TIM4_CH2
        config_.pwm_tim_ch1 = &htim1;
        config_.pwm_tim_ch2 = &htim1;
        config_.pwm_channel_ch1 = TIM_CHANNEL_1;
        config_.pwm_channel_ch2 = TIM_CHANNEL_2;
        config_.encoder_tim = &htim4;
        config_.encoder_channel_a = TIM_CHANNEL_1;
        config_.encoder_channel_b = TIM_CHANNEL_2;
        break;

    case MOTOR_D:
        // 电机 D:
        // PWM: PE13/PE14 -> TIM1_CH3/TIM1_CH4
        // ENC: PA0/PA1 -> TIM5_CH1/TIM5_CH2
        config_.pwm_tim_ch1 = &htim1;
        config_.pwm_tim_ch2 = &htim1;
        config_.pwm_channel_ch1 = TIM_CHANNEL_3;
        config_.pwm_channel_ch2 = TIM_CHANNEL_4;
        config_.encoder_tim = &htim5;
        config_.encoder_channel_a = TIM_CHANNEL_1;
        config_.encoder_channel_b = TIM_CHANNEL_2;
        break;

    default:
        is_valid_ = false;
        break;
    }
}

void PwmEncBsp::start_hardware()
{
    if (!is_valid_)
    {
        return;
    }

    // 启动电机对应的两路 PWM 输出。
    (void)HAL_TIM_PWM_Start(config_.pwm_tim_ch1, config_.pwm_channel_ch1);
    (void)HAL_TIM_PWM_Start(config_.pwm_tim_ch2, config_.pwm_channel_ch2);

    // 启动编码器接口的两个通道。
    (void)HAL_TIM_Encoder_Start(config_.encoder_tim, config_.encoder_channel_a);
    (void)HAL_TIM_Encoder_Start(config_.encoder_tim, config_.encoder_channel_b);
}

int32_t PwmEncBsp::convert_encoder_count(uint32_t raw_count) const
{
    if (!is_valid_)
    {
        return 0;
    }

    // 编码器模式下，计数器反转时常常会以补码形式表现为“大数”。
    // 这里按当前 ARR 推算计数器位宽，将无符号原始值转换成有符号增量。
    const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(config_.encoder_tim);
    const uint64_t full_range = static_cast<uint64_t>(auto_reload) + 1ULL;
    const uint64_t half_range = full_range / 2ULL;

    if (static_cast<uint64_t>(raw_count) >= half_range)
    {
        return static_cast<int32_t>(static_cast<int64_t>(raw_count) - static_cast<int64_t>(full_range));
    }

    return static_cast<int32_t>(raw_count);
}

uint32_t PwmEncBsp::clamp_compare_value(int64_t compare_value, uint32_t period) const
{
    if (compare_value <= 0)
    {
        return 0u;
    }

    if (compare_value >= static_cast<int64_t>(period))
    {
        return period;
    }

    return static_cast<uint32_t>(compare_value);
}
