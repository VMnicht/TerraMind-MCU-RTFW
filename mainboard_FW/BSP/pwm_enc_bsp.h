#ifndef PWM_ENC_BSP_H
#define PWM_ENC_BSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class PwmEncBsp
{
public:
    // 电机预设编号，仅负责固定硬件映射关系，不包含任何电机控制策略。
    enum MotorId
    {
        MOTOR_A = 0,
        MOTOR_B,
        MOTOR_C,
        MOTOR_D
    };

    // 构造时只需要传入电机编号，类内部会自动绑定对应的 PWM 与编码器资源。
    explicit PwmEncBsp(MotorId motor_id);

    // 设置电机两路 PWM 输出。
    // 约定传入值为有符号控制量：
    // pwm_output > 0 时，CH1 输出 PWM，CH2 关闭；
    // pwm_output < 0 时，CH2 输出 PWM，CH1 关闭；
    // pwm_output = 0 时，两路都关闭。
    void set_pwm_output(int32_t pwm_output);

    // 获取编码器自上次读取以来的增量计数，并在读取后自动清零计数器。
    int32_t get_encoder_count();

private:
    struct MotorConfig
    {
        TIM_HandleTypeDef *pwm_tim_ch1;
        TIM_HandleTypeDef *pwm_tim_ch2;
        uint32_t pwm_channel_ch1;
        uint32_t pwm_channel_ch2;

        TIM_HandleTypeDef *encoder_tim;
        uint32_t encoder_channel_a;
        uint32_t encoder_channel_b;
    };

    void bind_motor_config(MotorId motor_id);
    void start_hardware();
    int32_t convert_encoder_count(uint32_t raw_count) const;
    uint32_t clamp_compare_value(int64_t compare_value, uint32_t period) const;

private:
    MotorConfig config_;
    uint32_t pwm_period_ch1_;
    uint32_t pwm_period_ch2_;
    bool is_valid_;
};

#endif

#endif
