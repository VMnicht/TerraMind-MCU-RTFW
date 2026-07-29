#include "pwm_motor.h"

PwmMotor::HardwareConfig::HardwareConfig()
    : motor_id(PwmEncBsp::MOTOR_A),
      gear_ratio(1.0f),
      encoder_counts_per_rev(1u),
      control_period_s(0.01f),
      direction_sign(1.0f)
{
}

PwmMotor::SpeedPidConfig::SpeedPidConfig()
    : kp(0.0f),
      ki(0.0f),
      kd(0.0f),
      integral_limit(10000.0f),
      output_limit(30000.0f),
      deadzone(0.0f),
      integral_separation_threshold(1000000.0f)
{
}

PwmMotor::PwmMotor(const HardwareConfig &hardware_config,
                   const SpeedPidConfig &pid_config)
    : bsp_(hardware_config.motor_id),
      speed_pid_(
          pid_config.kp,
          pid_config.ki,
          pid_config.kd,
          pid_config.integral_limit,
          pid_config.output_limit,
          pid_config.deadzone,
          pid_config.integral_separation_threshold),
      hardware_config_(hardware_config),
      target_rpm_(0.0f),
      current_rpm_(0.0f),
      last_encoder_count_(0)
{
    // 速度环 PID 内部使用 setpoint 字段保存目标值，这里先清零，避免刚构造完成时误动作。
    speed_pid_.setpoint = 0.0f;

    // 电机对象创建后，先明确输出为 0，确保启动阶段处于安全状态。
    bsp_.set_pwm_output(0);
}

void PwmMotor::control_speed(float target_rpm)
{
    target_rpm_ = target_rpm;

    // 每次控制前先刷新一次转速反馈。
    update_speed_feedback();

    if (!hardware_config_is_valid())
    {
        bsp_.set_pwm_output(0);
        return;
    }

    // 将目标转速写入 PID，再用当前转速做闭环控制。
    speed_pid_.setpoint = target_rpm_;
    const float pwm_output = speed_pid_.PID_Compute(current_rpm_);

    // 底层 PWM 接口使用整型控制量，这里直接进行转换。
    bsp_.set_pwm_output(static_cast<int32_t>(pwm_output));
}

float PwmMotor::get_current_rpm() const
{
    return current_rpm_;
}

void PwmMotor::update_speed_feedback()
{
    if (!hardware_config_is_valid())
    {
        last_encoder_count_ = 0;
        current_rpm_ = 0.0f;
        return;
    }

    // 读取“本控制周期内”的编码器增量，并换算成输出轴 rpm。
    last_encoder_count_ = bsp_.get_encoder_count();
    current_rpm_ = convert_encoder_count_to_output_rpm(last_encoder_count_);
}

float PwmMotor::convert_encoder_count_to_output_rpm(int32_t encoder_count) const
{
    if (!hardware_config_is_valid())
    {
        return 0.0f;
    }

    // 换算逻辑说明：
    // 1. encoder_count / encoder_counts_per_rev -> 本周期电机本体转过的圈数
    // 2. 再除以控制周期 -> 电机本体转速 rps
    // 3. 再乘以 60 -> 电机本体 rpm
    // 4. 最后除以减速比 -> 输出轴 rpm
    const float motor_revolutions =
        static_cast<float>(encoder_count) / static_cast<float>(hardware_config_.encoder_counts_per_rev);
    const float motor_rps = motor_revolutions / hardware_config_.control_period_s;
    const float motor_rpm = motor_rps * 60.0f;
    return (motor_rpm / hardware_config_.gear_ratio) * hardware_config_.direction_sign;
}

bool PwmMotor::hardware_config_is_valid() const
{
    if (hardware_config_.gear_ratio <= 0.0f)
    {
        return false;
    }

    if (hardware_config_.encoder_counts_per_rev == 0u)
    {
        return false;
    }

    if (hardware_config_.control_period_s <= 0.0f)
    {
        return false;
    }

    if (hardware_config_.direction_sign == 0.0f)
    {
        return false;
    }

    return true;
}
