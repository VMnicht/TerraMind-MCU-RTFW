#ifndef PWM_OPEN_LOOP_MOTOR_H
#define PWM_OPEN_LOOP_MOTOR_H

#include "../BSP/pwm_enc_bsp.h"

// 固定方向的开环直流电机：逻辑油门映射到 PWM，没有转速反馈。
class PwmOpenLoopMotor
{
public:
    enum class Direction { Forward, Reverse };
    struct HardwareConfig
    {
        PwmEncBsp::MotorId motor_id = PwmEncBsp::MOTOR_B;
        Direction direction = Direction::Forward;
        // 非零逻辑油门 0..100% 映射到此 PWM 区间；零油门始终停机。
        float min_throttle_percent = 0.0f;
        float max_throttle_percent = 100.0f;
    };

    explicit PwmOpenLoopMotor(const HardwareConfig &config);
    bool set_throttle(float percent);
    void stop();
    bool is_valid() const;
    // 限幅后的逻辑油门，用于回报 PC 目标值。
    float get_target_throttle() const;
    // 映射后的 PWM 设定值，不是实测反馈。
    float get_applied_throttle() const;

private:
    PwmEncBsp bsp_;
    HardwareConfig config_;
    float target_throttle_;
    float applied_throttle_;
};

#endif
