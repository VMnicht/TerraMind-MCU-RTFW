#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "../BSP/pwm_servo_bsp.h"

class PwmServo
{
public:
    struct HardwareConfig
    {
        // 舵机连接到哪个固定接口口位。
        PwmServoBsp::ServoId servo_id;

        // 舵机总角度范围，单位为度。
        // 为保持与参考工程一致，内部会自动取半程作为有效输入范围。
        float max_angle_deg;

        // 舵机回中时输出的 compare 值。
        float center_compare;

        // 从中位到端点时的 compare 变化量。
        float compare_delta;

        HardwareConfig();
    };

public:
    explicit PwmServo(const HardwareConfig &hardware_config);

    bool set_angle(float angle_deg);
    bool reset();
    float get_current_angle() const;

private:
    float limit_angle(float angle_deg) const;
    bool hardware_config_is_valid() const;

private:
    PwmServoBsp bsp_;
    HardwareConfig hardware_config_;
    float current_angle_deg_;
    float compare_value_;
};

#endif

#endif
