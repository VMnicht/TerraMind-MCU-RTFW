#ifndef PWM_ESC_H
#define PWM_ESC_H

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

#include "../BSP/pwm_esc_bsp.h"

class PwmEsc
{
public:
    struct HardwareConfig
    {
        // 电调连接到哪个固定接口口位。
        PwmEscBsp::EscId esc_id;

        // 油门最小脉宽对应的compare值（1ms = 1000）。
        uint32_t min_throttle_compare;

        // 油门最大脉宽对应的compare值（2ms = 2000）。
        uint32_t max_throttle_compare;

        HardwareConfig();
    };

public:
    explicit PwmEsc(const HardwareConfig &hardware_config);

    // 设置油门百分比（0-100%）。
    // 0% 对应1ms脉宽，100% 对应2ms脉宽。
    bool set_throttle(float throttle_percent);

    // 紧急停止，输出0%油门（1ms脉宽）。
    bool emergency_stop();

    // 获取当前油门百分比。
    float get_current_throttle() const;

private:
    float limit_throttle(float throttle_percent) const;
    bool hardware_config_is_valid() const;
    uint32_t throttle_to_compare(float throttle_percent) const;

private:
    PwmEscBsp bsp_;
    HardwareConfig hardware_config_;
    float current_throttle_percent_;
};

#endif

#endif