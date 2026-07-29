#ifndef PWM_ESC_BSP_H
#define PWM_ESC_BSP_H

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

class PwmEscBsp
{
public:
    // 电调预设编号，仅负责固定硬件映射关系。
    enum EscId
    {
        ESC_A = 0,
        ESC_B
    };

    explicit PwmEscBsp(EscId esc_id);

    void set_compare(uint32_t compare_value);
    bool is_valid() const;

private:
    void bind_esc_config(EscId esc_id);
    void start_hardware();
    uint32_t clamp_compare_value(uint32_t compare_value) const;

private:
    TIM_HandleTypeDef *tim_;
    uint32_t channel_;
    uint32_t period_;
    bool is_valid_;
};

#endif

#endif