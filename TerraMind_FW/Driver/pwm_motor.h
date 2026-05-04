#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

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

#include "../Algorithm/PID.h"
#include "../BSP/pwm_enc_bsp.h"

class PwmMotor
{
public:
    struct HardwareConfig
    {
        // 电机连接到哪个固定接口口位。
        PwmEncBsp::MotorId motor_id;

        // 减速比。
        // 例如减速箱为 19:1，则这里填写 19.0f。
        float gear_ratio;

        // 减速前，电机本体旋转一圈对应的编码器计数值。
        uint32_t encoder_counts_per_rev;

        // 速度控制调用周期，单位为秒。
        // 例如 10ms 调一次，则填写 0.01f。
        float control_period_s;

        // 速度方向符号。
        // 取值一般为 +1.0f 或 -1.0f：
        // +1.0f 表示当前编码器方向与 PWM 正方向一致；
        // -1.0f 表示当前编码器方向与 PWM 正方向相反。
        float direction_sign;

        HardwareConfig();
    };

    struct SpeedPidConfig
    {
        // 速度环 PID 参数。
        float kp;
        float ki;
        float kd;

        // 积分限幅与输出限幅。
        float integral_limit;
        float output_limit;

        // 死区与积分分离阈值。
        float deadzone;
        float integral_separation_threshold;

        SpeedPidConfig();
    };

public:
    // 构造时传入最关键的两类参数：
    // 1. 电机相关硬件参数
    // 2. 速度环 PID 参数
    PwmMotor(const HardwareConfig &hardware_config,
             const SpeedPidConfig &pid_config);

    // 速度控制接口。
    // 该函数应按固定周期被循环调用，每次调用会：
    // 1. 读取编码器增量
    // 2. 计算当前输出轴转速
    // 3. 执行速度环 PID
    // 4. 更新底层 PWM 输出
    void control_speed(float target_rpm);

    // 查询最近一次控制周期计算得到的输出轴转速，单位为 rpm。
    float get_current_rpm() const;

private:
    void update_speed_feedback();
    float convert_encoder_count_to_output_rpm(int32_t encoder_count) const;
    bool hardware_config_is_valid() const;

private:
    PwmEncBsp bsp_;
    pid speed_pid_;
    HardwareConfig hardware_config_;
    float target_rpm_;
    float current_rpm_;
    int32_t last_encoder_count_;
};

#endif

#endif
