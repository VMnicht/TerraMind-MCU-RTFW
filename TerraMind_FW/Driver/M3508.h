#ifndef M3508_H
#define M3508_H

#include <stdint.h>
#include "../Algorithm/PID.h"

#ifdef __cplusplus

class M3508
{
public:
    enum class ControlMode : uint8_t
    {
        SingleMotor = 0u, // 设置目标后立即发送控制指令。
        Component = 1u    // 仅更新目标，由外部manager统一发送。
    };

    // 电机与控制参数，默认值可直接使用。
    struct Config
    {
        float gear_ratio;
        float kp;
        float ki;
        float kd;
        float max_current_ma;
        int16_t max_can_current;
        uint16_t max_encoder;

        Config();
    };

    // 电机实时状态与最近一次控制输出。
    struct State
    {
        float target_output_rpm;
        float output_rpm;
        float rotor_rpm;
        float angle_deg;
        float multi_turn_deg;
        float current_ma;
        uint8_t temperature;
        int16_t command;

        State();
    };

    explicit M3508(uint8_t motor_id);
    explicit M3508(uint8_t motor_id, ControlMode mode);
    explicit M3508(uint8_t motor_id, const Config &config);
    explicit M3508(uint8_t motor_id, const Config &config, ControlMode mode);

    bool update_feedback(uint32_t std_id, const uint8_t rx_data[8]);
    void update_feedback_data(const uint8_t rx_data[8]);

    void set_target_rpm(float output_rpm);
    void set_pid(float kp, float ki, float kd);
    void reset_controller();
    void set_control_mode(ControlMode mode);
    ControlMode get_control_mode() const;

    int16_t speed_control_step();
    static void pack_can_command(int16_t c1, int16_t c2, int16_t c3, int16_t c4, uint8_t tx_data[8]);
    bool send_current_command(int16_t current);

    const State &get_state() const;
    uint8_t get_motor_id() const;
    bool is_component_mode() const;

private:
    // 实例级固定信息（ID与对应反馈报文ID）。
    struct Meta
    {
        uint8_t motor_id;
        uint32_t feedback_std_id;

        Meta();
    };

    // 速度环内部量（避免散落在类成员中）。
    struct Controller
    {
        bool angle_initialized;
        float last_angle_deg;

        Controller();
    };

    Config cfg_;
    Meta meta_;
    State state_;
    ControlMode control_mode_;
    Controller ctrl_;
    pid speed_pid_;

    void update_multi_turn(float now_angle_deg);
    float can_to_current_ma(int16_t can_current) const;
    int16_t current_ma_to_can(float current_ma) const;
    void init_meta(uint8_t motor_id);
};

#endif

#endif
