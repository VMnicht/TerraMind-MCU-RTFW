#include "M3508.h"
#include "../BSP/can_bsp.h"

M3508::Config::Config()
    : gear_ratio(19.2032f),
      kp(32.0f),
      ki(0.76f),
      kd(8.6f),
      max_current_ma(20000.0f),
      max_can_current(16384),
      max_encoder(8191)
{
}

M3508::Meta::Meta() : motor_id(1), feedback_std_id(0x201u)
{
}

M3508::State::State()
    : target_output_rpm(0.0f),
      output_rpm(0.0f),
      rotor_rpm(0.0f),
      angle_deg(0.0f),
      multi_turn_deg(0.0f),
      current_ma(0.0f),
      temperature(0),
      command(0)
{
}

M3508::Controller::Controller()
    : angle_initialized(false),
      last_angle_deg(0.0f)
{
}

M3508::M3508(uint8_t motor_id)
    : M3508(motor_id, Config(), ControlMode::SingleMotor)
{
}

M3508::M3508(uint8_t motor_id, ControlMode mode)
    : M3508(motor_id, Config(), mode)
{
}

M3508::M3508(uint8_t motor_id, const Config &config)
    : M3508(motor_id, config, ControlMode::SingleMotor)
{
}

M3508::M3508(uint8_t motor_id, const Config &config, ControlMode mode)
    : cfg_(config),
      control_mode_(mode),
      speed_pid_(cfg_.kp, cfg_.ki, cfg_.kd, cfg_.max_current_ma, cfg_.max_current_ma, 0.0f, 1.0e9f)
{
    init_meta(motor_id);
    CAN_BUS.register_m3508(this);
}

void M3508::init_meta(uint8_t motor_id)
{
    meta_.motor_id = motor_id;
    if (meta_.motor_id < 1)
    {
        meta_.motor_id = 1;
    }
    if (meta_.motor_id > 8)
    {
        meta_.motor_id = 8;
    }
    meta_.feedback_std_id = 0x200u + meta_.motor_id;
}

bool M3508::update_feedback(uint32_t std_id, const uint8_t rx_data[8])
{
    if (std_id != meta_.feedback_std_id)
    {
        return false;
    }
    update_feedback_data(rx_data);
    return true;
}

void M3508::update_feedback_data(const uint8_t rx_data[8])
{
    // DJI反馈帧格式: angle(0-1), rpm(2-3), current(4-5), temp(6)。
    const uint16_t raw_angle = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    const int16_t raw_rpm = static_cast<int16_t>((rx_data[2] << 8) | rx_data[3]);
    const int16_t raw_current = static_cast<int16_t>((rx_data[4] << 8) | rx_data[5]);

    state_.angle_deg = (static_cast<float>(raw_angle) / static_cast<float>(cfg_.max_encoder)) * 360.0f;
    state_.rotor_rpm = static_cast<float>(raw_rpm);
    state_.output_rpm = state_.rotor_rpm / cfg_.gear_ratio;
    state_.current_ma = can_to_current_ma(raw_current);
    state_.temperature = rx_data[6];

    update_multi_turn(state_.angle_deg);
}

void M3508::set_target_rpm(float output_rpm)
{
    state_.target_output_rpm = output_rpm;
    if (control_mode_ == ControlMode::Component)
    {
        return;
    }
    const int16_t cmd = speed_control_step();
    (void)send_current_command(cmd);
}

void M3508::set_pid(float kp, float ki, float kd)
{
    cfg_.kp = kp;
    cfg_.ki = ki;
    cfg_.kd = kd;
    speed_pid_.PID_SetParameters(kp, ki, kd);
}

void M3508::reset_controller()
{
    speed_pid_.error_sum = 0.0f;
    speed_pid_.previous_error = 0.0f;
    speed_pid_.output = 0.0f;
    speed_pid_.last_output = 0.0f;
    speed_pid_.error = 0.0f;
}

void M3508::set_control_mode(ControlMode mode)
{
    control_mode_ = mode;
}

M3508::ControlMode M3508::get_control_mode() const
{
    return control_mode_;
}

int16_t M3508::speed_control_step()
{
    const float target_rotor_rpm = state_.target_output_rpm * cfg_.gear_ratio;
    const float error = target_rotor_rpm - state_.rotor_rpm;
    const float current_ma = speed_pid_.PID_ComputeError(error);

    state_.command = current_ma_to_can(current_ma);
    return state_.command;
}

void M3508::pack_can_command(int16_t c1, int16_t c2, int16_t c3, int16_t c4, uint8_t tx_data[8])
{
    tx_data[0] = static_cast<uint8_t>((c1 >> 8) & 0xFF);
    tx_data[1] = static_cast<uint8_t>(c1 & 0xFF);
    tx_data[2] = static_cast<uint8_t>((c2 >> 8) & 0xFF);
    tx_data[3] = static_cast<uint8_t>(c2 & 0xFF);
    tx_data[4] = static_cast<uint8_t>((c3 >> 8) & 0xFF);
    tx_data[5] = static_cast<uint8_t>(c3 & 0xFF);
    tx_data[6] = static_cast<uint8_t>((c4 >> 8) & 0xFF);
    tx_data[7] = static_cast<uint8_t>(c4 & 0xFF);
}

bool M3508::send_current_command(int16_t current)
{
    int16_t currents[4] = {0, 0, 0, 0};
    const uint32_t std_id = (meta_.motor_id <= 4u) ? 0x200u : 0x1FFu;
    const uint8_t index = (meta_.motor_id <= 4u) ? (meta_.motor_id - 1u) : (meta_.motor_id - 5u);
    currents[index] = current;

    CanBsp::TxHeader tx_header;
    tx_header.id = std_id;
    tx_header.is_extended_id = false;
    tx_header.is_remote_frame = false;
    tx_header.dlc = 8u;
    tx_header.transmit_global_time = false;

    uint8_t tx_data[8] = {0};
    pack_can_command(currents[0], currents[1], currents[2], currents[3], tx_data);
    return CAN_BUS.send_raw(tx_header, tx_data);
}

const M3508::State &M3508::get_state() const
{
    return state_;
}

uint8_t M3508::get_motor_id() const
{
    return meta_.motor_id;
}

bool M3508::is_component_mode() const
{
    return control_mode_ == ControlMode::Component;
}

void M3508::update_multi_turn(float now_angle_deg)
{
    // 通过跨0点处理实现多圈角度累计。
    if (!ctrl_.angle_initialized)
    {
        ctrl_.last_angle_deg = now_angle_deg;
        ctrl_.angle_initialized = true;
        return;
    }

    float delta = now_angle_deg - ctrl_.last_angle_deg;
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    state_.multi_turn_deg += delta;
    ctrl_.last_angle_deg = now_angle_deg;
}

float M3508::can_to_current_ma(int16_t can_current) const
{
    return (static_cast<float>(can_current) / static_cast<float>(cfg_.max_can_current)) * cfg_.max_current_ma;
}

int16_t M3508::current_ma_to_can(float current_ma) const
{
    float out = (current_ma / cfg_.max_current_ma) * static_cast<float>(cfg_.max_can_current);
    if (out > static_cast<float>(cfg_.max_can_current))
    {
        out = static_cast<float>(cfg_.max_can_current);
    }
    else if (out < -static_cast<float>(cfg_.max_can_current))
    {
        out = -static_cast<float>(cfg_.max_can_current);
    }
    return static_cast<int16_t>(out);
}
