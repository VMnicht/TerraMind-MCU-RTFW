#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include <stdint.h>

namespace PcProtocol
{
static const uint8_t VERSION = 1u;
static const uint8_t CONTROL_TYPE = 0x01u;
static const uint8_t STATUS_TYPE = 0x81u;
static const uint16_t MAX_PAYLOAD = 256u;
static const uint16_t MAX_FRAME = MAX_PAYLOAD + 12u;
static const uint32_t COMMAND_TIMEOUT_MS = 250u;

enum Result : uint8_t
{
    RESULT_OK = 0u,
    RESULT_BAD_FRAME = 1u,
    RESULT_BAD_VALUE = 2u,
    RESULT_UNSUPPORTED = 3u,
    RESULT_TIMEOUT = 4u
};

enum Capability : uint16_t
{
    CAP_CHASSIS = 1u << 0,
    CAP_LEFT_SEEDER = 1u << 1,
    CAP_RIGHT_SEEDER = 1u << 2,
    CAP_MOWING = 1u << 3,
    CAP_LIFT = 1u << 4,
    CAP_SPRAY = 1u << 5
};

enum Fault : uint16_t
{
    FAULT_PC_TIMEOUT = 1u << 0,
    FAULT_CAN_TX = 1u << 1,
    FAULT_RX_OVERFLOW = 1u << 2
};

struct Command
{
    bool enable;
    bool stop;
    float linear_mps;
    float angular_radps;
    bool left_on;
    float left_rpm;
    bool right_on;
    float right_rpm;
    bool mowing_on;
    float mowing_percent;
    bool lift_on;
    float lift_height_mm;
    bool spray_on;
    float spray_percent; // 逻辑油门 0..100%；0 停机，非零由水泵驱动映射。
};

struct Status
{
    uint32_t uptime_ms;
    uint16_t last_command_seq;
    uint8_t result;
    uint8_t mode; // 0=legacy UART5, 1=PC active, 2=PC safe stop
    uint16_t faults;
    uint16_t capabilities;
    uint16_t command_age_ms;
    uint16_t rx_error_count;
    float linear_mps;
    float angular_radps;
    float left_target_rpm;
    float right_target_rpm;
    float left_actual_rpm;
    float right_actual_rpm;
    bool left_seeder_on;
    float left_seeder_target_rpm;
    float left_seeder_actual_rpm;
    float left_servo_angle_deg;
    bool right_seeder_on;
    float right_seeder_target_rpm;
    float right_seeder_actual_rpm;
    float right_servo_angle_deg;
    bool mowing_on;
    float mowing_percent;
    bool lift_on;
    float lift_target_mm;
    float lift_actual_mm;
    bool lift_feedback_valid;
    bool spray_on;
    float spray_target_percent; // 逻辑目标，非映射后的 PWM，不是传感器反馈。
    float spray_actual_percent;
    bool spray_feedback_valid;
};

uint16_t crc16(const uint8_t *data, uint16_t length);
bool decode_control(const uint8_t *payload, uint16_t length, Command &out, Result &result);
uint16_t encode_status(const Status &status, uint16_t seq, uint8_t *out, uint16_t capacity);

class StreamParser
{
public:
    StreamParser();
    // Returns true for a complete frame with valid envelope and CRC.
    // Semantic rejection is reported in result and leaves command unchanged.
    bool feed(uint8_t byte, Command &command, uint16_t &seq, Result &result);
    uint16_t error_count() const;

private:
    uint8_t frame_[MAX_FRAME];
    uint16_t used_;
    uint16_t expected_;
    uint16_t errors_;
    void reset_with(uint8_t byte);
};
}

#endif
