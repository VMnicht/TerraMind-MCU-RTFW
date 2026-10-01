#include "pc_protocol.h"
#include <string.h>

namespace PcProtocol
{
namespace
{
uint16_t read_u16(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

float read_f32(const uint8_t *p)
{
    float value;
    memcpy(&value, p, sizeof(value));
    return value;
}

bool in_range(float value, float lo, float hi)
{
    return value >= lo && value <= hi; // Also rejects NaN and infinity.
}

void put_u16(uint8_t *&p, uint16_t value)
{
    *p++ = static_cast<uint8_t>(value);
    *p++ = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t *&p, uint32_t value)
{
    for (uint8_t i = 0; i < 4u; ++i)
    {
        *p++ = static_cast<uint8_t>(value >> (8u * i));
    }
}

void put_f32(uint8_t *&p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(p, bits);
}

void block(uint8_t *&p, uint8_t id, uint8_t len)
{
    *p++ = id;
    *p++ = len;
}
}

uint16_t crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u; // CRC-16/XMODEM, poly 0x1021, no reflection/xorout.
    while (length-- != 0u)
    {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t i = 0; i < 8u; ++i)
        {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool decode_control(const uint8_t *payload, uint16_t length, Command &out, Result &result)
{
    Command next = {};
    uint8_t seen = 0u;
    uint16_t pos = 0u;
    result = RESULT_BAD_FRAME;
    while (pos < length)
    {
        if (static_cast<uint16_t>(length - pos) < 2u)
            return false;
        const uint8_t id = payload[pos++];
        const uint8_t len = payload[pos++];
        if (length - pos < len)
            return false;
        const uint8_t *p = payload + pos;
        uint8_t bit = 0u;
        uint8_t expected = 0u;
        switch (id)
        {
        case 0x01u:
            bit = 1u << 0; expected = 1u;
            if (len == expected)
            {
                if ((p[0] & ~0x03u) != 0u) return false;
                next.enable = (p[0] & 0x01u) != 0u;
                next.stop = (p[0] & 0x02u) != 0u;
            }
            break;
        case 0x10u:
            bit = 1u << 1; expected = 8u;
            if (len == expected)
            {
                next.linear_mps = read_f32(p);
                next.angular_radps = read_f32(p + 4);
            }
            break;
        case 0x20u:
        case 0x21u:
            bit = (id == 0x20u) ? (1u << 2) : (1u << 3);
            expected = 5u;
            if (len == expected)
            {
                if (p[0] > 1u) return false;
                if (id == 0x20u)
                {
                    next.left_on = p[0] != 0u;
                    next.left_rpm = read_f32(p + 1);
                }
                else
                {
                    next.right_on = p[0] != 0u;
                    next.right_rpm = read_f32(p + 1);
                }
            }
            break;
        case 0x30u:
        case 0x40u:
        case 0x50u:
            bit = (id == 0x30u) ? (1u << 4) : ((id == 0x40u) ? (1u << 5) : (1u << 6));
            expected = 5u;
            if (len == expected)
            {
                if (p[0] > 1u) return false;
                if (id == 0x30u)
                {
                    next.mowing_on = p[0] != 0u;
                    next.mowing_percent = read_f32(p + 1);
                }
                else if (id == 0x40u)
                {
                    next.lift_on = p[0] != 0u;
                    next.lift_height_mm = read_f32(p + 1);
                }
                else
                {
                    next.spray_on = p[0] != 0u;
                    next.spray_percent = read_f32(p + 1);
                }
            }
            break;
        default:
            break; // Unknown future extension.
        }
        if (bit != 0u)
        {
            if (len != expected || (seen & bit) != 0u) return false;
            seen = static_cast<uint8_t>(seen | bit);
        }
        pos = static_cast<uint16_t>(pos + len);
    }
    if (seen != 0x7fu) return false;
    if (!in_range(next.linear_mps, -0.35f, 0.35f) ||
        !in_range(next.angular_radps, -2.0f, 2.0f) ||
        !in_range(next.left_rpm, -500.0f, 500.0f) ||
        !in_range(next.right_rpm, -500.0f, 500.0f) ||
        !in_range(next.mowing_percent, 0.0f, 100.0f) ||
        !in_range(next.lift_height_mm, 0.0f, 1000.0f) ||
        !in_range(next.spray_percent, 0.0f, 100.0f))
    {
        result = RESULT_BAD_VALUE;
        return false;
    }
    if (next.lift_on && next.enable && !next.stop)
    {
        result = RESULT_UNSUPPORTED;
        return false;
    }
    out = next;
    result = RESULT_OK;
    return true;
}

uint16_t encode_status(const Status &s, uint16_t seq, uint8_t *out, uint16_t capacity)
{
    if (out == 0 || capacity < 117u) return 0u;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t *p = payload;
    block(p, 0x01u, 16u);
    put_u32(p, s.uptime_ms);
    put_u16(p, s.last_command_seq);
    *p++ = s.result;
    *p++ = s.mode;
    put_u16(p, s.faults);
    put_u16(p, s.capabilities);
    put_u16(p, s.command_age_ms);
    put_u16(p, s.rx_error_count);
    block(p, 0x10u, 24u);
    put_f32(p, s.linear_mps);
    put_f32(p, s.angular_radps);
    put_f32(p, s.left_target_rpm);
    put_f32(p, s.right_target_rpm);
    put_f32(p, s.left_actual_rpm);
    put_f32(p, s.right_actual_rpm);
    block(p, 0x20u, 13u);
    *p++ = s.left_seeder_on ? 1u : 0u;
    put_f32(p, s.left_seeder_target_rpm);
    put_f32(p, s.left_seeder_actual_rpm);
    put_f32(p, s.left_servo_angle_deg);
    block(p, 0x21u, 13u);
    *p++ = s.right_seeder_on ? 1u : 0u;
    put_f32(p, s.right_seeder_target_rpm);
    put_f32(p, s.right_seeder_actual_rpm);
    put_f32(p, s.right_servo_angle_deg);
    block(p, 0x30u, 5u);
    *p++ = s.mowing_on ? 1u : 0u;
    put_f32(p, s.mowing_percent);
    block(p, 0x40u, 10u);
    *p++ = s.lift_on ? 1u : 0u;
    put_f32(p, s.lift_target_mm);
    put_f32(p, s.lift_actual_mm);
    *p++ = s.lift_feedback_valid ? 1u : 0u;
    block(p, 0x50u, 10u);
    *p++ = s.spray_on ? 1u : 0u;
    put_f32(p, s.spray_target_percent);
    put_f32(p, s.spray_actual_percent);
    *p++ = s.spray_feedback_valid ? 1u : 0u;

    const uint16_t len = static_cast<uint16_t>(p - payload);
    const uint16_t total = static_cast<uint16_t>(len + 12u);
    if (total > capacity) return 0u;
    out[0] = 0xFCu; out[1] = 0xFBu;
    out[2] = VERSION; out[3] = STATUS_TYPE;
    out[4] = static_cast<uint8_t>(seq);
    out[5] = static_cast<uint8_t>(seq >> 8);
    out[6] = static_cast<uint8_t>(len);
    out[7] = static_cast<uint8_t>(len >> 8);
    memcpy(out + 8, payload, len);
    const uint16_t crc = crc16(out + 2, static_cast<uint16_t>(len + 6u));
    out[8u + len] = static_cast<uint8_t>(crc);
    out[9u + len] = static_cast<uint8_t>(crc >> 8);
    out[10u + len] = 0xFDu;
    out[11u + len] = 0xFEu;
    return total;
}

StreamParser::StreamParser() : used_(0u), expected_(0u), errors_(0u)
{
}

void StreamParser::reset_with(uint8_t byte)
{
    used_ = 0u;
    expected_ = 0u;
    if (byte == 0xFCu) frame_[used_++] = byte;
}

bool StreamParser::feed(uint8_t byte, Command &command, uint16_t &seq, Result &result)
{
    if (used_ == 0u)
    {
        reset_with(byte);
        return false;
    }
    if (used_ == 1u)
    {
        if (byte == 0xFBu) frame_[used_++] = byte;
        else reset_with(byte);
        return false;
    }
    frame_[used_++] = byte;
    if (used_ == 8u)
    {
        const uint16_t len = read_u16(frame_ + 6);
        if (frame_[2] != VERSION || frame_[3] != CONTROL_TYPE || len > MAX_PAYLOAD)
        {
            ++errors_;
            reset_with(byte);
            return false;
        }
        expected_ = static_cast<uint16_t>(len + 12u);
    }
    if (expected_ == 0u || used_ < expected_) return false;
    const uint16_t len = read_u16(frame_ + 6);
    if (frame_[10u + len] != 0xFDu || frame_[11u + len] != 0xFEu ||
        read_u16(frame_ + 8u + len) != crc16(frame_ + 2, static_cast<uint16_t>(len + 6u)))
    {
        ++errors_;
        reset_with(byte);
        return false;
    }
    seq = read_u16(frame_ + 4);
    const bool ok = decode_control(frame_ + 8, len, command, result);
    if (!ok) ++errors_;
    reset_with(byte);
    return true;
}

uint16_t StreamParser::error_count() const
{
    return errors_;
}
}
