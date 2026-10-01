#include "../TerraMind_FW/Driver/pc_protocol.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <limits>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static void put_f32(uint8_t *p, float value) { memcpy(p, &value, 4); }

static uint16_t control_frame(uint8_t *out, bool lift_on, bool spray_on = false, float spray_percent = 0.0f)
{
    uint8_t payload[64] = {};
    uint16_t n = 0;
    payload[n++] = 0x01; payload[n++] = 1; payload[n++] = 1;
    payload[n++] = 0x10; payload[n++] = 8;
    put_f32(payload + n, 0.2f); n += 4;
    put_f32(payload + n, 0.0f); n += 4;
    payload[n++] = 0x20; payload[n++] = 5; payload[n++] = 1;
    put_f32(payload + n, 200.0f); n += 4;
    payload[n++] = 0x21; payload[n++] = 5; payload[n++] = 0;
    put_f32(payload + n, -200.0f); n += 4;
    payload[n++] = 0x30; payload[n++] = 5; payload[n++] = 0;
    put_f32(payload + n, 20.0f); n += 4;
    payload[n++] = 0x40; payload[n++] = 5; payload[n++] = lift_on ? 1 : 0;
    put_f32(payload + n, 0.0f); n += 4;
    payload[n++] = 0x50; payload[n++] = 5; payload[n++] = spray_on ? 1 : 0;
    put_f32(payload + n, spray_percent); n += 4;
    out[0] = 0xfc; out[1] = 0xfb; out[2] = 1; out[3] = 1;
    out[4] = 0x34; out[5] = 0x12;
    out[6] = static_cast<uint8_t>(n); out[7] = 0;
    memcpy(out + 8, payload, n);
    const uint16_t crc = PcProtocol::crc16(out + 2, n + 6);
    out[8 + n] = static_cast<uint8_t>(crc);
    out[9 + n] = static_cast<uint8_t>(crc >> 8);
    out[10 + n] = 0xfd; out[11 + n] = 0xfe;
    return n + 12;
}

int main(int argc, char **argv)
{
    assert(PcProtocol::crc16(reinterpret_cast<const uint8_t *>("123456789"), 9) == 0x31c3);
    uint8_t frame[PcProtocol::MAX_FRAME] = {};
    uint16_t size = control_frame(frame, false);
    PcProtocol::StreamParser parser;
    PcProtocol::Command command = {};
    uint16_t seq = 0;
    PcProtocol::Result result = PcProtocol::RESULT_BAD_FRAME;
    for (uint16_t i = 0; i < size; ++i)
    {
        const bool complete = parser.feed(frame[i], command, seq, result);
        assert(complete == (i == size - 1));
    }
    assert(seq == 0x1234 && result == PcProtocol::RESULT_OK);
    assert(command.enable && command.left_on && !command.lift_on);
    assert(command.linear_mps > 0.19f && command.linear_mps < 0.21f);
    frame[15] ^= 1;
    for (uint16_t i = 0; i < size; ++i)
        assert(!parser.feed(frame[i], command, seq, result));
    assert(parser.error_count() == 1);

    size = control_frame(frame, true);
    bool completed = false;
    for (uint16_t i = 0; i < size; ++i)
        completed = parser.feed(frame[i], command, seq, result) || completed;
    assert(completed && result == PcProtocol::RESULT_UNSUPPORTED);
    assert(!command.lift_on); // Rejected frames cannot partially apply.

    size = control_frame(frame, false);
    // Replace chassis linear speed with +infinity and repair CRC: semantic
    // validation must still reject the whole snapshot.
    frame[13] = 0x00; frame[14] = 0x00; frame[15] = 0x80; frame[16] = 0x7f;
    const uint16_t bad_value_crc = PcProtocol::crc16(frame + 2, size - 6);
    frame[size - 4] = static_cast<uint8_t>(bad_value_crc);
    frame[size - 3] = static_cast<uint8_t>(bad_value_crc >> 8);
    completed = false;
    for (uint16_t i = 0; i < size; ++i)
        completed = parser.feed(frame[i], command, seq, result) || completed;
    assert(completed && result == PcProtocol::RESULT_BAD_VALUE);
    assert(command.linear_mps > 0.19f && command.linear_mps < 0.21f);

    // Newly supported spray block accepts the whole snapshot, including the
    // existing chassis and seeder fields. Invalid throttle applies nothing.
    size = control_frame(frame, false, true, 35.0f);
    completed = false;
    for (uint16_t i = 0; i < size; ++i)
        completed = parser.feed(frame[i], command, seq, result) || completed;
    assert(completed && result == PcProtocol::RESULT_OK);
    assert(command.spray_on && command.spray_percent == 35.0f && command.left_on);
    const float invalid[] = {-1.0f, 101.0f, std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity()};
    for (float value : invalid)
    {
        size = control_frame(frame, false, true, value);
        completed = false;
        for (uint16_t i = 0; i < size; ++i)
            completed = parser.feed(frame[i], command, seq, result) || completed;
        assert(completed && result == PcProtocol::RESULT_BAD_VALUE);
        assert(command.spray_on && command.spray_percent == 35.0f);
    }

    PcProtocol::Status status = {};
    status.capabilities = PcProtocol::CAP_CHASSIS;
    uint8_t encoded[PcProtocol::MAX_FRAME] = {};
    const uint16_t written = PcProtocol::encode_status(status, 7, encoded, sizeof(encoded));
    assert(written == 117);
    assert(encoded[0] == 0xfc && encoded[1] == 0xfb);
    assert(encoded[3] == PcProtocol::STATUS_TYPE && encoded[4] == 7);
    assert(encoded[written - 2] == 0xfd && encoded[written - 1] == 0xfe);
    assert(PcProtocol::crc16(encoded + 2, written - 6) ==
           static_cast<uint16_t>(encoded[written - 4] | (encoded[written - 3] << 8)));
    if (argc > 1 && strcmp(argv[1], "--emit-status") == 0)
    {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(fwrite(encoded, 1, written, stdout) == written);
    }
    if (argc > 1 && strcmp(argv[1], "--emit-control") == 0)
    {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        const uint16_t control_size = control_frame(frame, false);
        assert(fwrite(frame, 1, control_size, stdout) == control_size);
    }
    return 0;
}
