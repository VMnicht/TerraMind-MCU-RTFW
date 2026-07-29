#ifndef CMD_PORT_H
#define CMD_PORT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "../BSP/Serial_Device.h"
#include <string.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define CMD_FRAME_HEAD_0 0xFC
#define CMD_FRAME_HEAD_1 0xFB
#define CMD_FRAME_END_0  0xFD
#define CMD_FRAME_END_1  0xFE
#define CMD_DATA_LENGTH  11
#define CMD_RX_BUF_SIZE  64

struct CmdData
{
    float linear_speed;
    float angular_speed;
    bool left_seeder;
    bool right_seeder;
    bool mowing;
};

class cmd_port : public SerialDevice
{
public:
    cmd_port(UART_HandleTypeDef *huartx);

    void handleReceiveData(uint8_t byte) override;

    CmdData cmd;

private:
    void parse_data(const uint8_t *dat, uint8_t len);

    uint8_t rx_buf_[CMD_RX_BUF_SIZE];
    uint8_t rx_index_;
    uint8_t data_length_;
    uint8_t frame_id_;
    union
    {
        uint16_t crc_code;
        uint8_t crc_buf[2];
    } crc_;
    enum RxState
    {
        WAIT_HEAD_0,
        WAIT_HEAD_1,
        WAIT_ID,
        WAIT_LEN,
        WAIT_DATA,
        WAIT_CRC_0,
        WAIT_CRC_1,
        WAIT_END_0,
        WAIT_END_1
    } state_;
};

#endif

#endif
