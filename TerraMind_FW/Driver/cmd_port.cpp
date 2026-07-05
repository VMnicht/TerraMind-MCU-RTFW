#include "cmd_port.h"
#include "../Utils/crc_util.h"
#include <string.h>

cmd_port::cmd_port(UART_HandleTypeDef *huartx)
    : SerialDevice(huartx),
      rx_index_(0u),
      data_length_(0u),
      frame_id_(0u),
      state_(WAIT_HEAD_0)
{
    memset(&cmd, 0, sizeof(cmd));
    memset(rx_buf_, 0, sizeof(rx_buf_));
    crc_.crc_code = 0u;
}

void cmd_port::handleReceiveData(uint8_t byte)
{
    switch (state_)
    {
    case WAIT_HEAD_0:
        if (byte == CMD_FRAME_HEAD_0)
        {
            state_ = WAIT_HEAD_1;
        }
        break;

    case WAIT_HEAD_1:
        if (byte == CMD_FRAME_HEAD_1)
        {
            state_ = WAIT_ID;
        }
        else
        {
            state_ = WAIT_HEAD_0;
        }
        break;

    case WAIT_ID:
        frame_id_ = byte;
        state_ = WAIT_LEN;
        break;

    case WAIT_LEN:
        data_length_ = byte;
        if (data_length_ > CMD_RX_BUF_SIZE)
        {
            state_ = WAIT_HEAD_0;
        }
        else
        {
            rx_index_ = 0u;
            state_ = WAIT_DATA;
        }
        break;

    case WAIT_DATA:
        rx_buf_[rx_index_++] = byte;
        if (rx_index_ >= data_length_)
        {
            state_ = WAIT_CRC_0;
        }
        break;

    case WAIT_CRC_0:
        crc_.crc_buf[0] = byte;
        state_ = WAIT_CRC_1;
        break;

    case WAIT_CRC_1:
        crc_.crc_buf[1] = byte;
        state_ = WAIT_END_0;
        break;

    case WAIT_END_0:
        if (byte == CMD_FRAME_END_0)
        {
            state_ = WAIT_END_1;
        }
        else
        {
            state_ = WAIT_HEAD_0;
        }
        break;

    case WAIT_END_1:
        if (byte == CMD_FRAME_END_1)
        {
//            const uint16_t calc_crc = CRC16_Table(rx_buf_, data_length_);
//            if (calc_crc == crc_.crc_code)
//            {
//                
//            }
					parse_data(rx_buf_, data_length_);
        }
        state_ = WAIT_HEAD_0;
        break;

    default:
        state_ = WAIT_HEAD_0;
        break;
    }
}

void cmd_port::parse_data(const uint8_t *dat, uint8_t len)
{
    if (len < CMD_DATA_LENGTH)
    {
        return;
    }

    float linear = 0.0f;
    float angular = 0.0f;
    memcpy(&linear, &dat[0], sizeof(float));
    memcpy(&angular, &dat[4], sizeof(float));

    cmd.linear_speed = linear;
    cmd.angular_speed = angular;
    cmd.left_seeder = (dat[8] != 0u);
    cmd.right_seeder = (dat[9] != 0u);
    cmd.mowing = (dat[10] != 0u);
}
