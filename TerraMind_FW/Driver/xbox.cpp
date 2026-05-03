#include "xbox.h"

// 重写从SerialDedvice里面继承的串口数据处理虚函数
void xbox::handleReceiveData(uint8_t byte)
{
    switch (state_)
    {
    // 帧头
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_0_XBOX)
        {
            state_ = WAITING_FOR_HEADER_1;
            rx_frame_mat.frame_head[0] = byte; // 存储帧头
        }
        break;
    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_1_XBOX)
        {
            state_ = WAITING_FOR_ID;
            rx_frame_mat.frame_head[1] = byte; // 存储帧头
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    // ID
    case WAITING_FOR_ID:
        rx_frame_mat.frame_id = byte; // 存储帧ID
        state_ = WAITING_FOR_LENGTH;
        break;
    // 数据长度
    case WAITING_FOR_LENGTH:
        rx_frame_mat.data_length = byte; // 存储数据长度
        rxIndex_ = 0;
        state_ = WAITING_FOR_DATA;
        break;
    // 数据接收
    case WAITING_FOR_DATA:
        rx_frame_mat.rx_temp_data_mat[rxIndex_++] = byte; // 存储接收到的数据
        if (rxIndex_ >= rx_frame_mat.data_length)
        {
            state_ = WAITING_FOR_CRC_0;
        }
        break;
    case WAITING_FOR_CRC_0:
        rx_frame_mat.check_code.crc_buff[0] = byte; // 存储CRC校验的高字节
        state_ = WAITING_FOR_CRC_1;
        break;
    case WAITING_FOR_CRC_1:
        rx_frame_mat.check_code.crc_buff[1] = byte; // 存储CRC校验的低字节
        state_ = WAITING_FOR_END_0;
        break;
    // 帧尾
    case WAITING_FOR_END_0:
        if (byte == FRAME_END_0_XBOX)
        {
            state_ = WAITING_FOR_END_1;
            rx_frame_mat.frame_end[0] = byte; // 存储帧尾
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    case WAITING_FOR_END_1:
        if (byte == FRAME_END_1_XBOX)
        {	
            rx_frame_mat.frame_end[1] = byte; // 存储帧尾

            for (uint8_t i = 0; i < rx_frame_mat.data_length; i++)
            {
                rx_frame_mat.data.buff_msg[i] = rx_frame_mat.rx_temp_data_mat[i];
            }
            btn_update(); // 注意这里是先更新了上一次的按键状态再跟新此次的按键状态
            msgs_update(rx_frame_mat.data_length, rx_frame_mat.data.buff_msg);
            joyDataCal();
            //      }
        }
        state_ = WAITING_FOR_HEADER_0;
        break;
    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

/* 更新按键上一次的按键状态 */
void xbox::btn_update(void)
{
    xbox_msgs.Y.btn_last = xbox_msgs.Y.btn;
    xbox_msgs.B.btn_last = xbox_msgs.B.btn;
    xbox_msgs.A.btn_last = xbox_msgs.A.btn;
    xbox_msgs.X.btn_last = xbox_msgs.X.btn;

    xbox_msgs.Share.btn_last  =  xbox_msgs.Share.btn;
    xbox_msgs.Start.btn_last  =  xbox_msgs.Start.btn;
    xbox_msgs.Select.btn_last =  xbox_msgs.Select.btn;
    xbox_msgs.Xbox.btn_last   =  xbox_msgs.Xbox.btn; 
																										 
    xbox_msgs.LB.btn_last = xbox_msgs.LB.btn;
    xbox_msgs.RB.btn_last = xbox_msgs.RB.btn;
    xbox_msgs.LS.btn_last = xbox_msgs.LS.btn;
    xbox_msgs.RS.btn_last = xbox_msgs.RS.btn;

    xbox_msgs.DPadUp.btn_last    = xbox_msgs.DPadUp.btn;
    xbox_msgs.DPadLeft.btn_last  = xbox_msgs.DPadLeft.btn;
    xbox_msgs.DPadRight.btn_last = xbox_msgs.DPadRight.btn;
    xbox_msgs.DPadDown.btn_last  = xbox_msgs.DPadDown.btn;
}

/* 将缓冲区的数据赋值到对应的按键中 */
void xbox::msgs_update(uint8_t len, uint8_t *dat)
{
    if (len == 28)
    {
        // A,B,X,Y按键
        xbox_msgs.Y.btn = dat[0];
        xbox_msgs.X.btn = dat[1];
        xbox_msgs.B.btn = dat[2];
        xbox_msgs.A.btn = dat[3];
        // 左右肩键
        xbox_msgs.LB.btn = dat[4];
        xbox_msgs.RB.btn = dat[5];
				// 菜单操作键
        xbox_msgs.Select.btn = dat[6];
        xbox_msgs.Start.btn = dat[7];
        xbox_msgs.Xbox.btn = dat[8];
        xbox_msgs.Share.btn = dat[9];
        // 左右摇杆按钮
        xbox_msgs.LS.btn = dat[10];
        xbox_msgs.RS.btn = dat[11];
        // 十字键
        xbox_msgs.DPadUp.btn = dat[12];
        xbox_msgs.DPadRight.btn = dat[13];
        xbox_msgs.DPadDown.btn = dat[14];
        xbox_msgs.DPadLeft.btn = dat[15];
        // 左右摇杆（霍尔值，进行合成变成float类型）
        xbox_msgs.joyLX = ((uint16_t)dat[16] << 8) | dat[17];
        xbox_msgs.joyLY = ((uint16_t)dat[18] << 8) | dat[19];
        xbox_msgs.joyRX = ((uint16_t)dat[20] << 8) | dat[21];
        xbox_msgs.joyRY = ((uint16_t)dat[22] << 8) | dat[23];
        // 左右扳机键
        xbox_msgs.trigL = ((uint16_t)dat[24] << 8) | dat[25];
        xbox_msgs.trigR = ((uint16_t)dat[26] << 8) | dat[27];
    }
}

// 将摇杆的值映射到-1到1之间
void xbox::joyDataCal(void)
{

    if (deadzone_min < xbox_msgs.joyLX && xbox_msgs.joyLX < deadzone_max)
    {
        joy.normalizedLX = 0;
    }
    else
    {
        joy.normalizedLX = ((float)xbox_msgs.joyLX - 32768.0f) / 32768.0f;
    }

    if (deadzone_min < xbox_msgs.joyLY && xbox_msgs.joyLY < deadzone_max)
    {
        joy.normalizedLY = 0;
    }
    else
    {
        joy.normalizedLY = (32768.0f - (float)xbox_msgs.joyLY) / 32768.0f;
    }

    if (deadzone_min < xbox_msgs.joyRX && xbox_msgs.joyRX < deadzone_max)
    {
        joy.normalizedRX = 0;
    }
    else
    {
        joy.normalizedRX = ((float)xbox_msgs.joyRX - 32768.0f) / 32768.0f;
    }

    if (deadzone_min < xbox_msgs.joyRY && xbox_msgs.joyRY < deadzone_max)
    {
        joy.normalizedRY = 0;
    }
    else
    {
        joy.normalizedRY = (32768.0f - (float)xbox_msgs.joyRY) / 32768.0f;
    }
		//扳机映射
		if (xbox_msgs.trigR < Trigger_deadzone_max)
    {
        joy.RightTrigger = 0;
    }
    else
    {
        joy.RightTrigger = (float)xbox_msgs.trigR / 1023.0f;
    }
		
    if (xbox_msgs.trigL < Trigger_deadzone_max)
    {
        joy.LeftTrigger = 0;
    }
    else
    {
        joy.LeftTrigger = ((float)xbox_msgs.trigL) / 1023.0f;
    }

}

