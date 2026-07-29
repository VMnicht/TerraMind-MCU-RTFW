#include "Serial_Device.h"
//初始化静态成员
SerialDevice* SerialDevice::instances_[MAX_INSTANCES]={nullptr}; 
int SerialDevice::instanceCount_=0;    
SerialDevice::SerialDevice(UART_HandleTypeDef *huartx)                   
{
    if(instanceCount_>MAX_INSTANCES)
    {
        init_status = false;
        return;
    }
    for(uint8_t i = 0; i < MAX_INSTANCES; i++)
    {
        if(instances_[i]->huart_ == huartx)
        {
            init_status = false;
            return;
        }
    }
    huart_ = huartx;
    instances_[instanceCount_] = this;
    instanceCount_++;
    init_status = true;
}

bool SerialDevice::SendByte(uint8_t data)
{
    if (huart_ == nullptr) {
        return false;  // 如果 UART 句柄为空，返回失败
    }
    // 使用 HAL 库发送单个字节
    if (HAL_UART_Transmit(huart_, &data, 1, 500) != HAL_OK) {
        return false;  // 发送失败
    }
    return true;  // 发送成功
}

bool SerialDevice::SendString(char *data)
{
    if (huart_ == nullptr || data == nullptr) {
        return false;  // 如果 UART 句柄或数据指针为空，返回失败
    }

    // 获取字符串长度
    size_t length = strlen(data);

    // 使用 HAL 库发送字符串
    if (HAL_UART_Transmit(huart_, (uint8_t *)data, length, HAL_MAX_DELAY) != HAL_OK) {
        return false;  // 发送失败
    }

    return true;  // 发送成功
}

bool SerialDevice::SendArray(uint8_t *data, uint8_t data_len)
{
    if (huart_ == nullptr || data == nullptr) {
        return false;  // 如果 UART 句柄或数据指针为空，返回失败
    }

    // 使用 HAL 库发送数组
    if (HAL_UART_Transmit(huart_, data, data_len, HAL_MAX_DELAY) != HAL_OK) {
        return false;  // 发送失败
    }

    return true;  // 发送成功
}

bool SerialDevice::SendFloat(float data)
{
    if (huart_ == nullptr) {
        return false;  // 如果 UART 句柄为空，返回失败
    }

    // 将浮点数转换为字节数组
    uint8_t buffer[4];
    memcpy(buffer, &data, sizeof(float));

    // 使用 HAL 库发送字节数组
    if (HAL_UART_Transmit(huart_, buffer, sizeof(float), HAL_MAX_DELAY) != HAL_OK) {
        return false;  // 发送失败
    }

    return true;  // 发送成功
}

void SerialDevice::startUartReceiveIT()
{
    HAL_UART_Receive_IT(huart_, rxBuffer_, RX_BUFFER_SIZE);
}

bool SerialDevice::SendInt32(int32_t data)
{
    if (huart_ == nullptr) {
        return false;  // 如果 UART 句柄为空，返回失败
    }

    // 将 int32_t 转换为字节数组
    uint8_t buffer[4];
    memcpy(buffer, &data, sizeof(int32_t));

    // 使用 HAL 库发送字节数组
    if (HAL_UART_Transmit(huart_, buffer, sizeof(int32_t), HAL_MAX_DELAY) != HAL_OK) {
        return false;  // 发送失败
    }

    return true;  // 发送成功
}

bool SerialDevice::SendInt16(int16_t data)
{
    if (huart_ == nullptr) {
        return false;  // 如果 UART 句柄为空，返回失败
    }

    // 将 int16_t 转换为字节数组
    uint8_t buffer[2];
    memcpy(buffer, &data, sizeof(int16_t));

    // 使用 HAL 库发送字节数组
    if (HAL_UART_Transmit(huart_, buffer, sizeof(int16_t), HAL_MAX_DELAY) != HAL_OK) {
        return false;  // 发送失败
    }

    return true;  // 发送成功
}



extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t rxByte  ;
	for(int i = 0;i<SerialDevice::instanceCount_;i++)
	{
		if(SerialDevice::instances_[i]->huart_ == huart)
		{	 
			 rxByte = SerialDevice::instances_[i]->rxBuffer_[0]; 
       SerialDevice::instances_[i]->handleReceiveData(rxByte);
       HAL_UART_Receive_IT(SerialDevice::instances_[i]->huart_, 
                           SerialDevice::instances_[i]->rxBuffer_,
													 RX_BUFFER_SIZE);
		}
	}
}

//该函数为虚函数，可以在子类中重新定义实现流程，也可以不实现（根据需求来）
void SerialDevice::handleReceiveData(uint8_t byte)
{
	;
}


