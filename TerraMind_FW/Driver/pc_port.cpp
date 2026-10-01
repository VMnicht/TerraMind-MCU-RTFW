#include "pc_port.h"

namespace
{
PcPort *g_pc_port = 0;
}

PcPort::PcPort(UART_HandleTypeDef *huart)
    : SerialDevice(huart), rx_head_(0u), rx_tail_(0u), overflow_count_(0u),
      command_{}, has_control_(false), last_command_ms_(0u), last_seq_(0u),
      result_(PcProtocol::RESULT_OK), tx_busy_(false), tx_seq_(0u)
{
    g_pc_port = this;
}

void PcPort::handleReceiveData(uint8_t byte)
{
    const uint16_t next = static_cast<uint16_t>((rx_head_ + 1u) % RX_SIZE);
    if (next == rx_tail_)
    {
        ++overflow_count_;
        return;
    }
    rx_[rx_head_] = byte;
    rx_head_ = next;
}

void PcPort::poll(uint32_t now_ms)
{
    while (rx_tail_ != rx_head_)
    {
        const uint8_t byte = rx_[rx_tail_];
        rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1u) % RX_SIZE);
        PcProtocol::Command candidate = {};
        uint16_t seq = 0u;
        PcProtocol::Result result = PcProtocol::RESULT_OK;
        if (parser_.feed(byte, candidate, seq, result))
        {
            last_seq_ = seq;
            result_ = result;
            if (result == PcProtocol::RESULT_OK)
            {
                command_ = candidate;
                has_control_ = true;
                last_command_ms_ = now_ms;
            }
        }
    }
}

bool PcPort::has_control() const { return has_control_; }
bool PcPort::command_fresh(uint32_t now_ms) const
{
    return has_control_ && (now_ms - last_command_ms_) <= PcProtocol::COMMAND_TIMEOUT_MS;
}
const PcProtocol::Command &PcPort::command() const { return command_; }
uint16_t PcPort::last_command_seq() const { return last_seq_; }
PcProtocol::Result PcPort::result() const { return result_; }
uint16_t PcPort::error_count() const
{
    return static_cast<uint16_t>(parser_.error_count() + overflow_count_);
}
uint32_t PcPort::command_age_ms(uint32_t now_ms) const
{
    return has_control_ ? now_ms - last_command_ms_ : 0xFFFFFFFFu;
}
bool PcPort::rx_overflow() const { return overflow_count_ != 0u; }

bool PcPort::send_status(const PcProtocol::Status &status)
{
    if (tx_busy_ || !init_status) return false;
    const uint16_t len = PcProtocol::encode_status(status, tx_seq_, tx_, sizeof(tx_));
    if (len == 0u) return false;
    tx_busy_ = true;
    if (HAL_UART_Transmit_IT(huart_, tx_, len) != HAL_OK)
    {
        tx_busy_ = false;
        return false;
    }
    ++tx_seq_;
    return true;
}

void PcPort::on_tx_complete() { tx_busy_ = false; }

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_pc_port != 0 && huart == g_pc_port->huart_)
        g_pc_port->on_tx_complete();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (g_pc_port != 0 && huart == g_pc_port->huart_ &&
        huart->gState == HAL_UART_STATE_READY)
        g_pc_port->on_tx_complete();
    for (int i = 0; i < SerialDevice::instanceCount_; ++i)
    {
        SerialDevice *device = SerialDevice::instances_[i];
        if (device != 0 && device->huart_ == huart)
        {
            device->startUartReceiveIT();
            break;
        }
    }
}
