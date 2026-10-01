#ifndef PC_PORT_H
#define PC_PORT_H

#include "../BSP/Serial_device.h"
#include "pc_protocol.h"

class PcPort : public SerialDevice
{
public:
    explicit PcPort(UART_HandleTypeDef *huart);
    void handleReceiveData(uint8_t byte) override;
    void poll(uint32_t now_ms);
    bool has_control() const;
    bool command_fresh(uint32_t now_ms) const;
    const PcProtocol::Command &command() const;
    uint16_t last_command_seq() const;
    PcProtocol::Result result() const;
    uint16_t error_count() const;
    uint32_t command_age_ms(uint32_t now_ms) const;
    bool rx_overflow() const;
    bool send_status(const PcProtocol::Status &status);
    void on_tx_complete();

private:
    static const uint16_t RX_SIZE = 256u;
    volatile uint16_t rx_head_;
    volatile uint16_t rx_tail_;
    uint8_t rx_[RX_SIZE];
    volatile uint16_t overflow_count_;
    PcProtocol::StreamParser parser_;
    PcProtocol::Command command_;
    bool has_control_;
    uint32_t last_command_ms_;
    uint16_t last_seq_;
    PcProtocol::Result result_;
    volatile bool tx_busy_;
    uint16_t tx_seq_;
    uint8_t tx_[PcProtocol::MAX_FRAME];
};

#endif
