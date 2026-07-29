#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "can.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class M3508;

class CanBsp
{
public:
    struct TxHeader
    {
        uint32_t id;
        bool is_extended_id;
        bool is_remote_frame;
        uint8_t dlc;
        bool transmit_global_time;

        TxHeader();
    };

    static CanBsp &instance();

    bool init_default(CAN_HandleTypeDef *hcan);
    void register_m3508(M3508 *instance);
    bool send_raw(const TxHeader &tx_header, uint8_t tx_data[8]);

    void on_rx_fifo0_pending(CAN_HandleTypeDef *hcan);

private:
    CanBsp();

    CAN_HandleTypeDef *hcan_;
    M3508 *instances_[8];
};

#define CAN_BUS (CanBsp::instance())
#endif

#endif
