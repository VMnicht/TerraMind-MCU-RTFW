#include "can_bsp.h"

#include "../Driver/M3508.h"

CanBsp::TxHeader::TxHeader()
    : id(0u),
      is_extended_id(false),
      is_remote_frame(false),
      dlc(8u),
      transmit_global_time(false)
{
}

CanBsp::CanBsp() : hcan_(0), instances_{0}
{
}

CanBsp &CanBsp::instance()
{
    static CanBsp bsp;
    return bsp;
}

bool CanBsp::init_default(CAN_HandleTypeDef *hcan)
{
    if (hcan == 0)
    {
        return false;
    }

    hcan_ = hcan;

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan_, &filter) != HAL_OK)
    {
        return false;
    }
    if (HAL_CAN_Start(hcan_) != HAL_OK)
    {
        return false;
    }
    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return false;
    }
    return true;
}

void CanBsp::register_m3508(M3508 *instance)
{
    if (instance == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < 8u; ++i)
    {
        if (instances_[i] == instance)
        {
            return;
        }
    }

    for (uint8_t i = 0; i < 8u; ++i)
    {
        if (instances_[i] == 0)
        {
            instances_[i] = instance;
            return;
        }
    }
}

bool CanBsp::send_raw(const TxHeader &tx_header, uint8_t tx_data[8])
{
    if (hcan_ == 0)
    {
        return false;
    }
    if (tx_data == 0)
    {
        return false;
    }
    if (tx_header.dlc > 8u)
    {
        return false;
    }

    CAN_TxHeaderTypeDef hal_header = {0};
    if (tx_header.is_extended_id)
    {
        hal_header.StdId = 0u;
        hal_header.ExtId = tx_header.id & 0x1FFFFFFFu;
        hal_header.IDE = CAN_ID_EXT;
    }
    else
    {
        hal_header.StdId = tx_header.id & 0x7FFu;
        hal_header.ExtId = 0u;
        hal_header.IDE = CAN_ID_STD;
    }
    hal_header.RTR = tx_header.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hal_header.DLC = tx_header.dlc;
    hal_header.TransmitGlobalTime = tx_header.transmit_global_time ? ENABLE : DISABLE;

    uint32_t mailbox = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0u)
    {
        return false;
    }
    return HAL_CAN_AddTxMessage(hcan_, &hal_header, tx_data, &mailbox) == HAL_OK;
}

void CanBsp::on_rx_fifo0_pending(CAN_HandleTypeDef *hcan)
{
    if (hcan == 0)
    {
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if (rx_header.IDE != CAN_ID_STD || rx_header.DLC < 7u)
        {
            continue;
        }

        for (uint8_t i = 0; i < 8u; ++i)
        {
            if (instances_[i] != 0 && instances_[i]->update_feedback(rx_header.StdId, rx_data))
            {
                break;
            }
        }
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_BUS.on_rx_fifo0_pending(hcan);
}
