#include "tim.h"
#include "usart.h"
#include "can.h"
#include <string.h>
#include <assert.h>

TIM_HandleTypeDef htim1 = {}, htim2 = {}, htim3 = {}, htim4 = {}, htim5 = {}, htim8 = {};
TIM_HandleTypeDef htim9 = {}, htim10 = {}, htim11 = {}, htim12 = {}, htim13 = {}, htim14 = {};
UART_HandleTypeDef huart3 = {}, huart5 = {}, huart6 = {};
CAN_HandleTypeDef hcan1 = {};
uint32_t test_tick = 0;
uint8_t test_uart_tx[300] = {};
uint16_t test_uart_tx_size = 0;

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *tim, uint32_t ch)
{
    ++tim->pwm_starts;
    if (tim->compare[ch] != 0u) ++tim->nonzero_starts;
    return tim->fail_start ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *tim, uint32_t)
{
    ++tim->encoder_starts;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t)
{
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *data, uint16_t len)
{
    assert(len <= sizeof(test_uart_tx));
    memcpy(test_uart_tx, data, len);
    test_uart_tx_size = len;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
uint32_t HAL_GetTick(void) { return test_tick; }
