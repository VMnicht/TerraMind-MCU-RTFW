#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct
{
    uint32_t period;
    uint32_t compare[4];
    uint32_t counter;
    uint32_t pwm_starts;
    uint32_t encoder_starts;
    uint32_t nonzero_starts;
    int fail_start;
} TIM_HandleTypeDef;
typedef struct { int gState; } UART_HandleTypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;

#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 1u
#define TIM_CHANNEL_3 2u
#define TIM_CHANNEL_4 3u
#define HAL_MAX_DELAY 0xffffffffu
#define HAL_UART_STATE_READY 0
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->period)
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter)
#define __HAL_TIM_SET_COUNTER(t, v) ((t)->counter = (v))
#define __HAL_TIM_SET_COMPARE(t, c, v) ((t)->compare[c] = (v))

#ifdef __cplusplus
extern "C" {
#endif
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t HAL_GetTick(void);
extern uint32_t test_tick;
extern uint8_t test_uart_tx[300];
extern uint16_t test_uart_tx_size;
#ifdef __cplusplus
}
#endif
#endif
