#ifndef __MOVE_HPP
#define __MOVE_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * Move module
 *
 * Responsibilities:
 *   - chassis translation
 *   - chassis in-place rotation
 *   - stop
 *   - USART1 community receive callback forwarding
 *   - TIM6 20ms chassis control callback
 *
 * It does NOT handle:
 *   - line tracing
 *   - arm/grab
 *   - building/lead-screw
 *   - servo actions
 */
void Move_Init(void);
void Move_Process(void);

void Move_TIM_Callback(TIM_HandleTypeDef *htim);

void Move_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Move_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
