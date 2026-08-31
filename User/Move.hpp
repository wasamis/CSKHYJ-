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
 *   - non-blocking line trace scheduling
 *   - stop
 *   - USART1 community receive callback forwarding
 *   - TIM6 20ms chassis control callback
 *
 * LineTrace itself is implemented in LineTrace.c.
 * Move_Process() is responsible for executing the non-blocking
 * action requests returned by LineTrace_Process().
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
