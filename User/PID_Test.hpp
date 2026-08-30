#ifndef PID_TEST_HPP
#define PID_TEST_HPP

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void PID_Test_Init(void);
void PID_Test_TIM_Callback(TIM_HandleTypeDef *htim);
void PID_Test_UART_Process(void);


#ifdef __cplusplus
}
#endif

#endif
