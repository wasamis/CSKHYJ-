#ifndef BUS_SERVO_H
#define BUS_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void BusServo_Init(UART_HandleTypeDef *huart);
void BusServo_SetAngle(uint8_t id, uint16_t pulse_us, uint16_t time_ms);
void BusServo_ReadPosition(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* BUS_SERVO_H */