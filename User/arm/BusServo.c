#include "BusServo.h"

#include <stdio.h>
#include <string.h>

/*
 * 当前总线舵机协议：
 *
 *   控制位置：
 *       #002P1500T1000!
 *
 *   读取位置：
 *       #002PRAD!
 *
 * 注意：
 *   协议帧本身以 '!' 结束，不需要额外发送 \\r\\n。
 */

static UART_HandleTypeDef *p_huart = NULL;

void BusServo_Init(UART_HandleTypeDef *huart)
{
    p_huart = huart;
}

void BusServo_SetAngle(uint8_t id,
                       uint16_t pulse_us,
                       uint16_t time_ms)
{
    char buf[32];
    int len;
    HAL_StatusTypeDef status;

    if (p_huart == NULL)
    {
        return;
    }

    /*
     * 协议有效范围。
     * 防止调试时错误参数导致舵机拒绝执行。
     */
    if (id > 254U)
    {
        return;
    }

    if (pulse_us < 500U)
    {
        pulse_us = 500U;
    }
    else if (pulse_us > 2500U)
    {
        pulse_us = 2500U;
    }

    if (time_ms > 9999U)
    {
        time_ms = 9999U;
    }

    len = snprintf(buf,
                   sizeof(buf),
                   "#%03uP%04uT%04u!",
                   (unsigned int)id,
                   (unsigned int)pulse_us,
                   (unsigned int)time_ms);

    if ((len <= 0) || ((size_t)len >= sizeof(buf)))
    {
        return;
    }

    /*
     * HAL_OK 只表示 MCU 已经把数据送出 UART，
     * 并不代表舵机一定执行。
     * 如果 UART 此刻忙/出错，短暂等待后再重发一次。
     */
    status = HAL_UART_Transmit(p_huart,
                               (uint8_t *)buf,
                               (uint16_t)len,
                               100U);

    if (status != HAL_OK)
    {
        HAL_Delay(5U);

        (void)HAL_UART_Transmit(p_huart,
                                (uint8_t *)buf,
                                (uint16_t)len,
                                100U);
    }
}

void BusServo_ReadPosition(uint8_t id)
{
    char buf[20];
    int len;

    if (p_huart == NULL)
    {
        return;
    }

    if (id > 254U)
    {
        return;
    }

    len = snprintf(buf,
                   sizeof(buf),
                   "#%03uPRAD!",
                   (unsigned int)id);

    if ((len <= 0) || ((size_t)len >= sizeof(buf)))
    {
        return;
    }

    (void)HAL_UART_Transmit(p_huart,
                            (uint8_t *)buf,
                            (uint16_t)len,
                            100U);
}
