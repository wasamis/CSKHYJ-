#include "PID_Test.hpp"

#include "tim.h"
#include "usart.h"
#include "Chassis.hpp"

#include <stdio.h>

/*
 * Four-wheel constant-speed open-loop / closed-loop test.
 *
 * Timing:
 *     TIM6 interrupt      : 20ms
 *     speed measurement   : 20ms
 *     PID update          : 20ms
 *     PWM update          : 20ms
 *     UART snapshot       : 20ms
 *
 * Serial output:
 *     target,omega0,omega1,omega2,omega3
 *
 * USART1 is expected to be configured to 115200 baud in CubeMX.
 */

/* Test parameters -----------------------------------------------------------*/

/*
 * Test mode:
 *
 *     Control_Method_OMEGA
 *         Closed-loop speed PID.
 *         Encoder speed feedback participates in PWM calculation.
 *
 *     Control_Method_OPENLOOP
 *         Open-loop speed command.
 *         Encoder is still measured and printed, but measured speed does not
 *         participate in PWM calculation.
 *
 * To switch to open-loop, change only this one line:
 *     Control_Method_OMEGA -> Control_Method_OPENLOOP
 */
static const Enum_Control_Method PID_Test_Control_Method =
    Control_Method_OMEGA;

/*
 * Commanded wheel speed, rad/s.
 *
 * Closed-loop:
 *     true speed target for PID.
 *
 * Open-loop:
 *     converted to PWM by Motor.cpp using Motor_Full_Omega;
 *     encoder feedback is measured only and does not alter the PWM.
 */
static const float PID_Test_Target = 6.0f;

static const float PID_Test_Kp = 55.0f;
static const float PID_Test_Ki = 35.0f;
static const float PID_Test_Kd = 0.0f;

/* TIM1 ARR = 999: integral output limit ~= 30%, total PID/PWM limit ~= 50% */
static const float PID_Test_I_Out_Max = 300.0f;
static const float PID_Test_PWM_Out_Max = 500.0f;

static const uint32_t PID_Test_UART_Timeout_Ms = 20U;

/* Private variables ---------------------------------------------------------*/

static Class_Chassis PID_Test_Chassis;


static volatile uint8_t pid_test_uart_data_ready = 0U;

static volatile float pid_test_omega_snapshot[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

/* Public functions ----------------------------------------------------------*/

extern "C" void PID_Test_Init(void)
{
    int i;

    /*
     * Chassis.Init() starts:
     *     TIM1 PWM channels
     *     TIM2/TIM3/TIM4/TIM8 Encoder Mode
     */
    PID_Test_Chassis.Init(
        htim1);

    for (i = 0; i < 4; i++)
    {
        PID_Test_Chassis.Motor[i].Set_Motor_PWM_Period(
            (int32_t)PID_Test_PWM_Out_Max);

        PID_Test_Chassis.Motor[i].Omega_PID.Init(
            PID_Test_Kp,
            PID_Test_Ki,
            PID_Test_Kd,
            PID_Test_I_Out_Max,
            PID_Test_PWM_Out_Max);

        PID_Test_Chassis.Motor[i].Omega_PID.Set_history_IE(
            0.0f);

        PID_Test_Chassis.Motor[i].Set_Control_Method(
            PID_Test_Control_Method);

        PID_Test_Chassis.Motor[i].Set_Omega_Target(
            PID_Test_Target);
    }

    pid_test_uart_data_ready = 0U;

    /*
     * TIM6 is configured to generate one interrupt every 20ms.
     */
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void PID_Test_TIM_Callback(TIM_HandleTypeDef *htim)
{
    int i;

    if (htim == 0)
    {
        return;
    }

    if (htim->Instance != TIM6)
    {
        return;
    }


    /*
     * Do not call Chassis.Calculate_TIM_PeriodElapsedCallback() here,
     * because that path derives each wheel target from chassis Velocity.
     *
     * This dedicated test keeps all four wheel commands fixed at +6 rad/s.
     * The selected test mode decides whether encoder feedback closes the loop.
     */
    for (i = 0; i < 4; i++)
    {
        PID_Test_Chassis.Motor[i].Set_Control_Method(
            PID_Test_Control_Method);

        PID_Test_Chassis.Motor[i].Set_Omega_Target(
            PID_Test_Target);

        /*
         * One call represents exactly one 20ms cycle:
         * Encoder CNT delta -> omega -> PID -> PWM.
         */
        PID_Test_Chassis.Motor[i].
            Calculate_TIM_PeriodElapsedCallback();

        pid_test_omega_snapshot[i] =
            PID_Test_Chassis.Motor[i].Get_Omega_Now();
    }

    /*
     * UART is sent from the main loop, not from the TIM6 ISR.
     */
    pid_test_uart_data_ready = 1U;
}

extern "C" void PID_Test_UART_Process(void)
{
    int i;
    float omega[4];
    uint32_t primask;
    char tx_buffer[96];
    int length;

    if (pid_test_uart_data_ready == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < 4; i++)
    {
        omega[i] = pid_test_omega_snapshot[i];
    }

    pid_test_uart_data_ready = 0U;

    __set_PRIMASK(primask);

    /*
     * Output:
     * target,omega0,omega1,omega2,omega3
     */
    length = snprintf(
        tx_buffer,
        sizeof(tx_buffer),
        "%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
        (double)PID_Test_Target,
        (double)omega[0],
        (double)omega[1],
        (double)omega[2],
        (double)omega[3]);

    if (length <= 0)
    {
        return;
    }

    if ((uint32_t)length >= sizeof(tx_buffer))
    {
        return;
    }

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)tx_buffer,
        (uint16_t)length,
        PID_Test_UART_Timeout_Ms);
}

