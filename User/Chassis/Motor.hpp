/**
 * @file Motor.hpp
 * @author yssickjgd 1345578933@qq.com
 * @brief Hall encoder DC motor control
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "PID.hpp"

/* Exported macros -----------------------------------------------------------*/

// Pi
const float PI = 3.14159f;

// Motor output-shaft no-load speed, rad/s
const float MOTOR_FULL_OMEGA =
    (260.0f / 60.0f * 2.0f * PI);

/*
 * Encoder Mode TI1 + TI2 performs quadrature decoding on both A/B channels.
 *
 * Current encoder parameter:
 *     13 PPR at motor shaft
 *     x4 quadrature decoding
 *     x27 gearbox
 *
 * Therefore:
 *     13 * 4 * 27 = 1404 count / output-shaft revolution
 */
const float MOTOR_ENCODER_NUM_PER_RAD =
    (13.0f * 4.0f * 27.0f / 2.0f / PI);

// TIM1 PWM ARR
const int32_t MOTOR_CALCULATE_PRESCALER = 999;

/*
 * Actual speed measurement / PID / PWM update period.
 *
 * TIM6 is configured directly for a 20ms interrupt.
 * One motor calculation therefore corresponds to one 20ms control period.
 */
const float MOTOR_CALCULATE_PERIOD = 0.02f;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Motor control method
 */
enum Enum_Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_OMEGA,
    Control_Method_ANGLE
};

/**
 * @brief Encoder sign correction flag
 */
enum Enum_Rotate_Direction
{
    CW = 0,
    CCW
};

class Class_Motor
{
    public:

        void Init(TIM_HandleTypeDef __Driver_PWM_TIM,
                  uint8_t __Driver_PWM_TIM_Channel_x,
                  uint16_t __Output_A_GPIO_Pin,
                  GPIO_TypeDef *__Output_A_GPIOx,
                  uint16_t __Output_B_GPIO_Pin,
                  GPIO_TypeDef *__Output_B_GPIOx);

        void Set_Rotate_Direction_Flag(
            Enum_Rotate_Direction __Rotate_Direction_Flag);

        void Set_Motor_Full_Omega(float __Motor_Full_Omega);
        void Set_Motor_PWM_Period(int32_t __Motor_PWM_Period);
        void Set_Out(int32_t __Out);

        Enum_Rotate_Direction Get_Rotate_Direction_Flag();
        float Get_Motor_Full_Omega();
        int32_t Get_Motor_PWM_Period();
        int32_t Get_Out();

        void Output();

    protected:

        // Motor PWM timer
        TIM_HandleTypeDef Driver_PWM_TIM;
        uint8_t Driver_PWM_TIM_Channel_x;

        // H-bridge direction GPIO
        uint16_t Output_A_GPIO_Pin;
        GPIO_TypeDef *Output_A_GPIOx;

        uint16_t Output_B_GPIO_Pin;
        GPIO_TypeDef *Output_B_GPIOx;

        /*
         * Software encoder-sign correction.
         *
         * Encoder Mode itself determines the raw direction from A/B phase.
         * This flag is retained so the four wheels can be unified to the
         * convention "physical forward = positive omega".
         */
        Enum_Rotate_Direction Rotate_Direction_Flag = CW;

        // Motor no-load speed, rad/s
        float Motor_Full_Omega = MOTOR_FULL_OMEGA;

        // PWM full-scale ARR value
        int32_t Motor_PWM_Period = MOTOR_CALCULATE_PRESCALER;

        // Current PWM command
        int32_t Out = 0;
};

class Class_Motor_With_Hall_Encoder : public Class_Motor
{
    public:

        // Speed-loop PID
        Class_PID Omega_PID;

        // Angle-loop PID
        Class_PID Angle_PID;

        void Init(TIM_HandleTypeDef __Driver_PWM_TIM,
                  uint8_t __Driver_PWM_TIM_Channel_x,
                  uint16_t __Output_A_GPIO_Pin,
                  GPIO_TypeDef *__Output_A_GPIOx,
                  uint16_t __Output_B_GPIO_Pin,
                  GPIO_TypeDef *__Output_B_GPIOx,
                  TIM_HandleTypeDef *__Encoder_TIM);

        void Set_Control_Method(Enum_Control_Method __Control_Method);
        void Set_Motor_Encoder_Num_Per_Rad(
            float __Motor_Encoder_Num_Per_Rad);
        void Set_Omega_Target(float __Omega_Target);
        void Set_Angle_Target(float __Angle_Target);

        Enum_Control_Method Get_Control_Method();
        float Get_Omega_Now();
        float Get_Angle_Now();
        float Get_Angle_Target();


        /*
         * One call = one complete 20ms control update:
         *
         * Encoder CNT difference
         * -> wheel speed
         * -> PID
         * -> PWM output
         */
        void Calculate_TIM_PeriodElapsedCallback();

    protected:

        // Hardware timer configured in Encoder Mode
        TIM_HandleTypeDef *Encoder_TIM = 0;

        // CNT sampled at the previous 20ms measurement
        uint32_t Encoder_Count_Previous = 0U;


        // Motor control mode
        Enum_Control_Method Control_Method = Control_Method_OMEGA;

        // Encoder counts per output-shaft radian
        float Motor_Encoder_Num_Per_Rad = MOTOR_ENCODER_NUM_PER_RAD;

        // Current wheel speed, rad/s
        float Omega_Now = 0.0f;

        // Target wheel speed, rad/s
        float Omega_Target = 0.0f;

        // Current wheel angle, rad
        float Angle_Now = 0.0f;

        // Target wheel angle, rad
        float Angle_Target = 0.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations -------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
