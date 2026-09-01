/**
 * @file Motor.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief Hall encoder DC motor control
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright Copyright (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "Motor.hpp"

/* Private macros ------------------------------------------------------------*/

/*
 * One call to Calculate_TIM_PeriodElapsedCallback() corresponds to 20ms.
 * TIM6 is configured directly to generate one interrupt every 20ms.
 */
#define MOTOR_OMEGA_TARGET_DEADBAND        0.05f

/*
 * 位置环需要继续运动时的最小驱动占空比。
 * 用于克服低速末段的电机静摩擦死区。
 */
#define MOTOR_ANGLE_MIN_PWM_PERCENT         30

/*
 * Maximum PWM command change in one 20ms control update.
 */
#define MOTOR_PWM_MAX_STEP_PER_CONTROL     80

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

/**
 * @brief 求绝对值
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type Math_Abs(Type x)
{
    return((x > 0) ? x : -x);
}

/**
 * @brief 初始化电机
 *
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 * @param __Driver_PWM_TIM_Channel_x 电机驱动定时器通道
 * @param __Output_A_GPIO_Pin 电机方向A相引脚号
 * @param __Output_A_GPIOx 电机方向A相引脚组
 * @param __Output_B_GPIO_Pin 电机方向B相引脚号
 * @param __Output_B_GPIOx 电机方向B相引脚组
 */
void Class_Motor::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x, uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
    Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
    Output_A_GPIOx = __Output_A_GPIOx;
    Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
    Output_B_GPIOx = __Output_B_GPIOx;

    //输出PWM
    HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

/**
 * @brief 设定电机正向旋转方向
 *
 * @param __Rotate_Direction_Flag 电机正向旋转方向
 */
void Class_Motor::Set_Rotate_Direction_Flag(Enum_Rotate_Direction __Rotate_Direction_Flag)
{
    Rotate_Direction_Flag = __Rotate_Direction_Flag;
}

/**
 * @brief 设定电机PWM满占空比对应的数值
 *
 * @param __Motor_Full_Omega 电机PWM满占空比对应的数值
 */
void Class_Motor::Set_Motor_Full_Omega(float __Motor_Full_Omega)
{
    Motor_Full_Omega = __Motor_Full_Omega;
}

/**
 * @brief 设定电机PWM满占空比对应的数值
 *
 * @param __Motor_PWM_Period 电机PWM满占空比对应的数值
 */
void Class_Motor::Set_Motor_PWM_Period(int32_t __Motor_PWM_Period)
{
    Motor_PWM_Period = __Motor_PWM_Period;
}

/**
 * @brief 设定电机目标输出强度, 即电机PWM占空比的分子
 *
 * @param __Out 电机目标输出强度, 即电机PWM占空比的分子
 */
void Class_Motor::Set_Out(int32_t __Out)
{
    Out = __Out;
}

/**
 * @brief 获取电机正向旋转方向
 *
 * @return Enum_Rotate_Direction 电机正向旋转方向
 */
Enum_Rotate_Direction Class_Motor::Get_Rotate_Direction_Flag()
{
    return(Rotate_Direction_Flag);
}

/**
 * @brief 设定电机减速后满转转速, rad/s
 *
 * @return float 电机减速后满转转速, rad/s
 */
float Class_Motor::Get_Motor_Full_Omega()
{
    return(Motor_Full_Omega);
}

/**
 * @brief 设定电机PWM满占空比对应的数值
 *
 * @return int32_t 电机PWM满占空比对应的数值
 */
int32_t Class_Motor::Get_Motor_PWM_Period()
{
    return(Motor_PWM_Period);
}

/**
 * @brief 获取电机目标输出强度, 即电机PWM占空比的分子
 *
 * @return int32_t 电机目标输出强度, 即电机PWM占空比的分子
 */
int32_t Class_Motor::Get_Out()
{
    return(Out);
}

/**
 * @brief 设定电机占空比, 确定输出
 */
void Class_Motor::Output()
{
    if (Out == 0)
    {
        HAL_GPIO_WritePin(
            Output_A_GPIOx,
            Output_A_GPIO_Pin,
            GPIO_PIN_RESET);

        HAL_GPIO_WritePin(
            Output_B_GPIOx,
            Output_B_GPIO_Pin,
            GPIO_PIN_RESET);
    }
    else if (Out > 0)
    {
        HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, GPIO_PIN_SET);
    }

    __HAL_TIM_SetCompare(
        &Driver_PWM_TIM,
        Driver_PWM_TIM_Channel_x,
        Math_Abs(Out));
}

/**
 * @brief Initialize motor + hardware Encoder Timer
 */
void Class_Motor_With_Hall_Encoder::Init(
    TIM_HandleTypeDef __Driver_PWM_TIM,
    uint8_t __Driver_PWM_TIM_Channel_x,
    uint16_t __Output_A_GPIO_Pin,
    GPIO_TypeDef *__Output_A_GPIOx,
    uint16_t __Output_B_GPIO_Pin,
    GPIO_TypeDef *__Output_B_GPIOx,
    TIM_HandleTypeDef *__Encoder_TIM)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;

    Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
    Output_A_GPIOx = __Output_A_GPIOx;

    Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
    Output_B_GPIOx = __Output_B_GPIOx;

    Encoder_TIM = __Encoder_TIM;

    Omega_Now = 0.0f;

    HAL_TIM_PWM_Start(
        &Driver_PWM_TIM,
        Driver_PWM_TIM_Channel_x);

    /*
     * Start hardware quadrature decoding.
     */
    if (Encoder_TIM != 0)
    {
        __HAL_TIM_SET_COUNTER(Encoder_TIM, 0U);

        HAL_TIM_Encoder_Start(
            Encoder_TIM,
            TIM_CHANNEL_ALL);

        Encoder_Count_Previous =
            __HAL_TIM_GET_COUNTER(Encoder_TIM);
    }
    else
    {
        Encoder_Count_Previous = 0U;
    }
}

void Class_Motor_With_Hall_Encoder::Set_Control_Method(
    Enum_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

void Class_Motor_With_Hall_Encoder::Set_Omega_Target(
    float __Omega_Target)
{
    Omega_Target = __Omega_Target;
}

void Class_Motor_With_Hall_Encoder::Set_Motor_Encoder_Num_Per_Rad(
    float __Motor_Encoder_Num_Per_Rad)
{
    Motor_Encoder_Num_Per_Rad =
        __Motor_Encoder_Num_Per_Rad;
}

void Class_Motor_With_Hall_Encoder::Set_Angle_Target(
    float __Angle_Target)
{
    Angle_Target = __Angle_Target;
}

Enum_Control_Method Class_Motor_With_Hall_Encoder::Get_Control_Method()
{
    return Control_Method;
}

float Class_Motor_With_Hall_Encoder::Get_Omega_Now()
{
    return Omega_Now;
}

float Class_Motor_With_Hall_Encoder::Get_Angle_Now()
{
    return Angle_Now;
}

float Class_Motor_With_Hall_Encoder::Get_Angle_Target()
{
    return Angle_Target;
}


void Class_Motor_With_Hall_Encoder::Calculate_TIM_PeriodElapsedCallback()
{
    /*
     * ============================================================
     * Step 1: read Encoder Timer CNT once per 20ms
     * ============================================================
     */
    int32_t encoder_delta = 0;

    if (Encoder_TIM != 0)
    {
        const uint32_t encoder_count_now =
            __HAL_TIM_GET_COUNTER(Encoder_TIM);

        const uint32_t encoder_arr =
            __HAL_TIM_GET_AUTORELOAD(Encoder_TIM);

        /*
         * TIM2 is 32-bit.
         * TIM3/TIM4/TIM8 are 16-bit.
         * Native-width subtraction handles counter wrap-around.
         */
        if (encoder_arr <= 0xFFFFU)
        {
            encoder_delta = (int32_t)(int16_t)(
                (uint16_t)encoder_count_now -
                (uint16_t)Encoder_Count_Previous);
        }
        else
        {
            encoder_delta = (int32_t)(
                encoder_count_now -
                Encoder_Count_Previous);
        }

        Encoder_Count_Previous = encoder_count_now;
    }

    /*
     * Software sign correction retained from the original project.
     * If a wheel reports the opposite sign in the first test,
     * only flip that wheel's Rotate_Direction_Flag in Chassis.cpp.
     */
    if (Rotate_Direction_Flag == CW)
    {
        encoder_delta = -encoder_delta;
    }

    /*
     * ============================================================
     * Step 2: update angle and 20ms wheel speed
     * ============================================================
     */
    if (Motor_Encoder_Num_Per_Rad > 0.0f)
    {
        const float angle_delta =
            (float)encoder_delta /
            Motor_Encoder_Num_Per_Rad;

        Angle_Now += angle_delta;

        Omega_Now =
            angle_delta /
            MOTOR_CALCULATE_PERIOD;
    }
    else
    {
        Omega_Now = 0.0f;
    }

    /*
     * ============================================================
     * Step 3: execute PID and update PWM once per 20ms
     * ============================================================
     */
    int32_t desired_out = 0;

    if (Control_Method == Control_Method_OPENLOOP)
    {
        desired_out = (int32_t)(
            Omega_Target *
            (float)Motor_PWM_Period /
            Motor_Full_Omega);
    }
    else if (Control_Method == Control_Method_OMEGA)
    {
        Omega_PID.Set_Now(Omega_Now);
        Omega_PID.Set_Target(Omega_Target);
        Omega_PID.Adjust_TIM_PeriodElapsedCallback();

        float pid_out = Omega_PID.Get_Out();

        if (Omega_Target > MOTOR_OMEGA_TARGET_DEADBAND)
        {
            if (pid_out < 0.0f)
            {
                pid_out = 0.0f;
            }
        }
        else if (Omega_Target < -MOTOR_OMEGA_TARGET_DEADBAND)
        {
            if (pid_out > 0.0f)
            {
                pid_out = 0.0f;
            }
        }
        else
        {
            pid_out = 0.0f;
        }

        desired_out = (int32_t)pid_out;
    }
    else if (Control_Method == Control_Method_ANGLE)
    {
        /*
         * Outer loop: angle -> target wheel speed
         */
        Angle_PID.Set_Target(Angle_Target);
        Angle_PID.Set_Now(Angle_Now);
        Angle_PID.Adjust_TIM_PeriodElapsedCallback();

        const float omega_target_from_angle =
            Angle_PID.Get_Out();

        /*
         * Inner loop: wheel speed -> PWM
         */
        Omega_PID.Set_Target(omega_target_from_angle);
        Omega_PID.Set_Now(Omega_Now);
        Omega_PID.Adjust_TIM_PeriodElapsedCallback();

        float pid_out = Omega_PID.Get_Out();

        if (omega_target_from_angle >
            MOTOR_OMEGA_TARGET_DEADBAND)
        {
            if (pid_out < 0.0f)
            {
                pid_out = 0.0f;
            }
        }
        else if (omega_target_from_angle <
                 -MOTOR_OMEGA_TARGET_DEADBAND)
        {
            if (pid_out > 0.0f)
            {
                pid_out = 0.0f;
            }
        }
        else
        {
            pid_out = 0.0f;
        }

        const int32_t min_angle_out =
            (Motor_PWM_Period *
             MOTOR_ANGLE_MIN_PWM_PERCENT + 99) /
            100;

        /*
         * 只对位置环仍然要求运动的非零输出补偿死区。
         * pid_out == 0 时保持为零，使超调减速和最终停车仍然有效。
         */
        if ((pid_out > 0.0f) &&
            (pid_out < (float)min_angle_out))
        {
            pid_out = (float)min_angle_out;
        }
        else if ((pid_out < 0.0f) &&
                 (pid_out > (float)-min_angle_out))
        {
            pid_out = (float)-min_angle_out;
        }

        desired_out = (int32_t)pid_out;
    }

    Math_Constrain(
        &desired_out,
        -Motor_PWM_Period,
        Motor_PWM_Period);

    /*
     * Closed-loop PWM slew-rate limit.
     * One step corresponds to one 20ms control update.
     */
    if (Control_Method != Control_Method_OPENLOOP)
    {
        const int32_t delta_out =
            desired_out - Out;

        if (delta_out > MOTOR_PWM_MAX_STEP_PER_CONTROL)
        {
            desired_out =
                Out + MOTOR_PWM_MAX_STEP_PER_CONTROL;
        }
        else if (delta_out <
                 -MOTOR_PWM_MAX_STEP_PER_CONTROL)
        {
            desired_out =
                Out - MOTOR_PWM_MAX_STEP_PER_CONTROL;
        }
    }

    Math_Constrain(
        &desired_out,
        -Motor_PWM_Period,
        Motor_PWM_Period);

    Out = desired_out;
    Output();
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
