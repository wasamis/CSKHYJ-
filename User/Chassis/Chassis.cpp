/**
 * @file Chassis.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 底盘控制
 * @version 0.1
 * @date 2022-05-04
 *
 * @copyright Copyright (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "Chassis.hpp"
#include "tim.h"

/* Private macros ------------------------------------------------------------*/

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
    return ((x > 0) ? x : -x);
}

/**
 * @brief 初始化底盘
 *
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 */
void Class_Chassis::Init(TIM_HandleTypeDef __Driver_PWM_TIM)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    /*
     * Motor / Encoder Timer mapping:
     *
     * Motor[0] -> TIM2 Encoder
     * Motor[1] -> TIM3 Encoder
     * Motor[2] -> TIM4 Encoder
     * Motor[3] -> TIM8 Encoder
     *
     * TIM1 CH1..CH4 remains the four motor PWM outputs.
     * TIM6 provides the 20ms motor control interrupt.
     */
    Motor[0].Init(__Driver_PWM_TIM,
                  TIM_CHANNEL_1,
                  Pin_Pushpull_MotorDirectionA1_Pin,
                  Pin_Pushpull_MotorDirectionA1_GPIO_Port,
                  Pin_Pushpull_MotorDirectionB1_Pin,
                  Pin_Pushpull_MotorDirectionB1_GPIO_Port,
                  &htim2);
    Motor[0].Set_Rotate_Direction_Flag(CW);

    Motor[1].Init(__Driver_PWM_TIM,
                  TIM_CHANNEL_2,
                  Pin_Pushpull_MotorDirectionA2_Pin,
                  Pin_Pushpull_MotorDirectionA2_GPIO_Port,
                  Pin_Pushpull_MotorDirectionB2_Pin,
                  Pin_Pushpull_MotorDirectionB2_GPIO_Port,
                  &htim3);
    Motor[1].Set_Rotate_Direction_Flag(CCW);

    Motor[2].Init(__Driver_PWM_TIM,
                  TIM_CHANNEL_3,
                  Pin_Pushpull_MotorDirectionA3_Pin,
                  Pin_Pushpull_MotorDirectionA3_GPIO_Port,
                  Pin_Pushpull_MotorDirectionB3_Pin,
                  Pin_Pushpull_MotorDirectionB3_GPIO_Port,
                  &htim4);
    Motor[2].Set_Rotate_Direction_Flag(CCW);

    Motor[3].Init(__Driver_PWM_TIM,
                  TIM_CHANNEL_4,
                  Pin_Pushpull_MotorDirectionA4_Pin,
                  Pin_Pushpull_MotorDirectionA4_GPIO_Port,
                  Pin_Pushpull_MotorDirectionB4_Pin,
                  Pin_Pushpull_MotorDirectionB4_GPIO_Port,
                  &htim8);
    Motor[3].Set_Rotate_Direction_Flag(CCW);

    /*
     * 电机 PID 初始化
     *
     * 注意：
     * 你的 Move.cpp 里面如果又重新 Init 了 PID，
     * 最终参数会以 Move.cpp 后设置的为准。
     */
    for (int i = 0; i < 4; i++)
    {
        /*
         * Speed-loop PID:
         *     Kp = 55
         *     Ki = 35
         *     Kd = 0
         *     integral-output limit = 300  (~30% of TIM1 ARR=999)
         *     total-output limit    = 500  (~50% of TIM1 ARR=999)
         */
        Motor[i].Omega_PID.Init(55.0f,
                                35.0f,
                                0.0f,
                                400.0f,
                                500.0f);

        /*
         * 最后一个参数限制了角度环输出，也就是麦轮稳定速度。
         */
        Motor[i].Angle_PID.Init(3.0f,
                                0.0f,
                                0.0f,
                                (float)ULONG_MAX,
                                CHASSIS_MEDIUM_SPEED_UPPERBOUND);
    }
}

/**
 * @brief 设定底盘速度
 *
 * @param __Velocity 底盘速度
 */
void Class_Chassis::Set_Velocity(SpeedTypeDef __Velocity)
{
    Velocity = __Velocity;
}

/**
 * @brief 设定底盘控制方式
 *
 * @param __Control_Method 底盘控制方式
 */
void Class_Chassis::Set_Control_Method(Enum_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;

    if (__Control_Method != Control_Method_ANGLE)
    {
        Angle_Sync_Valid = 0U;
    }

    for (int i = 0; i < 4; i++)
    {
        Motor[i].Set_Control_Method(__Control_Method);
    }
}


/**
 * @brief 定时器中断处理函数
 *
 */
void Class_Chassis::Calculate_TIM_PeriodElapsedCallback()
{
    /*
     * TIM6 is configured directly to 20ms.
     * Every callback performs one complete speed/PID/PWM update.
     */

    /*
     * ============================================================
     * ANGLE mode: four-wheel normalized-progress synchronization
     * ============================================================
     *
     * The original design lets each wheel independently run:
     *     angle PID -> omega target -> speed PID -> PWM
     *
     * That can make the chassis weave even if the final displacement
     * is correct, because the four instantaneous wheel speeds do not
     * stay synchronized.
     *
     * Here we keep every single-wheel PID unchanged.  We only add a
     * small temporary angle-target bias based on normalized progress:
     *
     *     progress_i = (angle_now_i - start_i) / add_i
     *
     * A wheel ahead of the group gets a slightly smaller effective
     * target; a wheel behind gets a slightly larger effective target.
     *
     * The real final Angle_Target is restored immediately after the
     * motor calculations, so Move.cpp completion logic still sees the
     * original commanded target.
     */
#if CHASSIS_ANGLE_SYNC_ENABLE
    if ((Control_Method == Control_Method_ANGLE) &&
        (Angle_Sync_Valid != 0U))
    {
        float progress[4] =
        {
            0.0f, 0.0f, 0.0f, 0.0f
        };

        float progress_sum = 0.0f;
        int active_count = 0;

        for (int i = 0; i < 4; i++)
        {
            const float add_abs = Math_Abs(Angle_Sync_Add[i]);

            if (add_abs >= CHASSIS_ANGLE_SYNC_MIN_ADD_RAD)
            {
                progress[i] =
                    (Motor[i].Get_Angle_Now() - Angle_Sync_Start[i]) /
                    Angle_Sync_Add[i];

                /*
                 * Reject unrealistic one-sample excursions from
                 * encoder noise.  Normal motion is around 0..1.
                 */
                Math_Constrain(&progress[i], -0.25f, 1.25f);

                progress_sum += progress[i];
                active_count++;
            }
        }

        if (active_count > 1)
        {
            const float mean_progress =
                progress_sum / (float)active_count;

            for (int i = 0; i < 4; i++)
            {
                const float add_abs = Math_Abs(Angle_Sync_Add[i]);

                if (add_abs >= CHASSIS_ANGLE_SYNC_MIN_ADD_RAD)
                {
                    /*
                     * Positive sync_error means this wheel is behind.
                     */
                    const float sync_error =
                        mean_progress - progress[i];

                    float bias =
                        CHASSIS_ANGLE_SYNC_K_RAD * sync_error;

                    Math_Constrain(
                        &bias,
                        -CHASSIS_ANGLE_SYNC_MAX_BIAS_RAD,
                        CHASSIS_ANGLE_SYNC_MAX_BIAS_RAD);

                    /*
                     * For a negative wheel command, "more progress"
                     * means a more negative effective target.
                     */
                    if (Angle_Sync_Add[i] < 0.0f)
                    {
                        bias = -bias;
                    }

                    Motor[i].Set_Angle_Target(
                        Angle_Sync_Base_Target[i] + bias);
                }
                else
                {
                    Motor[i].Set_Angle_Target(
                        Angle_Sync_Base_Target[i]);
                }
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                Motor[i].Set_Angle_Target(
                    Angle_Sync_Base_Target[i]);
            }
        }

        /*
         * Run the existing per-wheel angle/speed closed loop.
         */
        for (int i = 0; i < 4; i++)
        {
            Motor[i].Calculate_TIM_PeriodElapsedCallback();
        }

        /*
         * Restore the real final targets immediately.
         */
        for (int i = 0; i < 4; i++)
        {
            Motor[i].Set_Angle_Target(
                Angle_Sync_Base_Target[i]);
        }

        return;
    }
#endif

    /*
     * ============================================================
     * Original OMEGA / OPENLOOP chassis velocity path
     * ============================================================
     */
    Math_Constrain(&Velocity.X, -X_MAX, X_MAX);
    Math_Constrain(&Velocity.Y, -Y_MAX, Y_MAX);
    Math_Constrain(&Velocity.Omega, -OMEGA_MAX, OMEGA_MAX);

    float right_ms = Velocity.X;
    float ahead_ms = Velocity.Y;
    float rotate_ms = -OMEGA_TO_MS * Velocity.Omega;

    float motor_omega_target[4];

    /*
     * Motor layout:
     * Motor[0] left front
     * Motor[1] right front
     * Motor[2] left rear
     * Motor[3] right rear
     */
    motor_omega_target[0] =
        (ahead_ms + right_ms - rotate_ms) / WHEEL_RADIUS;

    motor_omega_target[1] =
        (ahead_ms - right_ms + rotate_ms) / WHEEL_RADIUS;

    motor_omega_target[2] =
        (ahead_ms - right_ms - rotate_ms) / WHEEL_RADIUS;

    motor_omega_target[3] =
        (ahead_ms + right_ms + rotate_ms) / WHEEL_RADIUS;

    for (int i = 0; i < 4; i++)
    {
        Motor[i].Set_Omega_Target(
            motor_omega_target[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        Motor[i].Calculate_TIM_PeriodElapsedCallback();
    }
}

/**
 * @brief 设置底盘移动目标角度增量
 *
 * @param ahead  向前移动对应的轮子转角，正数表示向前
 * @param left   向左移动对应的轮子转角，正数表示向左
 * @param rotate 逆时针旋转对应的轮子转角，正数表示逆时针
 */
void Class_Chassis::Set_add_rad(float ahead, float left, float rotate)
{
    float motor_add_rad[4];

    /*
     * ahead  ：向前为正
     * left   ：向左为正
     * rotate ：逆时针为正
     *
     * 前进时：
     *   四个轮子目标同号
     */
    motor_add_rad[0] = ahead - left - rotate;
    motor_add_rad[1] = ahead + left + rotate;
    motor_add_rad[2] = ahead + left - rotate;
    motor_add_rad[3] = ahead - left + rotate;

    /*
     * 注意：
     * 这里不要再根据 Rotate_Direction_Flag 反转 motor_add_rad。
     *
     * 因为你开环测试时，四个轮子正 PWM 都是向前的，
     * 所以前进目标也应该四个都给正。
     */

    for (int i = 0; i < 4; i++)
    {
        const float start_angle =
            Motor[i].Get_Angle_Now();

        const float target =
            start_angle + motor_add_rad[i];

        Angle_Sync_Start[i] = start_angle;
        Angle_Sync_Add[i] = motor_add_rad[i];
        Angle_Sync_Base_Target[i] = target;

        Motor[i].Set_Angle_Target(target);

        Motor[i].Omega_PID.Set_history_IE(0.0f);
    }

    Angle_Sync_Valid = 1U;
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
