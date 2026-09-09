#include "Move.hpp"

#include "Chassis.hpp"
#include "LineTrace.h"
#include "community.h"

#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdint.h>

/*
 * ============================================================
 * Move module
 * ============================================================
 *
 * Current control chain:
 *
 *   Community command
 *       ->
 *   Community_Mission_t circular queue
 *       ->
 *   Move_Process()
 *       ->
 *   normal move / rotate
 *       or
 *   LineTrace_Process()
 *       ->
 *   non-blocking 0x00 position-sequence request
 *       ->
 *   Class_Chassis
 *
 * TIM6 generates the 20ms motor-control interrupt.
 *
 * Important:
 *   - No HAL_Delay() is used here.
 *   - Line trace remains a queued mission.
 *   - While line trace is active, later queued missions stay in FIFO
 *     until line trace reaches its end state.
 */

/*
 * ============================================================
 * Configuration
 * ============================================================
 */

#ifndef MOVE_PI
#define MOVE_PI                         3.14159265358979323846f
#endif

/* 角度环实际允许的最大轮速。 */
#define MOVE_ANGLE_WHEEL_OMEGA_LIMIT_RAD_S       6.0f

/*
 * 前进 2.24 m 时车体横向偏移 0.03 m：
 *
 * left_trim / ahead = 0.03 / 2.24 = 0.013
 *
 * 实车测试确认正值会向右补偿，因此反转为负值。
 */
#define MOVE_FORWARD_LEFT_TRIM_RATIO            0.013f

/*
 * Movement is complete when all four wheel-angle errors are small
 * enough for several consecutive TIM6 callbacks.
 *
 * 0.12rad wheel angle is about 6mm linear travel with the current
 * 50mm wheel radius, and below 1 degree chassis yaw for a 90deg turn.
 */
#define MOVE_FINISH_ANGLE_TOL_RAD        0.12f
#define MOVE_FINISH_STABLE_COUNT         3U

/*
 * Current Chassis::Set_add_rad() convention:
 *
 *   ahead  > 0 : forward
 *   left   > 0 : left
 *   rotate > 0 : counter-clockwise
 */
#define MOVE_ROTATE_SIGN                 1.0f

/*
 * OpenMV visual outer-loop tuning. Errors arrive in millimetres and the
 * output is chassis velocity in m/s. Start with P-only control; Ki/Kd remain
 * available as macros after the direction and Kp values are verified.
 */
#define MOVE_VISION_FORWARD_KP                   1.00f
#define MOVE_VISION_FORWARD_KI                   0.00f
#define MOVE_VISION_FORWARD_KD                   0.00f
#define MOVE_VISION_RIGHT_KP                     1.00f
#define MOVE_VISION_RIGHT_KI                     0.00f
#define MOVE_VISION_RIGHT_KD                     0.00f
#define MOVE_VISION_FORWARD_SIGN                 1.00f
#define MOVE_VISION_RIGHT_SIGN                   -1.00f
#define MOVE_VISION_MAX_SPEED_M_S                0.18f
#define MOVE_VISION_MIN_SPEED_M_S                0.04f
#define MOVE_VISION_ERROR_DEADBAND_M             0.015f
#define MOVE_VISION_INTEGRAL_LIMIT               0.20f
#define MOVE_VISION_FRAME_TIMEOUT_MS             250U
#define MOVE_VISION_DEFAULT_DT_S                 0.05f
#define MOVE_VISION_MIN_DT_S                     0.01f
#define MOVE_VISION_MAX_DT_S                     0.20f

/*
 * ============================================================
 * Internal state
 * ============================================================
 */

static Class_Chassis Move_Chassis;

/*
 * A finite translation / rotation task is currently running.
 *
 * Continuous line-following velocity mode does NOT set this flag.
 */
static volatile uint8_t s_move_active = 0U;

/*
 * Number of consecutive 20ms callbacks for which all wheels
 * satisfy the finish condition.
 */
static volatile uint8_t s_finish_stable_count = 0U;

/*
 * Set in TIM6 ISR when one finite movement has physically finished.
 * Consumed in Move_Process().
 */
static volatile uint8_t s_task_finished_pending = 0U;

/*
 * 0x82 is sent only after all queued chassis / line-trace work
 * has finished.
 */
static uint8_t s_all_done_reported = 1U;
static uint8_t s_all_done_command =
    COMMUNITY_TX_CHASSIS_ALL_DONE;

typedef struct
{
    float integral;
    float previous_error;
    uint8_t has_previous;

} Move_VisionPidState_t;

static Move_VisionPidState_t s_vision_forward_pid;
static Move_VisionPidState_t s_vision_right_pid;
static uint8_t s_vision_active = 0U;
static uint32_t s_vision_last_rx_tick_ms = 0U;

/*
 * ============================================================
 * Internal declarations
 * ============================================================
 */

static void Move_StartMission(
    const Community_Mission_t *mission);

static void Move_StartTranslate(
    uint16_t angle_deg,
    uint16_t distance_cm);

static void Move_StartRotate(
    int16_t rotate_deg);

static void Move_StopChassis(void);

static void Move_ProcessVisionTarget(
    const Community_VisionTarget_t *target);

static void Move_StopVisionControl(void);

static void Move_ResetVisionPid(void);

static float Move_VisionPidStep(
    Move_VisionPidState_t *pid,
    float error_m,
    float dt_s,
    float kp,
    float ki,
    float kd);

static void Move_ProcessLineTrace(
    uint8_t movement_finished_event);

static void Move_SetAngleSpeedLimit(
    float wheel_omega_limit_rad_s);

static uint8_t Move_IsTargetReached(void);

static float Move_AbsF(float x);

/*
 * ============================================================
 * Public API
 * ============================================================
 */

extern "C" void Move_Init(void)
{
    /*
     * Chassis internally binds:
     *
     *   Motor[0] -> TIM1_CH1 + TIM2 encoder
     *   Motor[1] -> TIM1_CH2 + TIM3 encoder
     *   Motor[2] -> TIM1_CH3 + TIM4 encoder
     *   Motor[3] -> TIM1_CH4 + TIM8 encoder
     */
    Move_Chassis.Init(htim1);

    /*
     * Idle state uses speed control with zero chassis velocity.
     */
    Move_StopChassis();

    /*
     * Community owns USART1 reception / command queue.
     */
    Community_Init();

    /* LineTrace 当前是 0x00 路线的纯位置环状态机。 */
    LineTrace_Init();

    s_move_active = 0U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;

    s_vision_active = 0U;
    s_vision_last_rx_tick_ms = 0U;
    Move_ResetVisionPid();

    s_all_done_reported = 1U;
    s_all_done_command =
        COMMUNITY_TX_CHASSIS_ALL_DONE;

    /*
     * TIM6 must be configured as a 20ms base timer
     * with global interrupt enabled.
     */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    Community_SendDebugString(
        "Move Init OK\r\n");
}

extern "C" void Move_Process(void)
{
    uint8_t movement_finished_event = 0U;
    Community_VisionTarget_t vision_target;

    /*
     * ========================================================
     * STOP has highest priority
     * ========================================================
     */
    if (Community_IsStopRequested() != 0U)
    {
        s_move_active = 0U;
        s_finish_stable_count = 0U;
        s_task_finished_pending = 0U;
        s_vision_active = 0U;
        Move_ResetVisionPid();

        /*
         * 同时终止巡线状态机。
         */
        LineTrace_Stop();

        Community_ClearQueue();
        Community_ClearVisionTarget();
        Community_ClearStopRequest();

        Move_StopChassis();

        /*
         * Treat STOP as ending the current chassis command set.
         * Main loop will send 0x82 once.
         */
        s_all_done_reported = 0U;
        s_all_done_command =
            COMMUNITY_TX_CHASSIS_ALL_DONE;

        Community_SendDebugString(
            "Move STOP\r\n");

        return;
    }

    /*
     * Vision frames form a latest-value mailbox, not a queued mission.
     * An invalid frame stops velocity control immediately; a valid frame may
     * take control only while no finite movement or route owns the chassis.
     */
    if (Community_TakeVisionTarget(&vision_target) != 0U)
    {
        Move_ProcessVisionTarget(&vision_target);
    }

    if (s_vision_active != 0U)
    {
        if ((uint32_t)(HAL_GetTick() -
                       s_vision_last_rx_tick_ms) >
            MOVE_VISION_FRAME_TIMEOUT_MS)
        {
            Move_StopVisionControl();
            Community_SendDebugString(
                "Vision timeout\r\n");
        }
        else
        {
            /* Continuous visual velocity owns the chassis for this cycle. */
            return;
        }
    }

    /*
     * One finite movement has just completed.
     *
     * The event is passed to LineTrace_Process() when the 0x00
     * position-sequence state machine is waiting for one step.
     */
    if (s_task_finished_pending != 0U)
    {
        s_task_finished_pending = 0U;
        movement_finished_event = 1U;

        Community_SendDebugString(
            "Move task done\r\n");
    }

    /*
     * ========================================================
     * Active 0x00 position-sequence mission
     * ========================================================
     *
     * The sequence owns the chassis until all five steps finish.
     *
     * Later missions remain in the Community FIFO.
     */
    if (LineTrace_IsActive() != 0U)
    {
        Move_ProcessLineTrace(
            movement_finished_event);

        /*
         * If line trace is still active, do not pop another mission.
         */
        if (LineTrace_IsActive() != 0U)
        {
            return;
        }

        /*
         * If LineTrace just ended, the finite movement must also
         * already be idle before continuing.
         */
        if (s_move_active != 0U)
        {
            return;
        }
    }
    else
    {
        /*
         * Normal finite movement still running.
         */
        if (s_move_active != 0U)
        {
            return;
        }
    }

    /*
     * One line-trace task reached its route END.
     */
    if (LineTrace_TakeFinished() != 0U)
    {
        s_all_done_reported = 0U;

        Community_SendDebugString(
            "Line trace done\r\n");
    }

    /*
     * ========================================================
     * Fetch queued missions
     * ========================================================
     */
    Community_Mission_t mission;

    while (Mission_Queue(&mission) != 0U)
    {
        switch (mission.type)
        {
        case COMMUNITY_MISSION_CHASSIS_MOVE:
        case COMMUNITY_MISSION_CHASSIS_ROTATE:
        {
            s_all_done_command =
                COMMUNITY_TX_CHASSIS_ALL_DONE;

            Move_StartMission(&mission);

            return;
        }

        case COMMUNITY_MISSION_LINE_TRACE:
        {
            /*
             * 当前只实现 0x00 Initial_Status。
             */
            if (LineTrace_Start(
                    mission.parsed.line_trace_status) != 0U)
            {
                if (mission.parsed.line_trace_status ==
                    LINETRACE_STATUS_TO_BUILD)
                {
                    s_all_done_command =
                        COMMUNITY_TX_BUILD_AREA_ARRIVED;
                }
                else
                {
                    s_all_done_command =
                        COMMUNITY_TX_CHASSIS_ALL_DONE;
                }

                s_all_done_reported = 0U;

                Move_StopChassis();

                Community_SendDebugString(
                    "Start line trace\r\n");

                /*
                 * 同一轮主循环立即执行一次接口，
                 * 不必等下一次 while(1)。
                 */
                Move_ProcessLineTrace(0U);

                return;
            }

            /*
             * 不支持的巡线状态：
             * 丢弃本任务，继续 FIFO 中的下一任务。
             */
            s_all_done_reported = 0U;
            s_all_done_command =
                COMMUNITY_TX_CHASSIS_ALL_DONE;

            Community_SendDebugString(
                "Unsupported line status\r\n");

            break;
        }

        case COMMUNITY_MISSION_STOP_ALL:
        {
            s_move_active = 0U;
            s_finish_stable_count = 0U;
            s_task_finished_pending = 0U;

            LineTrace_Stop();

            Community_ClearQueue();
            Move_StopChassis();

            s_all_done_reported = 0U;
            s_all_done_command =
                COMMUNITY_TX_CHASSIS_ALL_DONE;

            Community_SendDebugString(
                "Stop mission\r\n");

            break;
        }

        default:
        {
            /*
             * Keep compatibility with existing non-chassis missions.
             */
            break;
        }
        }
    }

    /*
     * ========================================================
     * All queued chassis / line-trace movement has completed
     * ========================================================
     */
    if (s_all_done_reported == 0U)
    {
        Community_SendSimpleFrame(
            s_all_done_command);

        s_all_done_reported = 1U;
        s_all_done_command =
            COMMUNITY_TX_CHASSIS_ALL_DONE;

        Community_SendDebugString(
            "Move all done\r\n");
    }
}

/*
 * ============================================================
 * TIM6 20ms callback
 * ============================================================
 */

extern "C" void Move_TIM_Callback(
    TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
    {
        return;
    }

    if (htim->Instance != TIM6)
    {
        return;
    }

    /*
     * Always run the chassis control update.
     *
     * Finite movement:
     *   ANGLE -> OMEGA -> PWM
     *
     * Continuous line following:
     *   chassis velocity -> wheel OMEGA -> PWM
     */
    Move_Chassis.Calculate_TIM_PeriodElapsedCallback();

    if (s_move_active == 0U)
    {
        s_finish_stable_count = 0U;
        return;
    }

    /*
     * Require several consecutive good samples so one noisy encoder
     * sample cannot prematurely terminate movement.
     */
    if (Move_IsTargetReached() != 0U)
    {
        if (s_finish_stable_count <
            MOVE_FINISH_STABLE_COUNT)
        {
            s_finish_stable_count++;
        }
    }
    else
    {
        s_finish_stable_count = 0U;
    }

    if (s_finish_stable_count >=
        MOVE_FINISH_STABLE_COUNT)
    {
        s_move_active = 0U;
        s_finish_stable_count = 0U;
        s_task_finished_pending = 1U;

        /*
         * Stop immediately at the control level.
         * UART response / state-machine progression stays in main loop.
         */
        Move_StopChassis();
    }
}

/*
 * ============================================================
 * USART1 / USART3 callbacks
 * ============================================================
 */

extern "C" void Move_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if ((huart->Instance == USART1) ||
        (huart->Instance == USART3))
    {
        Receive_Analyse(huart);
    }
}

extern "C" void Move_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if ((huart->Instance == USART1) ||
        (huart->Instance == USART3))
    {
        Community_RestartReceive(huart);
    }
}

/*
 * ============================================================
 * Mission execution
 * ============================================================
 */

static void Move_StartMission(
    const Community_Mission_t *mission)
{
    if (mission == NULL)
    {
        return;
    }

    switch (mission->type)
    {
    case COMMUNITY_MISSION_CHASSIS_MOVE:
    {
        Move_StartTranslate(
            mission->parsed.move.angle_deg,
            mission->parsed.move.distance_cm);

        break;
    }

    case COMMUNITY_MISSION_CHASSIS_ROTATE:
    {
        Move_StartRotate(
            mission->parsed.rotate_deg);

        break;
    }

    default:
    {
        break;
    }
    }
}

/*
 * ============================================================
 * Translation
 * ============================================================
 *
 * Community move convention:
 *
 *   0 deg   = forward
 *   90 deg  = left
 *   180 deg = backward
 *   270 deg = right
 */

static void Move_StartTranslate(
    uint16_t angle_deg,
    uint16_t distance_cm)
{
    if (distance_cm == 0U)
    {
        Move_StopChassis();

        s_move_active = 0U;
        s_finish_stable_count = 0U;
        s_task_finished_pending = 1U;
        s_all_done_reported = 0U;

        return;
    }

    const uint16_t angle_norm =
        (uint16_t)(angle_deg % 360U);

    const float angle_rad =
        (float)angle_norm *
        MOVE_PI /
        180.0f;

    const float distance_m =
        (float)distance_cm *
        0.01f;

    /*
     * Linear wheel travel -> wheel angle:
     *
     *   wheel_angle = distance / wheel_radius
     */
    const float wheel_rad =
        distance_m /
        WHEEL_RADIUS;

    const float ahead_rad =
        cosf(angle_rad) *
        wheel_rad;

    const float left_rad =
        sinf(angle_rad) *
        wheel_rad +
        ahead_rad *
        MOVE_FORWARD_LEFT_TRIM_RATIO;

    Move_SetAngleSpeedLimit(
        MOVE_ANGLE_WHEEL_OMEGA_LIMIT_RAD_S);

    Move_Chassis.Set_Control_Method(
        Control_Method_ANGLE);

    Move_Chassis.Set_add_rad(
        ahead_rad,
        left_rad,
        0.0f);

    s_move_active = 1U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;
    s_all_done_reported = 0U;

    Community_SendDebugString(
        "Start move\r\n");
}

/*
 * ============================================================
 * In-place rotation
 * ============================================================
 *
 * Chassis::Set_add_rad() receives "rotate" as wheel angle radians,
 * not chassis yaw radians.
 *
 * For chassis yaw theta:
 *
 *   wheel linear travel = OMEGA_TO_MS * theta
 *   wheel angle         = linear travel / WHEEL_RADIUS
 */

static void Move_StartRotate(
    int16_t rotate_deg)
{
    if (rotate_deg == 0)
    {
        Move_StopChassis();

        s_move_active = 0U;
        s_finish_stable_count = 0U;
        s_task_finished_pending = 1U;
        s_all_done_reported = 0U;

        return;
    }

    const float chassis_rotate_rad =
        (float)rotate_deg *
        MOVE_PI /
        180.0f;

    const float wheel_rotate_rad =
        MOVE_ROTATE_SIGN *
        OMEGA_TO_MS *
        chassis_rotate_rad /
        WHEEL_RADIUS;

    Move_SetAngleSpeedLimit(
        MOVE_ANGLE_WHEEL_OMEGA_LIMIT_RAD_S);

    Move_Chassis.Set_Control_Method(
        Control_Method_ANGLE);

    Move_Chassis.Set_add_rad(
        0.0f,
        0.0f,
        wheel_rotate_rad);

    s_move_active = 1U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;
    s_all_done_reported = 0U;

    Community_SendDebugString(
        "Start rotate\r\n");
}

/*
 * ============================================================
 * Line trace integration
 * ============================================================
 */

static void Move_ProcessLineTrace(
    uint8_t movement_finished_event)
{
    LineTrace_Command_t command;

    LineTrace_Process(
        s_move_active,
        movement_finished_event,
        &command);

    switch (command.type)
    {
    case LINETRACE_COMMAND_MOVE_CM:
    {
        if (s_move_active == 0U)
        {
            Move_StartTranslate(
                command.angle_deg,
                command.distance_cm);
        }

        break;
    }

    case LINETRACE_COMMAND_ROTATE_DEG:
    {
        if (s_move_active == 0U)
        {
            Move_StartRotate(
                command.rotate_deg);
        }

        break;
    }

    case LINETRACE_COMMAND_STOP:
    {
        /* 0x00 位置序列全部完成。 */
        if (s_move_active == 0U)
        {
            Move_StopChassis();
        }

        break;
    }

    case LINETRACE_COMMAND_NONE:
    default:
    {
        break;
    }
    }
}

/*
 * ============================================================
 * Continuous visual PID
 * ============================================================
 */

static void Move_ResetVisionPid(void)
{
    s_vision_forward_pid.integral = 0.0f;
    s_vision_forward_pid.previous_error = 0.0f;
    s_vision_forward_pid.has_previous = 0U;

    s_vision_right_pid.integral = 0.0f;
    s_vision_right_pid.previous_error = 0.0f;
    s_vision_right_pid.has_previous = 0U;
}

static float Move_VisionPidStep(
    Move_VisionPidState_t *pid,
    float error_m,
    float dt_s,
    float kp,
    float ki,
    float kd)
{
    float derivative = 0.0f;
    float output;

    if (Move_AbsF(error_m) <=
        MOVE_VISION_ERROR_DEADBAND_M)
    {
        pid->integral = 0.0f;
        pid->previous_error = error_m;
        pid->has_previous = 1U;
        return 0.0f;
    }

    pid->integral += error_m * dt_s;

    if (pid->integral > MOVE_VISION_INTEGRAL_LIMIT)
    {
        pid->integral = MOVE_VISION_INTEGRAL_LIMIT;
    }
    else if (pid->integral < -MOVE_VISION_INTEGRAL_LIMIT)
    {
        pid->integral = -MOVE_VISION_INTEGRAL_LIMIT;
    }

    if ((pid->has_previous != 0U) &&
        (dt_s > 0.0f))
    {
        derivative =
            (error_m - pid->previous_error) / dt_s;
    }

    pid->previous_error = error_m;
    pid->has_previous = 1U;

    output =
        kp * error_m +
        ki * pid->integral +
        kd * derivative;

    if (output > MOVE_VISION_MAX_SPEED_M_S)
    {
        output = MOVE_VISION_MAX_SPEED_M_S;
    }
    else if (output < -MOVE_VISION_MAX_SPEED_M_S)
    {
        output = -MOVE_VISION_MAX_SPEED_M_S;
    }

    if ((output > 0.0f) &&
        (output < MOVE_VISION_MIN_SPEED_M_S))
    {
        output = MOVE_VISION_MIN_SPEED_M_S;
    }
    else if ((output < 0.0f) &&
             (output > -MOVE_VISION_MIN_SPEED_M_S))
    {
        output = -MOVE_VISION_MIN_SPEED_M_S;
    }

    return output;
}

static void Move_ProcessVisionTarget(
    const Community_VisionTarget_t *target)
{
    float dt_s = MOVE_VISION_DEFAULT_DT_S;
    SpeedTypeDef velocity;

    if (target == NULL)
    {
        return;
    }

    if ((target->flags &
         COMMUNITY_VISION_FLAG_VALID) == 0U)
    {
        Move_StopVisionControl();
        return;
    }

    /* Finite movement and route state machines keep priority. */
    if ((s_move_active != 0U) ||
        (LineTrace_IsActive() != 0U) ||
        (Community_GetQueueCount() != 0U))
    {
        return;
    }

    if (s_vision_active == 0U)
    {
        Move_ResetVisionPid();
    }
    else
    {
        const uint32_t dt_ms =
            (uint32_t)(target->received_tick_ms -
                       s_vision_last_rx_tick_ms);

        dt_s = (float)dt_ms * 0.001f;

        if (dt_s < MOVE_VISION_MIN_DT_S)
        {
            dt_s = MOVE_VISION_MIN_DT_S;
        }
        else if (dt_s > MOVE_VISION_MAX_DT_S)
        {
            dt_s = MOVE_VISION_MAX_DT_S;
        }
    }

    velocity.Y =
        MOVE_VISION_FORWARD_SIGN *
        Move_VisionPidStep(
            &s_vision_forward_pid,
            (float)target->forward_error_mm * 0.001f,
            dt_s,
            MOVE_VISION_FORWARD_KP,
            MOVE_VISION_FORWARD_KI,
            MOVE_VISION_FORWARD_KD);

    velocity.X =
        MOVE_VISION_RIGHT_SIGN *
        Move_VisionPidStep(
            &s_vision_right_pid,
            (float)target->right_error_mm * 0.001f,
            dt_s,
            MOVE_VISION_RIGHT_KP,
            MOVE_VISION_RIGHT_KI,
            MOVE_VISION_RIGHT_KD);

    velocity.Omega = 0.0f;

    Move_Chassis.Set_Control_Method(
        Control_Method_OMEGA);
    Move_Chassis.Set_Velocity(velocity);

    s_vision_last_rx_tick_ms =
        target->received_tick_ms;
    s_vision_active = 1U;
}

static void Move_StopVisionControl(void)
{
    if (s_vision_active == 0U)
    {
        return;
    }

    s_vision_active = 0U;
    s_vision_last_rx_tick_ms = 0U;
    Move_ResetVisionPid();
    Move_StopChassis();
}

/*
 * ============================================================
 * Chassis stop / completion
 * ============================================================
 */

static void Move_StopChassis(void)
{
    SpeedTypeDef zero_velocity;

    zero_velocity.X = 0.0f;
    zero_velocity.Y = 0.0f;
    zero_velocity.Omega = 0.0f;

    Move_Chassis.Set_Velocity(
        zero_velocity);

    Move_Chassis.Set_Control_Method(
        Control_Method_OMEGA);

    /*
     * Clear speed PID integral and force PWM to zero immediately.
     *
     * Following TIM6 callbacks continue maintaining zero speed
     * through normal OMEGA control.
     */
    for (int i = 0; i < 4; i++)
    {
        Move_Chassis.Motor[i].
            Set_Omega_Target(0.0f);

        Move_Chassis.Motor[i].
            Omega_PID.Set_history_IE(0.0f);

        Move_Chassis.Motor[i].
            Set_Out(0);

        Move_Chassis.Motor[i].
            Output();
    }
}

static void Move_SetAngleSpeedLimit(
    float wheel_omega_limit_rad_s)
{
    for (int i = 0; i < 4; i++)
    {
        /*
         * Angle PID output is used as wheel Omega_Target.
         */
        Move_Chassis.Motor[i].
            Angle_PID.Set_Out_Max(
                wheel_omega_limit_rad_s);
    }
}

static uint8_t Move_IsTargetReached(void)
{
    for (int i = 0; i < 4; i++)
    {
        const float angle_error =
            Move_Chassis.Motor[i].
                Get_Angle_Target() -
            Move_Chassis.Motor[i].
                Get_Angle_Now();

        if (Move_AbsF(angle_error) >
            MOVE_FINISH_ANGLE_TOL_RAD)
        {
            return 0U;
        }
    }

    return 1U;
}

/*
 * ============================================================
 * Helpers
 * ============================================================
 */

static float Move_AbsF(float x)
{
    if (x >= 0.0f)
    {
        return x;
    }

    return -x;
}
