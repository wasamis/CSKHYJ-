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
 *   non-blocking line-trace action request
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

/*
 * Outer angle loop output is wheel target speed, rad/s.
 */
#define MOVE_WHEEL_OMEGA_LIMIT          8.0f

/*
 * Movement is considered complete only when all four wheels are:
 *
 *   1. close enough to their final angle target
 *   2. moving slowly enough
 *
 * and remain so for several consecutive TIM6 callbacks.
 */
/*
 * 0.12rad wheel angle is about 6mm linear travel with the current
 * 50mm wheel radius, and below 1 degree chassis yaw for a 90deg turn.
 *
 * The previous 0.05rad window made the angle loop enter a very-low
 * output region where static friction could delay completion for
 * several seconds even though the chassis had visibly stopped.
 */
#define MOVE_FINISH_ANGLE_TOL_RAD        0.12f
#define MOVE_FINISH_OMEGA_TOL_RAD_S      0.80f
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

/*
 * ============================================================
 * Internal declarations
 * ============================================================
 */

static void Move_StartMission(
    const Community_Mission_t *mission);

static void Move_StartTranslate(
    uint16_t angle_deg,
    uint8_t distance_cm);

static void Move_StartRotate(
    int16_t rotate_deg);

static void Move_StopChassis(void);

static void Move_SetLineTraceVelocity(
    float forward_mps,
    float right_mps,
    float omega_rad_s);

static void Move_ProcessLineTrace(
    uint8_t movement_finished_event);

static void Move_SetAngleSpeedLimit(void);

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

    /*
     * LineTrace owns:
     *   - I2C line-sensor sampling
     *   - Initial_Status route state machine
     *
     * PB6/PB7 should already be configured by CubeMX as:
     *   PB6 -> I2C1_SCL
     *   PB7 -> I2C1_SDA
     *
     * and MX_I2C1_Init() should run before Move_Init().
     */
    LineTrace_Init();

    s_move_active = 0U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;

    s_all_done_reported = 1U;

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

        /*
         * 同时终止巡线状态机。
         */
        LineTrace_Stop();

        Community_ClearQueue();
        Community_ClearStopRequest();

        Move_StopChassis();

        /*
         * Treat STOP as ending the current chassis command set.
         * Main loop will send 0x82 once.
         */
        s_all_done_reported = 0U;

        Community_SendDebugString(
            "Move STOP\r\n");

        return;
    }

    /*
     * One finite movement has just completed.
     *
     * The event is passed to LineTrace_Process() when line trace
     * is currently waiting for its 20cm advance or ±90° turn.
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
     * Active line-trace mission
     * ========================================================
     *
     * A line-trace mission owns the chassis until it reaches
     * its configured terminal route action.
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
        Community_SendFinish();

        s_all_done_reported = 1U;

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
 * USART1 callbacks
 * ============================================================
 */

extern "C" void Move_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        Receive_Analyse();
    }
}

extern "C" void Move_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        Community_RestartReceive();
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
    uint8_t distance_cm)
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
        wheel_rad;

    Move_SetAngleSpeedLimit();

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

    Move_SetAngleSpeedLimit();

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
    case LINETRACE_COMMAND_SET_VELOCITY:
    {
        /*
         * 只有当前没有 20cm / 90deg 有限动作时，
         * 才允许连续巡线速度覆盖底盘目标。
         */
        if (s_move_active == 0U)
        {
            Move_SetLineTraceVelocity(
                command.forward_mps,
                command.right_mps,
                command.omega_rad_s);
        }

        break;
    }

    case LINETRACE_COMMAND_MOVE_FORWARD_CM:
    {
        if (s_move_active == 0U)
        {
            Move_StartTranslate(
                0U,
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
        /*
         * 巡线 I2C 失败 / 丢线过久 / 到达终点都会走这里。
         *
         * 状态机是否结束由 LineTrace 自己决定。
         */
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

static void Move_SetLineTraceVelocity(
    float forward_mps,
    float right_mps,
    float omega_rad_s)
{
    SpeedTypeDef velocity;

    velocity.X = right_mps;
    velocity.Y = forward_mps;
    velocity.Omega = omega_rad_s;

    /*
     * 连续巡线使用速度闭环。
     *
     * Set_Control_Method(OMEGA) 会退出 ANGLE 模式；
     * TIM6 继续每 20ms 完成一次速度 PID + PWM 更新。
     */
    Move_Chassis.Set_Control_Method(
        Control_Method_OMEGA);

    Move_Chassis.Set_Velocity(
        velocity);
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

static void Move_SetAngleSpeedLimit(void)
{
    for (int i = 0; i < 4; i++)
    {
        /*
         * Angle PID output is used as wheel Omega_Target.
         */
        Move_Chassis.Motor[i].
            Angle_PID.Set_Out_Max(
                MOVE_WHEEL_OMEGA_LIMIT);
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

        const float omega_now =
            Move_Chassis.Motor[i].
                Get_Omega_Now();

        if (Move_AbsF(angle_error) >
            MOVE_FINISH_ANGLE_TOL_RAD)
        {
            return 0U;
        }

        if (Move_AbsF(omega_now) >
            MOVE_FINISH_OMEGA_TOL_RAD_S)
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
