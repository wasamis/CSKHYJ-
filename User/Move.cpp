#include "Move.hpp"

#include "Chassis.hpp"
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
 * This file only executes chassis translation / rotation.
 *
 * Current control chain:
 *
 *   Community command
 *       ->
 *   Community_Mission_t
 *       ->
 *   Move_Process()
 *       ->
 *   Class_Chassis::Set_add_rad()
 *       ->
 *   ANGLE mode
 *       ->
 *   wheel angle PID
 *       ->
 *   wheel speed PID
 *       ->
 *   TIM1 PWM
 *
 * TIM6 generates the 20ms control interrupt.
 *
 * No line-trace / arm / build / lead-screw logic exists here.
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
 *
 * This value limits wheel speed during position movement.
 * It can be tuned later without changing Motor / Chassis.
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
#define MOVE_FINISH_ANGLE_TOL_RAD        0.05f
#define MOVE_FINISH_OMEGA_TOL_RAD_S      0.60f
#define MOVE_FINISH_STABLE_COUNT         5U

/*
 * Current Chassis::Set_add_rad() convention:
 *
 *   ahead  > 0 : forward
 *   left   > 0 : left
 *   rotate > 0 : counter-clockwise
 *
 * If the physical rotation direction is opposite to what you want,
 * change this to -1.0f only.
 */
#define MOVE_ROTATE_SIGN                 1.0f

/*
 * ============================================================
 * Internal state
 * ============================================================
 */

static Class_Chassis Move_Chassis;

/* A translation / rotation task is currently running. */
static volatile uint8_t s_move_active = 0U;

/*
 * Number of consecutive 20ms callbacks for which all wheels
 * satisfy the finish condition.
 */
static volatile uint8_t s_finish_stable_count = 0U;

/*
 * Set in TIM6 ISR when one movement has physically finished.
 * Community transmission is intentionally left to main loop.
 */
static volatile uint8_t s_task_finished_pending = 0U;


/*
 * 0x82 is sent only after all queued chassis movement commands
 * have finished.
 */
static uint8_t s_all_done_reported = 1U;

/*
 * ============================================================
 * Internal declarations
 * ============================================================
 */

static void Move_StartMission(const Community_Mission_t *mission);

static void Move_StartTranslate(uint16_t angle_deg,
                                uint8_t distance_cm);

static void Move_StartRotate(int16_t rotate_deg);

static void Move_StopChassis(void);

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
     * Current Chassis::Init() only needs the PWM timer.
     *
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

    s_move_active = 0U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;

    s_all_done_reported = 1U;

    /*
     * TIM6 must be configured in CubeMX as a 20ms base timer
     * with global interrupt enabled.
     *
     * Example for 84MHz APB1 timer clock:
     *
     *   Prescaler = 83
     *   Period    = 19999
     *
     *   84MHz / 84 = 1MHz
     *   20000 ticks = 20ms
     */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    Community_SendDebugString("Move Init OK\r\n");
}

extern "C" void Move_Process(void)
{
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

    
        Community_ClearQueue();
        Community_ClearStopRequest();

        Move_StopChassis();

        /*
         * Treat STOP as ending the current chassis command set.
         * Main loop will send 0x82 once.
         */
        s_all_done_reported = 0U;

        Community_SendDebugString("Move STOP\r\n");
        return;
    }

    /*
     * One physical movement has just completed.
     *
     * Nothing blocking is done in TIM6 ISR.  We only clear the
     * pending flag here and continue to the next queued mission.
     */
    if (s_task_finished_pending != 0U)
    {
        s_task_finished_pending = 0U;
        Community_SendDebugString("Move task done\r\n");
    }

    /*
     * Current movement is still running.
     */
    if (s_move_active != 0U)
    {
        return;
    }

    /*
     * ========================================================
     * Fetch queued chassis missions
     * ========================================================
     *
     * We continue through non-motion commands such as 0x08.
     * As soon as a translation / rotation starts, return and let
     * TIM6 closed-loop control execute it.
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

        case COMMUNITY_MISSION_STOP_ALL:
        {
            s_move_active = 0U;
            s_finish_stable_count = 0U;
            s_task_finished_pending = 0U;

        
            Community_ClearQueue();
            Move_StopChassis();

            s_all_done_reported = 0U;

            Community_SendDebugString("Stop mission\r\n");
            break;
        }

        default:
        {
            /*
             * Move intentionally ignores all non-chassis missions.
             *
             * No arm / build / line-trace action is performed here.
             */
            break;
        }
        }
    }

    /*
     * ========================================================
     * All queued chassis movement has completed
     * ========================================================
     */
    if (s_all_done_reported == 0U)
    {
        Community_SendFinish();
        s_all_done_reported = 1U;

        Community_SendDebugString("Move all done\r\n");
    }
}

/*
 * ============================================================
 * TIM6 20ms callback
 * ============================================================
 */

extern "C" void Move_TIM_Callback(TIM_HandleTypeDef *htim)
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
     * During movement:
     *   ANGLE -> OMEGA -> PWM
     *
     * During idle:
     *   zero OMEGA target -> PWM tends to zero
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
        if (s_finish_stable_count < MOVE_FINISH_STABLE_COUNT)
        {
            s_finish_stable_count++;
        }
    }
    else
    {
        s_finish_stable_count = 0U;
    }

    if (s_finish_stable_count >= MOVE_FINISH_STABLE_COUNT)
    {
        s_move_active = 0U;
        s_finish_stable_count = 0U;
        s_task_finished_pending = 1U;

        /*
         * Stop immediately at the control level.
         * UART response is sent later from Move_Process().
         */
        Move_StopChassis();
    }
}

/*
 * ============================================================
 * USART1 callbacks
 * ============================================================
 */

extern "C" void Move_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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

extern "C" void Move_UART_ErrorCallback(UART_HandleTypeDef *huart)
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

static void Move_StartMission(const Community_Mission_t *mission)
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
 * Community move convention used here:
 *
 *   0 deg   = forward
 *   90 deg  = left
 *   180 deg = backward
 *   270 deg = right
 *
 * distance_cm is converted to wheel rotation radians.
 */

static void Move_StartTranslate(uint16_t angle_deg,
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
        (float)angle_norm * MOVE_PI / 180.0f;

    const float distance_m =
        (float)distance_cm * 0.01f;

    /*
     * Linear wheel travel -> wheel angle:
     *
     *   wheel_angle = distance / wheel_radius
     */
    const float wheel_rad =
        distance_m / WHEEL_RADIUS;

    const float ahead_rad =
        cosf(angle_rad) * wheel_rad;

    const float left_rad =
        sinf(angle_rad) * wheel_rad;

    Move_SetAngleSpeedLimit();

    Move_Chassis.Set_Control_Method(Control_Method_ANGLE);

    Move_Chassis.Set_add_rad(
        ahead_rad,
        left_rad,
        0.0f);

    s_move_active = 1U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;
    s_all_done_reported = 0U;

    Community_SendDebugString("Start move\r\n");
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

static void Move_StartRotate(int16_t rotate_deg)
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
        (float)rotate_deg * MOVE_PI / 180.0f;

    const float wheel_rotate_rad =
        MOVE_ROTATE_SIGN *
        OMEGA_TO_MS *
        chassis_rotate_rad /
        WHEEL_RADIUS;

    Move_SetAngleSpeedLimit();

    Move_Chassis.Set_Control_Method(Control_Method_ANGLE);

    Move_Chassis.Set_add_rad(
        0.0f,
        0.0f,
        wheel_rotate_rad);

    s_move_active = 1U;
    s_finish_stable_count = 0U;
    s_task_finished_pending = 0U;
    s_all_done_reported = 0U;

    Community_SendDebugString("Start rotate\r\n");
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

    Move_Chassis.Set_Velocity(zero_velocity);
    Move_Chassis.Set_Control_Method(Control_Method_OMEGA);

    /*
     * Clear speed PID integral and force PWM to zero immediately.
     *
     * The following TIM6 callbacks continue to maintain zero
     * speed target through normal OMEGA control.
     */
    for (int i = 0; i < 4; i++)
    {
        Move_Chassis.Motor[i].Set_Omega_Target(0.0f);
        Move_Chassis.Motor[i].Omega_PID.Set_history_IE(0.0f);

        Move_Chassis.Motor[i].Set_Out(0);
        Move_Chassis.Motor[i].Output();
    }
}

static void Move_SetAngleSpeedLimit(void)
{
    for (int i = 0; i < 4; i++)
    {
        /*
         * Angle PID output is used as wheel Omega_Target.
         */
        Move_Chassis.Motor[i].Angle_PID.Set_Out_Max(
            MOVE_WHEEL_OMEGA_LIMIT);
    }
}

static uint8_t Move_IsTargetReached(void)
{
    for (int i = 0; i < 4; i++)
    {
        const float angle_error =
            Move_Chassis.Motor[i].Get_Angle_Target() -
            Move_Chassis.Motor[i].Get_Angle_Now();

        const float omega_now =
            Move_Chassis.Motor[i].Get_Omega_Now();

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
