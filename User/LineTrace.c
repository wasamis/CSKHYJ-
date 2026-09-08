#include "LineTrace.h"

#include <stddef.h>
#include <string.h>

/*
 * 0x00：当前保留用户的 10 cm 测试路线，到达橙色矿区后回传 0x82。
 * 0x01：向后 210 cm 到达搭建区，到达后回传 0x84。
 *
 * 实车已标定：正角度为右转，负角度为左转。
 */

typedef struct
{
    LineTrace_CommandType_t type;
    uint16_t angle_deg;
    uint16_t distance_cm;
    int16_t rotate_deg;

} LineTrace_Step_t;


static const LineTrace_Step_t s_initial_steps[] =
{
    {LINETRACE_COMMAND_MOVE_CM,       0U,  90U,   0},
    {LINETRACE_COMMAND_ROTATE_DEG,    0U,   0U,  90},
    {LINETRACE_COMMAND_MOVE_CM,       0U, 240U,   0},
    {LINETRACE_COMMAND_ROTATE_DEG,    0U,   0U, -90},
    {LINETRACE_COMMAND_MOVE_CM,       0U, 208U,   0},
    {LINETRACE_COMMAND_MOVE_CM,      90U,  100U,   0},
    {LINETRACE_COMMAND_MOVE_CM,      0U,  120U,   0},
    {LINETRACE_COMMAND_MOVE_CM,      180U,  15U,   0},
};


// static const LineTrace_Step_t s_initial_steps[] =
// {
//     {LINETRACE_COMMAND_MOVE_CM,       0U,  1U,   0},

// };


static const LineTrace_Step_t s_to_build_steps[] =
{
    {LINETRACE_COMMAND_MOVE_CM,     180U, 210U,   0},
};

#define LINETRACE_STEP_COUNT(steps) \
    ((uint8_t)(sizeof(steps) / sizeof((steps)[0])))

typedef enum
{
    LINETRACE_STATE_IDLE = 0,
    LINETRACE_STATE_START_STEP,
    LINETRACE_STATE_WAIT_STEP,

} LineTrace_State_t;

static volatile uint8_t s_active = 0U;
static volatile uint8_t s_finished_event = 0U;
static LineTrace_State_t s_state = LINETRACE_STATE_IDLE;
static uint8_t s_step_index = 0U;
static const LineTrace_Step_t *s_steps = NULL;
static uint8_t s_step_count = 0U;

static void LineTrace_ClearCommand(
    LineTrace_Command_t *command);

static void LineTrace_IssueCurrentStep(
    LineTrace_Command_t *command);

static void LineTrace_Finish(
    LineTrace_Command_t *command);

void LineTrace_Init(void)
{
    s_active = 0U;
    s_finished_event = 0U;
    s_state = LINETRACE_STATE_IDLE;
    s_step_index = 0U;
    s_steps = NULL;
    s_step_count = 0U;
}

uint8_t LineTrace_Start(uint8_t status)
{
    switch (status)
    {
    case LINETRACE_STATUS_INITIAL:
        s_steps = s_initial_steps;
        s_step_count =
            LINETRACE_STEP_COUNT(s_initial_steps);
        break;

    case LINETRACE_STATUS_TO_BUILD:
        s_steps = s_to_build_steps;
        s_step_count =
            LINETRACE_STEP_COUNT(s_to_build_steps);
        break;

    default:
        return 0U;
    }

    s_step_index = 0U;
    s_state = LINETRACE_STATE_START_STEP;
    s_active = 1U;
    s_finished_event = 0U;

    return 1U;
}

void LineTrace_Stop(void)
{
    s_active = 0U;
    s_finished_event = 0U;
    s_state = LINETRACE_STATE_IDLE;
    s_step_index = 0U;
    s_steps = NULL;
    s_step_count = 0U;
}

void LineTrace_Process(
    uint8_t move_busy,
    uint8_t move_finished_event,
    LineTrace_Command_t *command)
{
    if (command == NULL)
    {
        return;
    }

    LineTrace_ClearCommand(command);

    if (s_active == 0U)
    {
        return;
    }

    if (s_state == LINETRACE_STATE_START_STEP)
    {
        if (move_busy == 0U)
        {
            LineTrace_IssueCurrentStep(command);
            s_state = LINETRACE_STATE_WAIT_STEP;
        }

        return;
    }

    if (s_state != LINETRACE_STATE_WAIT_STEP)
    {
        LineTrace_Finish(command);
        return;
    }

    if (move_finished_event != 0U)
    {
        s_step_index++;

        if (s_step_index >= s_step_count)
        {
            LineTrace_Finish(command);
            return;
        }

        /* 上一步完成后立即启动下一步。 */
        LineTrace_IssueCurrentStep(command);
        return;
    }

    /*
     * WAIT_STEP 状态只等待明确的完成事件。
     *
     * 不能在 move_busy == 0 且尚未取到完成事件时重发当前步骤：
     * TIM6 可能刚在主循环读取完成标志之后结束动作，此时忙标志
     * 已经清零，但完成事件要到下一轮主循环才会被取出。若在这里
     * 重发，就会把同一段直线或转弯完整执行两次。
     */
}

uint8_t LineTrace_IsActive(void)
{
    return s_active;
}

uint8_t LineTrace_TakeFinished(void)
{
    const uint8_t finished = s_finished_event;

    s_finished_event = 0U;

    return finished;
}

static void LineTrace_ClearCommand(
    LineTrace_Command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->type = LINETRACE_COMMAND_NONE;
}

static void LineTrace_IssueCurrentStep(
    LineTrace_Command_t *command)
{
    const LineTrace_Step_t *step =
        &s_steps[s_step_index];

    command->type = step->type;
    command->angle_deg = step->angle_deg;
    command->distance_cm = step->distance_cm;
    command->rotate_deg = step->rotate_deg;
}

static void LineTrace_Finish(
    LineTrace_Command_t *command)
{
    s_active = 0U;
    s_finished_event = 1U;
    s_state = LINETRACE_STATE_IDLE;

    LineTrace_ClearCommand(command);
    command->type = LINETRACE_COMMAND_STOP;
}
