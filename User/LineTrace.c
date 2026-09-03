#include "LineTrace.h"

#include <stddef.h>
#include <string.h>

/*
 * 0x00 状态机路线：
 *
 *   1. 向前  90 cm
 *   2. 右转  90 deg
 *   3. 向前 273 cm
 *   4. 左转  90 deg
 *   5. 向前 208 cm
 *   6. 斜左前方平移：左50 cm + 前70 cm
 *      合成为方向 36 deg、距离 86 cm
 *   7. 停止并上报完成
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


// static const LineTrace_Step_t s_initial_steps[] =
// {
//     {LINETRACE_COMMAND_MOVE_CM,       0U,  90U,   0},
//     {LINETRACE_COMMAND_ROTATE_DEG,    0U,   0U,  88},
//     {LINETRACE_COMMAND_MOVE_CM,       0U, 273U,   0},
//     {LINETRACE_COMMAND_ROTATE_DEG,    0U,   0U, -90},
//     {LINETRACE_COMMAND_MOVE_CM,       0U, 208U,   0},
//     {LINETRACE_COMMAND_MOVE_CM,      270U,  100U,   0},
//     {LINETRACE_COMMAND_MOVE_CM,      0U,  100U,   0}
// };

static const LineTrace_Step_t s_initial_steps[] =
{
    {LINETRACE_COMMAND_MOVE_CM,       0U,  10U,   0},
};

#define LINETRACE_INITIAL_STEP_COUNT \
    ((uint8_t)(sizeof(s_initial_steps) / sizeof(s_initial_steps[0])))

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
}

uint8_t LineTrace_Start(uint8_t status)
{
    if (status != LINETRACE_STATUS_INITIAL)
    {
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

        if (s_step_index >= LINETRACE_INITIAL_STEP_COUNT)
        {
            LineTrace_Finish(command);
            return;
        }

        /* 上一步完成后立即启动下一步。 */
        LineTrace_IssueCurrentStep(command);
        return;
    }

    /* 如果上层未接受请求，底盘空闲时重发当前步骤。 */
    if (move_busy == 0U)
    {
        LineTrace_IssueCurrentStep(command);
    }
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
        &s_initial_steps[s_step_index];

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
