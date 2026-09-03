#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LINETRACE_STATUS_INITIAL 0x00U

/*
 * LineTrace 现在只是 0x00 路线的非阻塞位置环状态机，
 * 不再读取巡线传感器。
 */
typedef enum
{
    LINETRACE_COMMAND_NONE = 0,
    LINETRACE_COMMAND_MOVE_CM,
    LINETRACE_COMMAND_ROTATE_DEG,
    LINETRACE_COMMAND_STOP,

} LineTrace_CommandType_t;

typedef struct
{
    LineTrace_CommandType_t type;
    uint16_t angle_deg;
    uint16_t distance_cm;
    int16_t rotate_deg;

} LineTrace_Command_t;

void LineTrace_Init(void);
uint8_t LineTrace_Start(uint8_t status);
void LineTrace_Stop(void);

/*
 * 由 Move_Process() 反复调用，本函数不阻塞。
 * move_busy 表示位置环动作正在执行；
 * move_finished_event 表示上一个动作刚完成。
 */
void LineTrace_Process(
    uint8_t move_busy,
    uint8_t move_finished_event,
    LineTrace_Command_t *command);

uint8_t LineTrace_IsActive(void);
uint8_t LineTrace_TakeFinished(void);

#ifdef __cplusplus
}
#endif

#endif
