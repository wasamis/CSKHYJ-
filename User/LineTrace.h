#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * ============================================================
 * LineTrace configuration
 * ============================================================
 *
 * 硬件连线：
 *
 *   PB6 -> I2C1_SCL
 *   PB7 -> I2C1_SDA
 *
 * 用户提供的 Hiwonder 8 路巡线模块例程 / 协议已确认：
 *
 *   I2C 7-bit address : 0x5D
 *   Digital-state reg : 0x05
 *   Read length       : 1 byte
 *
 * 数字状态字节的位定义：
 *
 *   bit0 = S1
 *   bit1 = S2
 *   ...
 *   bit7 = S8
 *
 * 小车向前时用户已确认物理方向：
 *
 *   左侧                                    右侧
 *   S8  S7  S6  S5  S4  S3  S2  S1
 *
 * 厂商协议还明确说明：
 *   GPIO 检测到目标颜色时输出低电平；
 *   I2C/UART 返回值与 GPIO 电平相反。
 *
 * 因此通过 I2C 读取数字状态时：
 *
 *   bit = 1  -> 该探头检测到学习后的“目标巡线颜色”
 *   bit = 0  -> 该探头未检测到目标巡线颜色
 *
 * 所以本工程默认 active-high，不再对读取字节取反。
 */

#ifndef LINETRACE_I2C_ADDRESS_7BIT
#define LINETRACE_I2C_ADDRESS_7BIT          0x5DU
#endif

#ifndef LINETRACE_I2C_STATE_REGISTER
#define LINETRACE_I2C_STATE_REGISTER        0x05U
#endif

/*
 * 单次 I2C 主机发送或接收的最长等待时间。
 *
 * 巡线任务本身仍然是循环状态机，不使用 HAL_Delay()。
 * 每 10ms 最多进行一次 1-byte I2C 寄存器读取。
 */
#ifndef LINETRACE_I2C_TIMEOUT_MS
#define LINETRACE_I2C_TIMEOUT_MS            10U
#endif

/*
 * Hiwonder I2C 状态数据：
 *   1 = 检测到目标巡线颜色
 *   0 = 未检测到
 *
 * 因此默认必须为 0U。
 * 仅当你换成了定义相反的其它模块时才需要改成 1U。
 */
#ifndef LINETRACE_SENSOR_ACTIVE_LOW
#define LINETRACE_SENSOR_ACTIVE_LOW         0U
#endif

/*
 * 认为同时有 >=5 个探头压线时进入 T / 十字路口判定。
 * 如果你的探头数量/安装宽度不同，可单独调整。
 */
#ifndef LINETRACE_JUNCTION_MIN_ACTIVE
#define LINETRACE_JUNCTION_MIN_ACTIVE       5U
#endif

/*
 * I2C 采样周期与消抖。
 */
#ifndef LINETRACE_SAMPLE_PERIOD_MS
#define LINETRACE_SAMPLE_PERIOD_MS          10U
#endif

#ifndef LINETRACE_JUNCTION_CONFIRM_COUNT
#define LINETRACE_JUNCTION_CONFIRM_COUNT    2U
#endif

#ifndef LINETRACE_LOST_LINE_STOP_COUNT
#define LINETRACE_LOST_LINE_STOP_COUNT      5U
#endif

/*
 * 普通巡线速度参数。
 *
 * SpeedTypeDef:
 *   X     : 向右为正
 *   Y     : 向前为正
 *   Omega : 你当前 Chassis.hpp 定义中逆时针为负
 */
#ifndef LINETRACE_FORWARD_SPEED_MPS
#define LINETRACE_FORWARD_SPEED_MPS         0.20f
#endif

#ifndef LINETRACE_LOST_FORWARD_SPEED_MPS
#define LINETRACE_LOST_FORWARD_SPEED_MPS    0.05f
#endif

#ifndef LINETRACE_LATERAL_KP_MPS
#define LINETRACE_LATERAL_KP_MPS            0.10f
#endif

#ifndef LINETRACE_YAW_KP_RAD_S
#define LINETRACE_YAW_KP_RAD_S              0.45f
#endif

#ifndef LINETRACE_LATERAL_MAX_MPS
#define LINETRACE_LATERAL_MAX_MPS           0.18f
#endif

#ifndef LINETRACE_YAW_MAX_RAD_S
#define LINETRACE_YAW_MAX_RAD_S             0.70f
#endif

/*
 * 如果实际小车修正方向反了，只改这两个符号即可。
 */
#ifndef LINETRACE_LATERAL_SIGN
#define LINETRACE_LATERAL_SIGN             -1.0f
#endif

#ifndef LINETRACE_YAW_SIGN
#define LINETRACE_YAW_SIGN                 -1.0f
#endif

/*
 * 巡线模块到小车旋转中心距离 = 20 cm。
 * 识别路口后先前移到旋转中心，再处理路口动作。
 */
#define LINETRACE_CENTER_ADVANCE_CM          20U
#define LINETRACE_TURN_DEG                   90

#define LINETRACE_STATUS_INITIAL             0x00U

/*
 * ============================================================
 * LineTrace -> Move 的非阻塞动作请求
 * ============================================================
 */

typedef enum
{
    LINETRACE_COMMAND_NONE = 0,

    /*
     * 连续巡线速度命令。
     * Move.cpp 只更新速度目标，不等待完成。
     */
    LINETRACE_COMMAND_SET_VELOCITY,

    /*
     * 有限距离前移，由 Move.cpp 原有 ANGLE 闭环非阻塞执行。
     */
    LINETRACE_COMMAND_MOVE_FORWARD_CM,

    /*
     * 原地旋转，由 Move.cpp 原有 ANGLE 闭环非阻塞执行。
     */
    LINETRACE_COMMAND_ROTATE_DEG,

    /*
     * 立即把底盘速度目标置零。
     */
    LINETRACE_COMMAND_STOP,

} LineTrace_CommandType_t;

typedef struct
{
    LineTrace_CommandType_t type;

    float forward_mps;
    float right_mps;
    float omega_rad_s;

    uint8_t distance_cm;
    int16_t rotate_deg;

} LineTrace_Command_t;

/*
 * ============================================================
 * Public API
 * ============================================================
 */

/*
 * 初始化巡线模块内部状态。
 */
void LineTrace_Init(void);

/*
 * 开始一个巡线状态机。
 *
 * 当前只支持：
 *   status = 0x00
 *
 * 返回：
 *   1 = 已接受
 *   0 = 当前不支持该状态值
 */
uint8_t LineTrace_Start(uint8_t status);

/*
 * 立即终止当前巡线状态机。
 */
void LineTrace_Stop(void);

/*
 * 主循环接口。
 *
 * 必须从 Move_Process() 高频、重复调用。
 * 函数本身不使用 HAL_Delay()，不等待底盘运动完成。
 *
 * move_busy:
 *   1 = Move.cpp 当前正在执行有限距离/角度动作
 *
 * move_finished_event:
 *   1 = Move.cpp 刚刚完成了一次有限距离/角度动作
 *
 * command:
 *   LineTrace 本次希望 Move.cpp 执行的动作。
 */
void LineTrace_Process(
    uint8_t move_busy,
    uint8_t move_finished_event,
    LineTrace_Command_t *command);

uint8_t LineTrace_IsActive(void);

/*
 * 读取并清除“一次巡线任务已经到终点”的完成事件。
 */
uint8_t LineTrace_TakeFinished(void);

/*
 * 可选：运行时覆盖 I2C 7-bit 地址。
 *
 * Hiwonder 当前模块固定地址为 0x5D，
 * 正常情况下无需调用这个接口。
 */
void LineTrace_SetI2CAddress(uint8_t address_7bit);
uint8_t LineTrace_GetI2CAddress(void);

/*
 * 最近一次成功读取的原始 1 字节数据，便于调试。
 */
uint8_t LineTrace_GetLastRaw(void);

/*
 * 硬件读取层。
 *
 * 已按 Hiwonder 官方协议实现：
 *
 *   slave 7-bit address = 0x5D
 *   register            = 0x05
 *   length              = 1 byte
 *
 * LineTrace.c 严格按照厂商 STM32 例程分成两个事务：
 *
 *   HAL_I2C_Master_Transmit(..., &reg, 1, timeout);
 *   HAL_I2C_Master_Receive (..., raw,  1, timeout);
 *
 * 返回：
 *   1 = 成功得到 1 字节数字状态
 *   0 = 本次读取失败
 */
uint8_t LineTrace_PlatformReadRaw(uint8_t *raw);

#ifdef __cplusplus
}
#endif

#endif
