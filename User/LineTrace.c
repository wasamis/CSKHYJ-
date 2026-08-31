#include "LineTrace.h"

#include "i2c.h"

#include <string.h>

/*
 * ============================================================
 * Initial_Status route
 * ============================================================
 *
 * 用户定义：
 *
 *   [0, 2, 0, 1, 3]
 *
 *   0 = 直行
 *   1 = 左转
 *   2 = 右转
 *   3 = 终点
 */

typedef enum
{
    LINETRACE_ROUTE_STRAIGHT = 0,
    LINETRACE_ROUTE_LEFT = 1,
    LINETRACE_ROUTE_RIGHT = 2,
    LINETRACE_ROUTE_END = 3,

} LineTrace_RouteAction_t;

static const uint8_t s_initial_route[] =
{
    LINETRACE_ROUTE_STRAIGHT,
    LINETRACE_ROUTE_RIGHT,
    LINETRACE_ROUTE_STRAIGHT,
    LINETRACE_ROUTE_LEFT,
    LINETRACE_ROUTE_END
};

#define LINETRACE_INITIAL_ROUTE_COUNT \
    ((uint8_t)(sizeof(s_initial_route) / sizeof(s_initial_route[0])))

/*
 * ============================================================
 * Internal state machine
 * ============================================================
 */

typedef enum
{
    LINETRACE_STATE_IDLE = 0,

    /*
     * 正常巡线，并允许识别路口。
     */
    LINETRACE_STATE_FOLLOW,

    /*
     * 已识别路口，等待“前移 20 cm”完成。
     */
    LINETRACE_STATE_WAIT_CENTER_ADVANCE,

    /*
     * 已到路口旋转中心，等待 ±90° 转弯完成。
     */
    LINETRACE_STATE_WAIT_TURN,

} LineTrace_State_t;

/*
 * ============================================================
 * Internal data
 * ============================================================
 */

static volatile uint8_t s_active = 0U;
static volatile uint8_t s_finished_event = 0U;

static LineTrace_State_t s_state =
    LINETRACE_STATE_IDLE;

static uint8_t s_status =
    LINETRACE_STATUS_INITIAL;

static uint8_t s_route_index = 0U;

static uint8_t s_pending_action =
    LINETRACE_ROUTE_STRAIGHT;

static uint8_t s_junction_confirm_count = 0U;
/*
 * 一个路口只允许计数一次。
 *
 * 接受路口后保持锁存，直到传感器真正看到一次“非路口有效线”。
 * 这不再要求连续 3 次普通直线，只用于区分前后两个物理路口。
 */
static uint8_t s_junction_latched = 0U;
static uint8_t s_lost_line_count = 0U;

static float s_last_error = 0.0f;

static uint8_t s_last_raw = 0U;

static uint32_t s_last_sample_tick = 0U;

static uint8_t s_i2c_address =
    LINETRACE_I2C_ADDRESS_7BIT;

/*
 * 上一次有效的连续巡线速度请求。
 *
 * Move_Process() 的调用周期通常远快于 I2C 采样周期，
 * 因此两次采样之间重复输出最后一次速度目标即可。
 */
static LineTrace_Command_t s_cached_velocity_command;

/*
 * ============================================================
 * Internal declarations
 * ============================================================
 */

static void LineTrace_ClearCommand(
    LineTrace_Command_t *command);

static void LineTrace_SetVelocityCommand(
    LineTrace_Command_t *command,
    float forward_mps,
    float right_mps,
    float omega_rad_s);

static uint8_t LineTrace_UpdateSensorControl(
    LineTrace_Command_t *command);

static uint8_t LineTrace_DecodeRaw(
    uint8_t raw,
    float *error,
    uint8_t *active_count);

static float LineTrace_ConstrainF(
    float value,
    float min_value,
    float max_value);

static uint8_t LineTrace_IsSampleDue(void);

static uint8_t LineTrace_ReadRawOnce(
    uint8_t *raw);

static void LineTrace_Finish(
    LineTrace_Command_t *command);

/*
 * ============================================================
 * Public API
 * ============================================================
 */

void LineTrace_Init(void)
{
    s_active = 0U;
    s_finished_event = 0U;

    s_state = LINETRACE_STATE_IDLE;

    s_status = LINETRACE_STATUS_INITIAL;
    s_route_index = 0U;
    s_pending_action = LINETRACE_ROUTE_STRAIGHT;

    s_junction_confirm_count = 0U;
    s_junction_latched = 0U;
    s_lost_line_count = 0U;

    s_last_error = 0.0f;
    s_last_raw = 0U;

    s_i2c_address =
        LINETRACE_I2C_ADDRESS_7BIT;

    memset(
        &s_cached_velocity_command,
        0,
        sizeof(s_cached_velocity_command));

    /*
     * 让第一次调用 Process() 可以立即采样。
     */
    s_last_sample_tick =
        HAL_GetTick() - LINETRACE_SAMPLE_PERIOD_MS;
}

uint8_t LineTrace_Start(uint8_t status)
{
    if (status != LINETRACE_STATUS_INITIAL)
    {
        return 0U;
    }

    s_status = status;

    s_route_index = 0U;
    s_pending_action = LINETRACE_ROUTE_STRAIGHT;

    s_junction_confirm_count = 0U;
    s_junction_latched = 0U;
    s_lost_line_count = 0U;

    s_last_error = 0.0f;

    memset(
        &s_cached_velocity_command,
        0,
        sizeof(s_cached_velocity_command));

    s_cached_velocity_command.type =
        LINETRACE_COMMAND_SET_VELOCITY;

    s_cached_velocity_command.forward_mps =
        LINETRACE_FORWARD_SPEED_MPS;

    /*
     * 首次启动立即允许识别路口。
     * 不再要求先连续看到若干次普通直线。
     */
    s_state = LINETRACE_STATE_FOLLOW;

    s_active = 1U;
    s_finished_event = 0U;

    s_last_sample_tick =
        HAL_GetTick() - LINETRACE_SAMPLE_PERIOD_MS;

    return 1U;
}

void LineTrace_Stop(void)
{
    s_active = 0U;
    s_finished_event = 0U;

    s_state = LINETRACE_STATE_IDLE;

    s_junction_confirm_count = 0U;
    s_junction_latched = 0U;
    s_lost_line_count = 0U;

    memset(
        &s_cached_velocity_command,
        0,
        sizeof(s_cached_velocity_command));
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

    switch (s_state)
    {
    case LINETRACE_STATE_FOLLOW:
    {
        if (move_busy != 0U)
        {
            return;
        }

        /*
         * 正常巡线，同时允许识别路口。
         *
         * 返回 2 表示“本次确认了一个新路口”。
         */
        const uint8_t sensor_result =
            LineTrace_UpdateSensorControl(command);

        if (sensor_result != 2U)
        {
            return;
        }

        if (s_route_index >=
            LINETRACE_INITIAL_ROUTE_COUNT)
        {
            /*
             * 理论上不会走到这里。
             * 为安全起见，路线越界立即结束巡线。
             */
            LineTrace_Finish(command);
            return;
        }

        s_pending_action =
            s_initial_route[s_route_index];

        s_route_index++;

        /*
         * 无论直行 / 左转 / 右转 / 终点，
         * 都先向前走 20 cm，
         * 让小车旋转中心到达路口中心。
         */
        command->type =
            LINETRACE_COMMAND_MOVE_FORWARD_CM;

        command->distance_cm =
            LINETRACE_CENTER_ADVANCE_CM;

        s_state =
            LINETRACE_STATE_WAIT_CENTER_ADVANCE;

        break;
    }

    case LINETRACE_STATE_WAIT_CENTER_ADVANCE:
    {
        /*
         * 正常情况下进入本状态的同一轮 Process()
         * 已经请求了一次 20 cm 前移。
         *
         * 如果由于上层原因动作没有真正启动，
         * 在底盘空闲时重新发一次请求。
         */
        if (move_finished_event == 0U)
        {
            if (move_busy == 0U)
            {
                command->type =
                    LINETRACE_COMMAND_MOVE_FORWARD_CM;

                command->distance_cm =
                    LINETRACE_CENTER_ADVANCE_CM;
            }

            return;
        }

        switch (s_pending_action)
        {
        case LINETRACE_ROUTE_STRAIGHT:
        {
            /*
             * 直行：
             * 20 cm 前移已经完成，直接重新循线。
             */
            s_junction_confirm_count = 0U;

            /*
             * 前移 20 cm 完成后立即允许识别下一个路口。
             */
            s_state = LINETRACE_STATE_FOLLOW;

            LineTrace_SetVelocityCommand(
                command,
                LINETRACE_FORWARD_SPEED_MPS,
                0.0f,
                0.0f);

            break;
        }

        case LINETRACE_ROUTE_LEFT:
        {
            command->type =
                LINETRACE_COMMAND_ROTATE_DEG;

            /* 实车标定：负角度为左转。 */
            command->rotate_deg =
                -LINETRACE_TURN_DEG;

            s_state =
                LINETRACE_STATE_WAIT_TURN;

            break;
        }

        case LINETRACE_ROUTE_RIGHT:
        {
            command->type =
                LINETRACE_COMMAND_ROTATE_DEG;

            /* 实车标定：正角度为右转。 */
            command->rotate_deg =
                LINETRACE_TURN_DEG;

            s_state =
                LINETRACE_STATE_WAIT_TURN;

            break;
        }

        case LINETRACE_ROUTE_END:
        default:
        {
            /*
             * 终点：
             * 已经按要求先前移了 20 cm，
             * 现在停止并报告巡线任务完成。
             */
            LineTrace_Finish(command);

            break;
        }
        }

        break;
    }

    case LINETRACE_STATE_WAIT_TURN:
    {
        if (move_finished_event == 0U)
        {
            if (move_busy == 0U)
            {
                command->type =
                    LINETRACE_COMMAND_ROTATE_DEG;

                if (s_pending_action ==
                    LINETRACE_ROUTE_LEFT)
                {
                    command->rotate_deg =
                        -LINETRACE_TURN_DEG;
                }
                else
                {
                    command->rotate_deg =
                        LINETRACE_TURN_DEG;
                }
            }

            return;
        }

        /*
         * 转弯完成后立即允许识别下一个路口。
         */
        s_junction_confirm_count = 0U;
        s_lost_line_count = 0U;

        s_state = LINETRACE_STATE_FOLLOW;

        LineTrace_SetVelocityCommand(
            command,
            LINETRACE_FORWARD_SPEED_MPS,
            0.0f,
            0.0f);

        break;
    }

    case LINETRACE_STATE_IDLE:
    default:
    {
        s_active = 0U;
        break;
    }
    }
}

uint8_t LineTrace_IsActive(void)
{
    return s_active;
}

uint8_t LineTrace_TakeFinished(void)
{
    uint8_t finished;

    finished = s_finished_event;
    s_finished_event = 0U;

    return finished;
}

void LineTrace_SetI2CAddress(
    uint8_t address_7bit)
{
    /*
     * STM32 HAL 传给 Master_Receive 时再左移 1 bit。
     * 这里始终保存原始 7-bit 地址。
     */
    s_i2c_address =
        (uint8_t)(address_7bit & 0x7FU);
}

uint8_t LineTrace_GetI2CAddress(void)
{
    return s_i2c_address;
}

uint8_t LineTrace_GetLastRaw(void)
{
    return s_last_raw;
}

/*
 * ============================================================
 * Default I2C hardware read
 * ============================================================
 *
 * 如果你的巡线模块不是“直接读 1 字节”，而是：
 *
 *   先写寄存器地址 -> 再读 1 字节
 *
 * 请只改这个函数。
 */

/*
 * ============================================================
 * Hiwonder 8-ch Line Follower I2C read
 * ============================================================
 *
 * 官方 I2C 协议：
 *
 *   7-bit slave address : 0x5D
 *   state register      : 0x05
 *   state length        : 1 byte
 *
 * 等价于官方 Arduino 例程：
 *
 *   beginTransmission(0x5D)
 *   write(5)
 *   endTransmission()
 *   requestFrom(0x5D, 1)
 *
 * 厂商 STM32 例程同样使用两个独立调用：
 *   HAL_I2C_Master_Transmit()
 *   HAL_I2C_Master_Receive()
 *
 * 这里不使用 HAL_I2C_Mem_Read() 的重复起始形式，避免部分模块
 * 在超时或不接受 repeated-start 时把 SDA 留在低电平。
 */

__weak uint8_t LineTrace_PlatformReadRaw(
    uint8_t *raw)
{
    if (raw == NULL)
    {
        return 0U;
    }

    if (LineTrace_ReadRawOnce(raw) != 0U)
    {
        return 1U;
    }

    /*
     * 实车诊断发现路口附近偶发 I2C SR1.BERR。
     * HAL 返回错误后该错误状态可能持续存在，使巡线保持 STOP 数秒。
     *
     * HAL_I2C_Init 内部会软复位 I2C1 外设，然后立即重试一次；
     * 不复位 MCU，也不改变巡线路程状态。
     */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        return 0U;
    }

    return LineTrace_ReadRawOnce(raw);
}

static uint8_t LineTrace_ReadRawOnce(
    uint8_t *raw)
{
    uint8_t state_register =
        LINETRACE_I2C_STATE_REGISTER;

    if (HAL_I2C_Master_Transmit(
            &hi2c1,
            (uint16_t)(s_i2c_address << 1),
            &state_register,
            1U,
            LINETRACE_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_I2C_Master_Receive(
            &hi2c1,
            (uint16_t)(s_i2c_address << 1),
            raw,
            1U,
            LINETRACE_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/*
 * ============================================================
 * Sensor processing
 * ============================================================
 */

static uint8_t LineTrace_UpdateSensorControl(
    LineTrace_Command_t *command)
{
    uint8_t raw;
    uint8_t active_count;
    float error;

    /*
     * 两次 I2C 采样之间继续沿用上一速度目标。
     */
    if (LineTrace_IsSampleDue() == 0U)
    {
        *command = s_cached_velocity_command;
        return 1U;
    }

    if (LineTrace_PlatformReadRaw(&raw) == 0U)
    {
        /*
         * I2C 读失败：
         * 为安全起见停住，但不结束任务。
         *
         * 这样地址/接线恢复后，状态机还能继续。
         */
        LineTrace_ClearCommand(
            &s_cached_velocity_command);

        s_cached_velocity_command.type =
            LINETRACE_COMMAND_STOP;

        *command = s_cached_velocity_command;

        return 0U;
    }

    s_last_raw = raw;

    if (LineTrace_DecodeRaw(
            raw,
            &error,
            &active_count) == 0U)
    {
        /*
         * 没有任何探头检测到线。
         *
         * 短时间丢线：
         *   低速向前，并沿最后误差方向继续轻微修正。
         *
         * 连续丢线达到阈值：
         *   停住等待重新检测到线。
         */
        if (s_lost_line_count <
            LINETRACE_LOST_LINE_STOP_COUNT)
        {
            s_lost_line_count++;
        }

        if (s_lost_line_count >=
            LINETRACE_LOST_LINE_STOP_COUNT)
        {
            LineTrace_ClearCommand(
                &s_cached_velocity_command);

            s_cached_velocity_command.type =
                LINETRACE_COMMAND_STOP;
        }
        else
        {
            const float right_mps =
                LineTrace_ConstrainF(
                    LINETRACE_LATERAL_SIGN *
                    LINETRACE_LATERAL_KP_MPS *
                    s_last_error,
                    -LINETRACE_LATERAL_MAX_MPS,
                    LINETRACE_LATERAL_MAX_MPS);

            const float omega_rad_s =
                LineTrace_ConstrainF(
                    LINETRACE_YAW_SIGN *
                    LINETRACE_YAW_KP_RAD_S *
                    s_last_error,
                    -LINETRACE_YAW_MAX_RAD_S,
                    LINETRACE_YAW_MAX_RAD_S);

            LineTrace_SetVelocityCommand(
                &s_cached_velocity_command,
                LINETRACE_LOST_FORWARD_SPEED_MPS,
                right_mps,
                omega_rad_s);
        }

        s_junction_confirm_count = 0U;

        *command = s_cached_velocity_command;

        return 1U;
    }

    s_lost_line_count = 0U;
    s_last_error = error;

    /*
     * 路口判断：
     * T 字或十字路口通常会让横向探头同时有较多位压线。
     */
    if (active_count >=
        LINETRACE_JUNCTION_MIN_ACTIVE)
    {
        /*
         * 即使是路口，也继续给一个低修正的前进速度，
         * 直到确认达到消抖次数。
         */
        LineTrace_SetVelocityCommand(
            &s_cached_velocity_command,
            LINETRACE_FORWARD_SPEED_MPS,
            0.0f,
            0.0f);

        *command = s_cached_velocity_command;

        /*
         * 前一次已经接受过这个路口时，不重复增加路程索引。
         * 必须先看到一次有效的非路口线，才认为已经离开旧路口。
         */
        if (s_junction_latched != 0U)
        {
            s_junction_confirm_count = 0U;
            return 1U;
        }

        if (s_junction_confirm_count <
            LINETRACE_JUNCTION_CONFIRM_COUNT)
        {
            s_junction_confirm_count++;
        }

        if (s_junction_confirm_count >=
            LINETRACE_JUNCTION_CONFIRM_COUNT)
        {
            s_junction_confirm_count = 0U;
            s_junction_latched = 1U;

            /*
             * 返回 2：本次确认新路口。
             */
            return 2U;
        }

        return 1U;
    }

    /*
     * 成功读到有效线且探头数量低于路口阈值：
     * 已经离开上一个路口，现在可以识别下一个路口。
     */
    s_junction_latched = 0U;
    s_junction_confirm_count = 0U;

    /*
     * error:
     *   -1 左侧（靠 S8）
     *    0 中间
     *   +1 右侧（靠 S1）
     *
     * 小车：
     *   X 向右为正，因此 error > 0 时向右修正。
     *
     * 当前 Chassis.hpp 约定：
     *   Omega 逆时针为负。
     *
     * 如果实车发现横移或偏航修正方向相反，
     * 分别调整 LineTrace.h 中：
     *   LINETRACE_LATERAL_SIGN
     *   LINETRACE_YAW_SIGN
     */
    const float right_mps =
        LineTrace_ConstrainF(
            LINETRACE_LATERAL_SIGN *
            LINETRACE_LATERAL_KP_MPS *
            error,
            -LINETRACE_LATERAL_MAX_MPS,
            LINETRACE_LATERAL_MAX_MPS);

    const float omega_rad_s =
        LineTrace_ConstrainF(
            LINETRACE_YAW_SIGN *
            LINETRACE_YAW_KP_RAD_S *
            error,
            -LINETRACE_YAW_MAX_RAD_S,
            LINETRACE_YAW_MAX_RAD_S);

    LineTrace_SetVelocityCommand(
        &s_cached_velocity_command,
        LINETRACE_FORWARD_SPEED_MPS,
        right_mps,
        omega_rad_s);

    *command = s_cached_velocity_command;

    return 1U;
}

static uint8_t LineTrace_DecodeRaw(
    uint8_t raw,
    float *error,
    uint8_t *active_count)
{
    /*
     * 小车向前时的物理安装方向：
     *
     *   左侧                                    右侧
     *   S8  S7  S6  S5  S4  S3  S2  S1
     *
     * Hiwonder 官方例程已确认：
     *   bit0 = S1
     *   ...
     *   bit7 = S8
     *
     * error 定义：
     *   error < 0 : 线在车体左侧
     *   error > 0 : 线在车体右侧
     *
     * 所以 bit0/S1（最右）权重最大正值，
     * bit7/S8（最左）权重最大负值。
     */
    static const int8_t sensor_weight[8] =
    {
         7,  5,  3,  1,
        -1, -3, -5, -7
    };

    uint8_t active_bits;
    uint8_t count = 0U;
    int16_t weighted_sum = 0;

/*
     * Hiwonder I2C/UART 状态定义：
     *   1 = 检测到目标巡线颜色
     *
     * 所以默认 LINETRACE_SENSOR_ACTIVE_LOW=0，直接使用 raw。
     * 保留宏只是为了兼容未来可能更换的其它模块。
     */
#if LINETRACE_SENSOR_ACTIVE_LOW
    active_bits = (uint8_t)(~raw);
#else
    active_bits = raw;
#endif

    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((active_bits & (uint8_t)(1U << i)) != 0U)
        {
            count++;

            weighted_sum +=
                sensor_weight[i];
        }
    }

    if (active_count != NULL)
    {
        *active_count = count;
    }

    if (count == 0U)
    {
        if (error != NULL)
        {
            *error = s_last_error;
        }

        return 0U;
    }

    if (error != NULL)
    {
        /*
         * 归一化到约 -1.0 ~ +1.0。
         */
        *error =
            (float)weighted_sum /
            ((float)count * 7.0f);
    }

    return 1U;
}

/*
 * ============================================================
 * Helpers
 * ============================================================
 */

static void LineTrace_ClearCommand(
    LineTrace_Command_t *command)
{
    if (command == NULL)
    {
        return;
    }

    memset(
        command,
        0,
        sizeof(*command));

    command->type =
        LINETRACE_COMMAND_NONE;
}

static void LineTrace_SetVelocityCommand(
    LineTrace_Command_t *command,
    float forward_mps,
    float right_mps,
    float omega_rad_s)
{
    if (command == NULL)
    {
        return;
    }

    LineTrace_ClearCommand(command);

    command->type =
        LINETRACE_COMMAND_SET_VELOCITY;

    command->forward_mps =
        forward_mps;

    command->right_mps =
        right_mps;

    command->omega_rad_s =
        omega_rad_s;
}

static float LineTrace_ConstrainF(
    float value,
    float min_value,
    float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint8_t LineTrace_IsSampleDue(void)
{
    const uint32_t now =
        HAL_GetTick();

    if ((uint32_t)(now - s_last_sample_tick) <
        LINETRACE_SAMPLE_PERIOD_MS)
    {
        return 0U;
    }

    s_last_sample_tick = now;

    return 1U;
}

static void LineTrace_Finish(
    LineTrace_Command_t *command)
{
    s_active = 0U;
    s_finished_event = 1U;

    s_state = LINETRACE_STATE_IDLE;

    LineTrace_ClearCommand(
        &s_cached_velocity_command);

    if (command != NULL)
    {
        LineTrace_ClearCommand(command);

        command->type =
            LINETRACE_COMMAND_STOP;
    }
}
