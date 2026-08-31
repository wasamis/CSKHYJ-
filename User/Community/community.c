#include "community.h"
#include "usart.h"

#include <string.h>

/*
 * ============================================================
 * UART
 * ============================================================
 */

static uint8_t s_uart_rx_byte = 0U;

/*
 * ============================================================
 * 接收状态机
 * ============================================================
 */

typedef enum
{
    COMMUNITY_RX_WAIT_HEAD1 = 0,
    COMMUNITY_RX_WAIT_HEAD2,
    COMMUNITY_RX_WAIT_CMD,
    COMMUNITY_RX_WAIT_PAYLOAD,

} Community_RxState_t;

static Community_RxState_t s_rx_state = COMMUNITY_RX_WAIT_HEAD1;

static uint8_t s_rx_cmd = 0U;
static uint8_t s_rx_payload[8];
static uint8_t s_rx_payload_len = 0U;
static uint8_t s_rx_payload_index = 0U;

/*
 * ============================================================
 * 任务队列
 * ============================================================
 */

static Community_Mission_t s_queue[COMMUNITY_QUEUE_SIZE];
static uint8_t s_queue_head = 0U;
static uint8_t s_queue_tail = 0U;
static uint8_t s_queue_count = 0U;

/*
 * ============================================================
 * 全局状态
 * ============================================================
 */

static volatile uint8_t s_stop_request = 0U;

/*
 * 机械臂命令不进入普通任务队列。
 * 这样命令到达后，Move_Process() 下一次循环即可立即处理，
 * 不会排在已有底盘任务后面。
 */
static volatile uint8_t s_arm_grab_request = 0U;
static volatile uint8_t s_arm_build_request = 0U;

/*
 * ArmTask 现有流程复用 Community_SendFinish()：
 *   - 搭建完成时应发送 0x85，而不是底盘完成 0x82；
 *   - 夹取完成已经发送 0x83，随后一次通用完成调用应被忽略。
 *
 * 这里在 Community 内完成兼容，不改变 ArmTask 的现有接口。
 */
static uint8_t s_arm_build_active = 0U;
static uint8_t s_suppress_next_general_finish = 0U;

/*
 * ============================================================
 * 内部函数声明
 * ============================================================
 */

static uint8_t Community_GetPayloadLength(uint8_t cmd);
static void Community_ParseFrame(uint8_t cmd,
                                 const uint8_t *payload,
                                 uint8_t len);

static uint16_t Community_ReadUInt16BE(const uint8_t *p);

/*
 * ============================================================
 * 对外接口
 * ============================================================
 */

void Community_Init(void)
{
    s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
    s_rx_cmd = 0U;
    s_rx_payload_len = 0U;
    s_rx_payload_index = 0U;

    Community_ClearQueue();

    s_stop_request = 0U;

    s_arm_grab_request = 0U;
    s_arm_build_request = 0U;
    s_arm_build_active = 0U;
    s_suppress_next_general_finish = 0U;

    Community_RestartReceive();
}

void Community_RestartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &s_uart_rx_byte, 1U);
}

/*
 * USART1 每收到 1 字节后，
 * Move_UART_RxCpltCallback() 会调用 Receive_Analyse()。
 */
void Receive_Analyse(void)
{
    Community_RxByte(s_uart_rx_byte);
    Community_RestartReceive();
}

void Community_RxByte(uint8_t data)
{
    switch (s_rx_state)
    {
    case COMMUNITY_RX_WAIT_HEAD1:
    {
        if (data == COMMUNITY_FRAME_HEAD_1)
        {
            s_rx_state = COMMUNITY_RX_WAIT_HEAD2;
        }

        break;
    }

    case COMMUNITY_RX_WAIT_HEAD2:
    {
        if (data == COMMUNITY_FRAME_HEAD_2)
        {
            s_rx_state = COMMUNITY_RX_WAIT_CMD;
        }
        else if (data == COMMUNITY_FRAME_HEAD_1)
        {
            s_rx_state = COMMUNITY_RX_WAIT_HEAD2;
        }
        else
        {
            s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
        }

        break;
    }

    case COMMUNITY_RX_WAIT_CMD:
    {
        s_rx_cmd = data;
        s_rx_payload_len =
            Community_GetPayloadLength(s_rx_cmd);

        s_rx_payload_index = 0U;

        if (s_rx_payload_len == 0U)
        {
            Community_ParseFrame(
                s_rx_cmd,
                s_rx_payload,
                0U);

            s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
        }
        else
        {
            if (s_rx_payload_len > sizeof(s_rx_payload))
            {
                s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
            }
            else
            {
                s_rx_state = COMMUNITY_RX_WAIT_PAYLOAD;
            }
        }

        break;
    }

    case COMMUNITY_RX_WAIT_PAYLOAD:
    {
        s_rx_payload[s_rx_payload_index++] = data;

        if (s_rx_payload_index >= s_rx_payload_len)
        {
            Community_ParseFrame(
                s_rx_cmd,
                s_rx_payload,
                s_rx_payload_len);

            s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
        }

        break;
    }

    default:
    {
        s_rx_state = COMMUNITY_RX_WAIT_HEAD1;
        break;
    }
    }
}

uint8_t Mission_Queue(Community_Mission_t *mission)
{
    return Community_PopMission(mission);
}

uint8_t Community_PushMission(
    const Community_Mission_t *mission)
{
    if (mission == NULL)
    {
        return 0U;
    }

    if (s_queue_count >= COMMUNITY_QUEUE_SIZE)
    {
        return 0U;
    }

    s_queue[s_queue_tail] = *mission;

    s_queue_tail++;

    if (s_queue_tail >= COMMUNITY_QUEUE_SIZE)
    {
        s_queue_tail = 0U;
    }

    s_queue_count++;

    return 1U;
}

uint8_t Community_PopMission(
    Community_Mission_t *mission)
{
    if (mission == NULL)
    {
        return 0U;
    }

    if (s_queue_count == 0U)
    {
        return 0U;
    }

    *mission = s_queue[s_queue_head];

    s_queue_head++;

    if (s_queue_head >= COMMUNITY_QUEUE_SIZE)
    {
        s_queue_head = 0U;
    }

    s_queue_count--;

    return 1U;
}

void Community_ClearQueue(void)
{
    memset(s_queue, 0, sizeof(s_queue));

    s_queue_head = 0U;
    s_queue_tail = 0U;
    s_queue_count = 0U;
}

uint8_t Community_GetQueueCount(void)
{
    return s_queue_count;
}

uint8_t Community_IsStopRequested(void)
{
    return s_stop_request;
}

void Community_ClearStopRequest(void)
{
    s_stop_request = 0U;
}

uint8_t Community_TakeArmGrabRequest(void)
{
    uint8_t request;

    request = s_arm_grab_request;
    s_arm_grab_request = 0U;

    return request;
}

uint8_t Community_TakeArmBuildRequest(void)
{
    uint8_t request;

    request = s_arm_build_request;
    s_arm_build_request = 0U;

    if (request != 0U)
    {
        s_arm_build_active = 1U;
    }

    return request;
}

void Community_SendSimpleFrame(uint8_t cmd)
{
    uint8_t tx[3];

    tx[0] = COMMUNITY_FRAME_HEAD_1;
    tx[1] = COMMUNITY_FRAME_HEAD_2;
    tx[2] = cmd;

    HAL_UART_Transmit(
        &huart1,
        tx,
        sizeof(tx),
        20U);
}

void Community_SendFinish(void)
{
    if (s_suppress_next_general_finish != 0U)
    {
        s_suppress_next_general_finish = 0U;
        return;
    }

    if (s_arm_build_active != 0U)
    {
        s_arm_build_active = 0U;

        Community_SendSimpleFrame(
            COMMUNITY_TX_ARM_BUILD_DONE);

        return;
    }

    Community_SendSimpleFrame(
        COMMUNITY_TX_CHASSIS_ALL_DONE);
}

void Community_SendArmFinish(void)
{
    /*
     * 协议规定夹取完成帧为 0x83。
     */
    Community_SendSimpleFrame(
        COMMUNITY_TX_ARM_GRAB_DONE);

    /*
     * ArmTask 现有代码随后还会调用一次 Community_SendFinish()。
     * 协议中夹取完成只需要 0x83，因此忽略紧随其后的通用完成。
     */
    s_suppress_next_general_finish = 1U;
}

void Community_SendDebugString(const char *str)
{
    /*
     * USART1 是 OpenMV 二进制协议链路。
     * 原始 ASCII 会产生协议外数据，因此默认不在该串口输出调试文本。
     * 保留空接口，避免影响 Move/ArmTask 的现有调用关系。
     */
    (void)str;
}

/*
 * ============================================================
 * 内部函数
 * ============================================================
 */

static uint8_t Community_GetPayloadLength(uint8_t cmd)
{
    switch (cmd)
    {
    case COMMUNITY_CMD_CHASSIS_MOVE:
        return 3U;

    case COMMUNITY_CMD_CHASSIS_ROTATE:
        return 2U;

    case COMMUNITY_CMD_LINE_TRACE:
        return 1U;

    case COMMUNITY_CMD_ARM_GRAB:
        return 0U;

    case COMMUNITY_CMD_STOP_ALL:
        return 0U;

    case COMMUNITY_CMD_ARM_BUILD:
        return 0U;

    case COMMUNITY_CMD_OPENLOOP_TEST:
        return 0U;

    default:
        return 0U;
    }
}

static void Community_ParseFrame(
    uint8_t cmd,
    const uint8_t *payload,
    uint8_t len)
{
    Community_Mission_t mission;

    memset(&mission, 0, sizeof(mission));

    switch (cmd)
    {
    case COMMUNITY_CMD_CHASSIS_MOVE:
    {
        if (len < 3U)
        {
            return;
        }

        mission.type =
            COMMUNITY_MISSION_CHASSIS_MOVE;

        mission.parsed.move.angle_deg =
            Community_ReadUInt16BE(&payload[0]);

        mission.parsed.move.distance_cm =
            payload[2];

        Community_PushMission(&mission);

        break;
    }

    case COMMUNITY_CMD_CHASSIS_ROTATE:
    {
        if (len < 2U)
        {
            return;
        }

        mission.type =
            COMMUNITY_MISSION_CHASSIS_ROTATE;

        /*
         * 协议：
         *   encoded = a * 256 + b
         *   clockwise_deg = encoded - 180
         *
         * Move_StartRotate() / Chassis::Set_add_rad() 使用
         * “逆时针为正”，所以这里转换为相反符号。
         */
        const int16_t clockwise_deg =
            (int16_t)Community_ReadUInt16BE(&payload[0]) - 180;

        mission.parsed.rotate_deg =
            (int16_t)(-clockwise_deg);

        Community_PushMission(&mission);

        break;
    }

    case COMMUNITY_CMD_LINE_TRACE:
    {
        if (len < 1U)
        {
            return;
        }

        /*
         * 巡线任务和普通底盘任务一样进入现有环形队列。
         * 真正巡线状态机在 Move_Process() 主循环中非阻塞运行。
         */
        mission.type =
            COMMUNITY_MISSION_LINE_TRACE;

        mission.parsed.line_trace_status =
            payload[0];

        Community_PushMission(&mission);

        break;
    }

    case COMMUNITY_CMD_ARM_GRAB:
    {
        /*
         * 不入普通队列，直接置位高优先级请求。
         * 真正动作放在主循环中执行。
         */
        s_arm_grab_request = 1U;

        break;
    }

    case COMMUNITY_CMD_ARM_BUILD:
    {
        s_arm_build_request = 1U;

        break;
    }

    case COMMUNITY_CMD_STOP_ALL:
    {
        s_stop_request = 1U;
        s_arm_grab_request = 0U;
        s_arm_build_request = 0U;
        s_arm_build_active = 0U;
        s_suppress_next_general_finish = 0U;

        Community_ClearQueue();

        /*
         * 保留 STOP 任务兼容原有调度逻辑。
         * Move_Process() 仍然优先处理 s_stop_request。
         */
        mission.type =
            COMMUNITY_MISSION_STOP_ALL;

        Community_PushMission(&mission);

        break;
    }

    case COMMUNITY_CMD_OPENLOOP_TEST:
    {
        mission.type =
            COMMUNITY_MISSION_OPENLOOP_TEST;

        Community_PushMission(&mission);

        break;
    }

    default:
    {
        break;
    }
    }
}

static uint16_t Community_ReadUInt16BE(
    const uint8_t *p)
{
    uint16_t value;

    value =
        ((uint16_t)p[0] << 8) |
        ((uint16_t)p[1]);

    return value;
}
