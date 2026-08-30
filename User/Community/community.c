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
static uint8_t s_ack_enable = 1U;

/*
 * 机械臂命令不进入普通任务队列。
 * 这样 0x05 到达后，Move_Process() 下一次循环即可立即处理，
 * 不会排在已有底盘任务后面。
 */
static volatile uint8_t s_arm_grab_request = 0U;
static volatile uint8_t s_arm_build_request = 0U;


/*
 * ============================================================
 * 内部函数声明
 * ============================================================
 */

static uint8_t Community_GetPayloadLength(uint8_t cmd);
static void Community_ParseFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);
static int16_t Community_ReadInt16LE(const uint8_t *p);
static uint16_t Community_ReadUInt16LE(const uint8_t *p);

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
    s_ack_enable = 1U;

    s_arm_grab_request = 0U;
    s_arm_build_request = 0U;

    Community_RestartReceive();
}

void Community_RestartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &s_uart_rx_byte, 1U);
}

/*
 * USART1 每收到 1 字节后，Move_UART_RxCpltCallback() 会调用 Receive_Analyse()。
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
        s_rx_payload_len = Community_GetPayloadLength(s_rx_cmd);
        s_rx_payload_index = 0U;

        if (s_rx_payload_len == 0U)
        {
            Community_ParseFrame(s_rx_cmd, s_rx_payload, 0U);
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
            Community_ParseFrame(s_rx_cmd, s_rx_payload, s_rx_payload_len);
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

uint8_t Community_PushMission(const Community_Mission_t *mission)
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

uint8_t Community_PopMission(Community_Mission_t *mission)
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

    return request;
}


void Community_SendSimpleFrame(uint8_t cmd)
{
    uint8_t tx[3];

    tx[0] = COMMUNITY_FRAME_HEAD_1;
    tx[1] = COMMUNITY_FRAME_HEAD_2;
    tx[2] = cmd;

    HAL_UART_Transmit(&huart1, tx, sizeof(tx), 20U);
}

void Community_SendFinish(void)
{
    if (s_ack_enable == 0U)
    {
        return;
    }

    Community_SendSimpleFrame(COMMUNITY_TX_CHASSIS_ALL_DONE);
}

void Community_SendArmFinish(void)
{
    /*
     * 0x09/0x0A 只控制 0x82。
     * 夹取专用完成帧 0x85 始终发送。
     */
    Community_SendSimpleFrame(COMMUNITY_TX_ARM_GRAB_DONE);
}

void Community_SendDebugString(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)str,
                      (uint16_t)strlen(str),
                      100U);
}

void Community_SetAckEnable(uint8_t enable)
{
    s_ack_enable = enable ? 1U : 0U;
}

uint8_t Community_GetAckEnable(void)
{
    return s_ack_enable;
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

    case COMMUNITY_CMD_ARM_GRAB:
        return 0U;

    case COMMUNITY_CMD_STOP_ALL:
        return 0U;

    case COMMUNITY_CMD_ENABLE_ACK:
        return 0U;

    case COMMUNITY_CMD_ARM_BUILD:
        return 0U;

    case COMMUNITY_CMD_OPENLOOP_TEST:
        return 0U;

    default:
        return 0U;
    }
}

static void Community_ParseFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
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

        mission.type = COMMUNITY_MISSION_CHASSIS_MOVE;
        mission.parsed.move.angle_deg = Community_ReadUInt16LE(&payload[0]);
        mission.parsed.move.distance_cm = payload[2];

        Community_PushMission(&mission);
        break;
    }

    case COMMUNITY_CMD_CHASSIS_ROTATE:
    {
        if (len < 2U)
        {
            return;
        }

        mission.type = COMMUNITY_MISSION_CHASSIS_ROTATE;
        mission.parsed.rotate_deg = Community_ReadInt16LE(&payload[0]);

        Community_PushMission(&mission);
        break;
    }

    case COMMUNITY_CMD_ARM_GRAB:
    {
        /*
         * 不入普通队列，直接置位高优先级请求。
         * 真正的舵机动作放在主循环 Move_Process() 中执行，
         * 避免在 USART1 接收中断里阻塞发送 USART2。
         */
        s_arm_grab_request = 1U;
        break;
    }

    case COMMUNITY_CMD_ARM_BUILD:
    {
        /*
         * 0x08 搭建任务同样采用直接请求标志。
         * 真正的丝杆 STEP/DIR 动作放在主循环中非阻塞执行。
         */
        s_arm_build_request = 1U;
        break;
    }

    case COMMUNITY_CMD_STOP_ALL:
    {
        s_stop_request = 1U;
        s_arm_grab_request = 0U;
        s_arm_build_request = 0U;

        Community_ClearQueue();

        /*
         * 保留 STOP 任务兼容原有调度逻辑。
         * Move_Process() 仍会优先处理 s_stop_request。
         */
        mission.type = COMMUNITY_MISSION_STOP_ALL;
        Community_PushMission(&mission);

        break;
    }

    case COMMUNITY_CMD_ENABLE_ACK:
    {
        Community_SetAckEnable(1U);
        break;
    }

    case COMMUNITY_CMD_OPENLOOP_TEST:
    {
        mission.type = COMMUNITY_MISSION_OPENLOOP_TEST;
        Community_PushMission(&mission);
        break;
    }

    default:
    {
        break;
    }
    }
}

static int16_t Community_ReadInt16LE(const uint8_t *p)
{
    uint16_t value;

    value = ((uint16_t)p[1] << 8) | ((uint16_t)p[0]);

    return (int16_t)value;
}

static uint16_t Community_ReadUInt16LE(const uint8_t *p)
{
    uint16_t value;

    value = ((uint16_t)p[1] << 8) | ((uint16_t)p[0]);

    return value;
}

