#ifndef COMMUNITY_H
#define COMMUNITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * ============================================================
 * 协议定义
 * ============================================================
 *
 * 帧头：66 66
 *
 * 0x02 底盘平移
 *      payload:
 *      angle_deg    uint16_t, big-endian
 *                   decoded = payload[0] * 256 + payload[1]
 *      distance_cm  uint8_t
 *
 * 0x03 底盘旋转
 *      payload:
 *      encoded_angle uint16_t, big-endian
 *      clockwise_deg = encoded_angle - 180
 *
 *      Move 内部使用“逆时针为正”，所以接收后会转换符号。
 *
 * 0x05 立即发起机械臂夹取
 *      无 payload
 *
 * 0x07 停止全部
 *      无 payload
 *
 * 0x08 立即发起搭建任务
 *      无 payload
 *
 * 0x55 开环测试
 *      无 payload
 *
 * 0x06 巡线任务
 *      payload:
 *      line_trace_status uint8_t
 *
 *      目前支持：
 *      0x00 = Initial_Status
 *             路口序列 [0, 2, 0, 1, 3]
 *             0 = 直行
 *             1 = 左转
 *             2 = 右转
 *             3 = 终点
 *      0x01 = 从矿区后退 210 cm 到搭建区
 *             完成后回传 66 66 84
 */

#define COMMUNITY_FRAME_HEAD_1              0x66U
#define COMMUNITY_FRAME_HEAD_2              0x66U

#define COMMUNITY_CMD_CHASSIS_MOVE          0x02U
#define COMMUNITY_CMD_CHASSIS_ROTATE        0x03U
#define COMMUNITY_CMD_ARM_GRAB              0x05U
#define COMMUNITY_CMD_LINE_TRACE            0x06U
#define COMMUNITY_CMD_STOP_ALL              0x07U
#define COMMUNITY_CMD_ARM_BUILD             0x08U
#define COMMUNITY_CMD_OPENLOOP_TEST         0x55U


#define COMMUNITY_TX_CHASSIS_ALL_DONE       0x82U
#define COMMUNITY_TX_ARM_GRAB_DONE          0x83U
#define COMMUNITY_TX_BUILD_AREA_ARRIVED     0x84U
#define COMMUNITY_TX_ARM_BUILD_DONE         0x85U

#ifndef COMMUNITY_QUEUE_SIZE
#define COMMUNITY_QUEUE_SIZE                16U
#endif

typedef enum
{
    COMMUNITY_MISSION_NONE = 0,

    COMMUNITY_MISSION_CHASSIS_MOVE,
    COMMUNITY_MISSION_CHASSIS_ROTATE,
    COMMUNITY_MISSION_LINE_TRACE,
    COMMUNITY_MISSION_STOP_ALL,
    COMMUNITY_MISSION_OPENLOOP_TEST,

} Community_MissionType_t;

typedef struct
{
    uint16_t angle_deg;
    uint8_t distance_cm;

} Community_MoveParsed_t;

typedef struct
{
    Community_MissionType_t type;

    union
    {
        Community_MoveParsed_t move;
        int16_t rotate_deg;
        uint8_t line_trace_status;
        uint8_t raw_u8;
    } parsed;

} Community_Mission_t;

void Community_Init(void);

void Community_RxByte(uint8_t data);
void Community_RestartReceive(UART_HandleTypeDef *huart);
void Receive_Analyse(UART_HandleTypeDef *huart);

uint8_t Mission_Queue(Community_Mission_t *mission);
uint8_t Community_PushMission(const Community_Mission_t *mission);
uint8_t Community_PopMission(Community_Mission_t *mission);
void Community_ClearQueue(void);
uint8_t Community_GetQueueCount(void);

uint8_t Community_IsStopRequested(void);
void Community_ClearStopRequest(void);

/*
 * 非底盘模块若仍使用机械臂请求，可通过以下接口读取。
 */
uint8_t Community_TakeArmGrabRequest(void);
uint8_t Community_TakeArmBuildRequest(void);

void Community_SendSimpleFrame(uint8_t cmd);
void Community_SendFinish(void);
void Community_SendArmFinish(void);
void Community_SendDebugString(const char *str);

#ifdef __cplusplus
}
#endif

#endif
