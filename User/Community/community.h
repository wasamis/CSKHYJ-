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
 *
 * 0x02 底盘平移
 *      payload:
 *      angle_deg    uint16_t, little-endian
 *      distance_cm  uint8_t
 *
 * 0x03 底盘旋转
 *      payload:
 *      rotate_deg   int16_t, little-endian
 *
 * 0x05 立即发起机械臂夹取
 *      无 payload
 *
 * 0x06 当前未使用（原巡线命令已移除）
 *
 *
 * 0x07 停止全部
 *      无 payload
 *
 *
 * 0x09 禁止 0x82 完成应答
 *      无 payload
 *
 * 0x0A 允许 0x82 完成应答
 *      无 payload
 *
 * 0x0B 立即发起搭建任务
 *      无 payload
 *      丝杆移动到搭建位置后自动返回初始位置
 *
 * 0x55 开环测试
 *      无 payload
 */

#define COMMUNITY_FRAME_HEAD_1              0x66U
#define COMMUNITY_FRAME_HEAD_2              0x66U

#define COMMUNITY_CMD_CHASSIS_MOVE          0x02U
#define COMMUNITY_CMD_CHASSIS_ROTATE        0x03U
#define COMMUNITY_CMD_ARM_GRAB              0x05U
#define COMMUNITY_CMD_STOP_ALL              0x07U
#define COMMUNITY_CMD_ENABLE_ACK            0x08U
#define COMMUNITY_CMD_ARM_BUILD             0x0BU
#define COMMUNITY_CMD_OPENLOOP_TEST         0x55U
#define COMMUNITY_TX_CHASSIS_ALL_DONE       0x82U
#define COMMUNITY_TX_ARM_GRAB_DONE          0x85U

#ifndef COMMUNITY_QUEUE_SIZE
#define COMMUNITY_QUEUE_SIZE                16U
#endif

typedef enum
{
    COMMUNITY_MISSION_NONE = 0,

    COMMUNITY_MISSION_CHASSIS_MOVE,
    COMMUNITY_MISSION_CHASSIS_ROTATE,
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
        uint8_t raw_u8;
    } parsed;

} Community_Mission_t;

void Community_Init(void);

void Community_RxByte(uint8_t data);
void Community_RestartReceive(void);
void Receive_Analyse(void);

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

void Community_SetAckEnable(uint8_t enable);
uint8_t Community_GetAckEnable(void);

#ifdef __cplusplus
}
#endif

#endif
