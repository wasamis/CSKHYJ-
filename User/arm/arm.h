#ifndef ARM_H
#define ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// 在文件开头的包含区域后面添加
#include "BusServo.h"

// 定义舵机类型
typedef enum
{
    SERVO_TYPE_PWM = 0,
    SERVO_TYPE_BUS = 1
} Arm_ServoType_t;

/*
 * ============================================================
 * 机械臂舵机映射
 * ============================================================
 *
 * Servo 1：底座 Base      -> TIM2_CH1
 * Servo 2：大臂 Shoulder  -> TIM2_CH2
 * Servo 3：小臂 Elbow     -> TIM2_CH3
 * Servo 4：腕部 Wrist     -> TIM2_CH4
 * Servo 5：夹爪 Gripper   -> TIM3_CH1
 *
 * 当前自动流程中底座暂时不动作，只保持 HOME。
 * TIM3_CH2 / CH3 / CH4 保留给其他舵机。
 */

typedef enum
{
    ARM_SERVO_1_BASE = 0,
    ARM_SERVO_2_SHOULDER,
    ARM_SERVO_3_ELBOW,
    ARM_SERVO_4_WRIST,
    ARM_SERVO_5_GRIPPER,
    ARM_SERVO_COUNT
} Arm_Servo_t;

/*
 * ============================================================
 * 自动任务状态
 * ============================================================
 *
 * 收到夹取命令后：
 *
 * 0. 先松开夹爪
 *
 * 夹取：
 * 1. 小臂 -> PICK
 * 2. 大臂 -> PICK
 * 3. 腕部 -> PICK
 * 4. 夹爪 -> CLOSE
 *
 * 夹爪夹紧以后先执行竖直直线抬升。
 * 抬升高度和时间在 arm.c 中通过
 * ARM_VERTICAL_LIFT_HEIGHT_CM / ARM_VERTICAL_LIFT_TIME_MS 调节。
 *
 * 5. 竖直抬升
 *
 * 存放：
 * 6. 小臂 -> STORE
 * 7. 腕部 -> STORE
 * 8. 大臂 -> STORE
 * 9. 夹爪 -> OPEN
 *
 * 归位：
 * 10. 腕部 -> HOME
 * 11. 小臂 -> HOME
 * 12. 大臂 -> HOME
 */
typedef enum
{
    ARM_STATE_IDLE = 0,

    ARM_STATE_PREPARE_OPEN_GRIPPER,

    ARM_STATE_PICK_ELBOW,
    ARM_STATE_PICK_SHOULDER,
    ARM_STATE_PICK_WRIST,
    ARM_STATE_PICK_GRIPPER,

    ARM_STATE_LIFT_VERTICAL,   /* 夹紧后先竖直抬升，再进入存放 */

    ARM_STATE_STORE_SHOULDER,
    ARM_STATE_STORE_ELBOW,
    ARM_STATE_STORE_WRIST,
    ARM_STATE_STORE_RELEASE,

    ARM_STATE_HOME_WRIST,
    ARM_STATE_HOME_ELBOW,
    ARM_STATE_HOME_SHOULDER

} Arm_State_t;

/*
 * ============================================================
 * 丝杆搭建任务状态
 * ============================================================
 *
 * 0x0B 触发后：
 * 1. 从当前软件位置移动到 BUILD 位置；
 * 2. 到位后短暂停留；
 * 3. 自动返回 HOME 初始位置；
 * 4. 任务完成。
 */
typedef enum
{
    ARM_BUILD_STATE_IDLE = 0,
    ARM_BUILD_STATE_MOVE_TO_BUILD,
    ARM_BUILD_STATE_HOLD,
    ARM_BUILD_STATE_RETURN_HOME

} Arm_BuildState_t;

void Arm_Init(void);
void Arm_Process(void);

uint8_t Arm_StartGrab(void);
uint8_t Arm_StartBuild(void);
void Arm_Stop(void);

uint8_t Arm_IsBusy(void);
uint8_t Arm_TakeFinished(void);
Arm_State_t Arm_GetState(void);
uint32_t Arm_GetElapsedMs(void);

uint8_t Arm_IsBuildBusy(void);
uint8_t Arm_TakeBuildFinished(void);
Arm_BuildState_t Arm_GetBuildState(void);
int32_t Arm_GetScrewPositionSteps(void);

/*
 * 保留原上位机 0x01 接口：
 * 修改腕部 Wrist 的抓取位置。
 * 单位 us，内部限制为 500~2500 us。
 */
void Arm_SetPickWristPosition(uint16_t position);
uint16_t Arm_GetPickWristPosition(void);

/*
 * ============================================================
 * 实机调试接口
 * ============================================================
 */

uint8_t Arm_SetServoPulseUs(Arm_Servo_t servo, uint16_t pulse_us);
uint16_t Arm_GetServoPulseUs(Arm_Servo_t servo);

uint8_t Arm_MoveServoTo(Arm_Servo_t servo,
                        uint16_t target_us,
                        uint32_t duration_ms);

void Arm_SetPoseImmediate(uint16_t base_us,
                          uint16_t shoulder_us,
                          uint16_t elbow_us,
                          uint16_t wrist_us,
                          uint16_t gripper_us);

#ifdef __cplusplus
}
#endif

#endif
