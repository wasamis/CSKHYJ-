#include "arm.h"
#include "BusServo.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>

/* ============================================================
 * 硬件映射
 * ============================================================ */

/* 底座转盘 → TIM5_CH2 */
#define ARM_BASE_PWM_TIM        htim5
#define ARM_BASE_PWM_CHANNEL    TIM_CHANNEL_2

/* 气泵和电磁阀（PF1 / PF2）— 当前仅预留，测试阶段不使用 */
#define ARM_PUMP_GPIO_PORT      GPIOF
#define ARM_PUMP_GPIO_PIN       GPIO_PIN_1
#define ARM_VALVE_GPIO_PORT     GPIOF
#define ARM_VALVE_GPIO_PIN      GPIO_PIN_2

/* 等待时间（预留） */
#define ARM_PUMP_ON_DELAY_MS    400U
#define ARM_VALVE_OFF_DELAY_MS  300U

/* 总线舵机等待时间 */
#define ARM_BUS_SERVO_BOOT_DELAY_MS         1200U
#define ARM_BUS_SERVO_COMMAND_GAP_MS          30U

/* ============================================================
 * PWM 安全范围
 * ============================================================ */
#define ARM_PWM_MIN_US          500U
#define ARM_PWM_MAX_US          2500U

/* ============================================================
 * 舵机类型映射
 * ============================================================ */
const Arm_ServoType_t g_servo_type_map[ARM_SERVO_COUNT] = {
    SERVO_TYPE_PWM,   // 底座
    SERVO_TYPE_BUS,   // 大臂 ID=1
    SERVO_TYPE_BUS,   // 小臂 ID=2
    SERVO_TYPE_BUS,   // 腕部 ID=3
};

static const uint8_t g_bus_servo_id_map[ARM_SERVO_COUNT] = {
    0,   // 底座不是总线
    1,   // 大臂 ID=1
    2,   // 小臂 ID=2
    3,   // 腕部 ID=3
};

static const uint16_t g_bus_servo_time_ms[ARM_SERVO_COUNT] = {
    0,
    300,  // 大臂运动时间
    300,  // 小臂运动时间
    300,  // 腕部运动时间
};

/* ============================================================
 * 位置参数（调参区）
 * ============================================================ */

/* 底座转盘 */
#define ARM_BASE_HOME_US                        1500U
#define ARM_BASE_STORE_US                       500U     // 180°（需实测调）

/* 大臂 Shoulder (ID=1) */
#define ARM_SHOULDER_HOME_US                    2200U
#define ARM_SHOULDER_PICK_US                    1400U

/* 小臂 Elbow (ID=2) */
#define ARM_ELBOW_HOME_US                       2200U
#define ARM_ELBOW_PICK_US                       1450U

/* 腕部 Wrist (ID=3) */
#define ARM_WRIST_HOME_US                       1200U
#define ARM_WRIST_PICK_US                       1200U

/* 竖直抬升 */
#define ARM_VERTICAL_LIFT_HEIGHT_CM             5.0f
#define ARM_VERTICAL_LIFT_TIME_MS               1500U

/* 动作时间 */
#define ARM_PICK_MOVE_MS                        900U
#define ARM_BASE_ROTATE_MS                      2000U
#define ARM_HOME_MOVE_MS                        900U

/* ============================================================
 * 二连杆几何参数
 * ============================================================ */
#define ARM_LINK1_LENGTH_CM                     10.4f
#define ARM_LINK2_LENGTH_CM                     8.3f

#define ARM_IK_EPSILON                          0.0001f
#define ARM_PI_F                                3.14159265358979323846f
#define ARM_TWO_PI_F                            (2.0f * ARM_PI_F)

/* ============================================================
 * PWM ↔ 角度标定
 * ============================================================ */
#define ARM_SHOULDER_PWM_AT_0_DEG               2400.0f
#define ARM_SHOULDER_DEG_PER_US                 0.15f
#define ARM_ELBOW_PWM_AT_0_DEG                  1200.0f
#define ARM_ELBOW_DEG_PER_US                    0.1125f
#define ARM_WRIST_PWM_AT_0_DEG                  1000.0f
#define ARM_WRIST_DEG_PER_US                    0.1125f

/* ============================================================
 * 丝杆（推杆）配置
 * ============================================================ */
#define ARM_SCREW_DIR_GPIO_PORT                 GPIOF
#define ARM_SCREW_DIR_GPIO_PIN                  GPIO_PIN_9
#define ARM_SCREW_DIR_POSITIVE_LEVEL            GPIO_PIN_SET
#define ARM_SCREW_DIR_NEGATIVE_LEVEL            GPIO_PIN_RESET

#define ARM_SCREW_HOME_POSITION_STEPS           0L
#define ARM_SCREW_PUSH_POSITION_STEPS           3000L   // 推出物块
#define ARM_SCREW_STEP_FREQUENCY_HZ             5000U
#define ARM_SCREW_TIM_COUNTER_TARGET_HZ         2000000U

#define ARM_SCREW_EXTEND_TIME_MS                1000U   // 推杆伸出等待
#define ARM_SCREW_RETRACT_TIME_MS               1000U   // 推杆缩回等待

/* ============================================================
 * 内部结构
 * ============================================================ */
typedef struct
{
    uint16_t current_us;
    uint16_t start_us;
    uint16_t target_us;
    uint32_t start_tick;
    uint32_t duration_ms;
    uint8_t active;
} Arm_ServoMotion_t;

typedef struct
{
    uint8_t active;
    uint32_t start_tick;
    uint32_t duration_ms;
    float start_x_cm;
    float start_z_cm;
    float lift_height_cm;
    float tool_angle_rad;
    float start_elbow_relative_rad;
} Arm_VerticalLift_t;

typedef struct
{
    int32_t current_position_steps;
    int32_t target_position_steps;
    int8_t step_direction;
    uint8_t running;
} Arm_ScrewMotion_t;

/* ============================================================
 * 静态变量
 * ============================================================ */
static volatile uint8_t s_arm_busy = 0U;
static volatile uint8_t s_arm_finished = 0U;
static Arm_State_t s_arm_state = ARM_STATE_IDLE;
static uint32_t s_arm_start_tick = 0U;

static Arm_ServoMotion_t s_motion[ARM_SERVO_COUNT];
static Arm_VerticalLift_t s_lift;

static uint32_t s_pump_on_tick = 0U;
static uint32_t s_valve_off_tick = 0U;

static volatile uint8_t s_build_busy = 0U;
static volatile uint8_t s_build_finished = 0U;
static Arm_BuildState_t s_build_state = ARM_BUILD_STATE_IDLE;

static volatile Arm_ScrewMotion_t s_screw_motion;

/* ============================================================
 * 内部函数声明
 * ============================================================ */
static uint16_t Arm_ClampPulseUs(uint16_t value);
static void Arm_WriteServoUs(Arm_Servo_t servo, uint16_t pulse_us);
static void Arm_UpdateServoMotion(Arm_Servo_t servo);
static void Arm_UpdateAllServoMotion(void);
static void Arm_StopServoMotion(Arm_Servo_t servo);
static uint8_t Arm_IsServoMoving(Arm_Servo_t servo);

static float Arm_DegToRad(float deg);
static float Arm_RadToDeg(float rad);
static float Arm_ShoulderPwmToAngleRad(uint16_t pwm_us);
static float Arm_ElbowPwmToRelativeRad(uint16_t pwm_us);
static float Arm_WristPwmToRelativeRad(uint16_t pwm_us);
static float Arm_ShoulderAngleRadToPwm(float angle_rad);
static float Arm_ElbowRelativeRadToPwm(float angle_rad);
static float Arm_WristRelativeRadToPwm(float angle_rad);
static float Arm_WrapAngleNear(float angle_rad, float reference_rad);
static uint8_t Arm_FloatPwmToUint16(float pwm, uint16_t *out_pwm);

static void Arm_ForwardKinematics(float shoulder_rad, float elbow_relative_rad, float *x_cm, float *z_cm);
static uint8_t Arm_SolveIK(float x_cm, float z_cm, float elbow_reference_rad, float shoulder_reference_rad, float *shoulder_rad, float *elbow_relative_rad);
static uint8_t Arm_StartVerticalLift(void);
static uint8_t Arm_UpdateVerticalLift(void);
static uint8_t Arm_ApplyLiftPoint(float x_cm, float z_cm);

static void Arm_PumpOn(void);
static void Arm_PumpOff(void);
static void Arm_PumpStop(void);
static void Arm_StartPumpOff(void);

static void Arm_ScrewGPIOInit(void);
static uint8_t Arm_ScrewConfigureTim10(uint32_t step_frequency_hz);
static void Arm_ScrewSetTarget(int32_t target_steps);
static uint8_t Arm_ScrewUpdateMotion(void);
static void Arm_ScrewStopMotion(void);

static void Arm_StartPickElbow(void);
static void Arm_StartPickShoulder(void);
static void Arm_StartPickWrist(void);
static void Arm_StartRotateBase(void);
static void Arm_StartScrewExtend(void);
static void Arm_StartScrewRetract(void);
static void Arm_StartHomeBase(void);
static void Arm_StartHomeWrist(void);
static void Arm_StartHomeElbow(void);
static void Arm_StartHomeShoulder(void);
static void Arm_FinishSequence(void);

/* ============================================================
 * 实现：PWM 底层
 * ============================================================ */
static uint16_t Arm_ClampPulseUs(uint16_t value)
{
    if (value < ARM_PWM_MIN_US) return ARM_PWM_MIN_US;
    if (value > ARM_PWM_MAX_US) return ARM_PWM_MAX_US;
    return value;
}

static void Arm_WriteServoUs(Arm_Servo_t servo, uint16_t pulse_us)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return;

    pulse_us = Arm_ClampPulseUs(pulse_us);

    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        uint8_t bus_id = g_bus_servo_id_map[servo];
        uint16_t move_time = g_bus_servo_time_ms[servo];
        if (move_time > 0U) BusServo_SetAngle(bus_id, pulse_us, move_time);
        else BusServo_SetAngle(bus_id, pulse_us, 500U);
        s_motion[index].current_us = pulse_us;
        return;
    }

    if (servo == ARM_SERVO_1_BASE)
    {
        __HAL_TIM_SET_COMPARE(&ARM_BASE_PWM_TIM, ARM_BASE_PWM_CHANNEL, pulse_us);
        s_motion[index].current_us = pulse_us;
    }
}

/* ============================================================
 * 实现：舵机运动控制
 * ============================================================ */
uint8_t Arm_MoveServoTo(Arm_Servo_t servo, uint16_t target_us, uint32_t duration_ms)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return 0U;

    target_us = Arm_ClampPulseUs(target_us);

    if (duration_ms == 0U)
    {
        Arm_StopServoMotion(servo);
        Arm_WriteServoUs(servo, target_us);
        return 1U;
    }

    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        uint8_t bus_id = g_bus_servo_id_map[servo];
        BusServo_SetAngle(bus_id, target_us, duration_ms);
        s_motion[index].start_us = s_motion[index].current_us;
        s_motion[index].target_us = target_us;
        s_motion[index].start_tick = HAL_GetTick();
        s_motion[index].duration_ms = duration_ms;
        s_motion[index].active = 1U;
        s_motion[index].current_us = target_us;
        return 1U;
    }

    s_motion[index].start_us = s_motion[index].current_us;
    s_motion[index].target_us = target_us;
    s_motion[index].start_tick = HAL_GetTick();
    s_motion[index].duration_ms = duration_ms;
    s_motion[index].active = 1U;
    return 1U;
}

static void Arm_UpdateServoMotion(Arm_Servo_t servo)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return;

    if (g_servo_type_map[servo] == SERVO_TYPE_BUS) return;

    if (s_motion[index].active == 0U) return;

    uint32_t elapsed = HAL_GetTick() - s_motion[index].start_tick;
    if (elapsed >= s_motion[index].duration_ms)
    {
        Arm_WriteServoUs(servo, s_motion[index].target_us);
        s_motion[index].active = 0U;
        return;
    }

    int32_t start = (int32_t)s_motion[index].start_us;
    int32_t target = (int32_t)s_motion[index].target_us;
    int32_t value = start + (int32_t)((int32_t)((target - start) * (int32_t)elapsed) / (int32_t)s_motion[index].duration_ms);
    Arm_WriteServoUs(servo, (uint16_t)value);
}

static void Arm_UpdateAllServoMotion(void)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_SERVO_COUNT; i++)
        Arm_UpdateServoMotion((Arm_Servo_t)i);
}

static void Arm_StopServoMotion(Arm_Servo_t servo)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return;
    s_motion[index].active = 0U;
}

static uint8_t Arm_IsServoMoving(Arm_Servo_t servo)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return 0U;

    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        if (s_motion[index].active == 0U) return 0U;
        uint32_t elapsed = HAL_GetTick() - s_motion[index].start_tick;
        if (elapsed >= (s_motion[index].duration_ms + 100U))
        {
            s_motion[index].active = 0U;
            return 0U;
        }
        return 1U;
    }
    return s_motion[index].active;
}

/* ============================================================
 * 实现：角度转换
 * ============================================================ */
static float Arm_DegToRad(float deg) { return deg * ARM_PI_F / 180.0f; }
static float Arm_RadToDeg(float rad) { return rad * 180.0f / ARM_PI_F; }

static float Arm_ShoulderPwmToAngleRad(uint16_t pwm_us)
{
    return Arm_DegToRad((ARM_SHOULDER_PWM_AT_0_DEG - (float)pwm_us) * ARM_SHOULDER_DEG_PER_US);
}

static float Arm_ElbowPwmToRelativeRad(uint16_t pwm_us)
{
    return Arm_DegToRad(((float)pwm_us - ARM_ELBOW_PWM_AT_0_DEG) * ARM_ELBOW_DEG_PER_US);
}

static float Arm_WristPwmToRelativeRad(uint16_t pwm_us)
{
    return Arm_DegToRad(-((float)pwm_us - ARM_WRIST_PWM_AT_0_DEG) * ARM_WRIST_DEG_PER_US);
}

static float Arm_ShoulderAngleRadToPwm(float angle_rad)
{
    return ARM_SHOULDER_PWM_AT_0_DEG - Arm_RadToDeg(angle_rad) / ARM_SHOULDER_DEG_PER_US;
}

static float Arm_ElbowRelativeRadToPwm(float angle_rad)
{
    return ARM_ELBOW_PWM_AT_0_DEG + Arm_RadToDeg(angle_rad) / ARM_ELBOW_DEG_PER_US;
}

static float Arm_WristRelativeRadToPwm(float angle_rad)
{
    return ARM_WRIST_PWM_AT_0_DEG - Arm_RadToDeg(angle_rad) / ARM_WRIST_DEG_PER_US;
}

static float Arm_WrapAngleNear(float angle_rad, float reference_rad)
{
    while ((angle_rad - reference_rad) > ARM_PI_F) angle_rad -= ARM_TWO_PI_F;
    while ((angle_rad - reference_rad) < -ARM_PI_F) angle_rad += ARM_TWO_PI_F;
    return angle_rad;
}

static uint8_t Arm_FloatPwmToUint16(float pwm, uint16_t *out_pwm)
{
    if (out_pwm == NULL) return 0U;
    if (pwm < (float)ARM_PWM_MIN_US || pwm > (float)ARM_PWM_MAX_US) return 0U;
    *out_pwm = (uint16_t)(pwm + 0.5f);
    return 1U;
}

/* ============================================================
 * 实现：二连杆运动学
 * ============================================================ */
static void Arm_ForwardKinematics(float shoulder_rad, float elbow_relative_rad, float *x_cm, float *z_cm)
{
    float forearm_abs_rad = shoulder_rad + elbow_relative_rad;
    if (x_cm != NULL) *x_cm = ARM_LINK1_LENGTH_CM * cosf(shoulder_rad) + ARM_LINK2_LENGTH_CM * cosf(forearm_abs_rad);
    if (z_cm != NULL) *z_cm = ARM_LINK1_LENGTH_CM * sinf(shoulder_rad) + ARM_LINK2_LENGTH_CM * sinf(forearm_abs_rad);
}

static uint8_t Arm_SolveIK(float x_cm, float z_cm, float elbow_reference_rad, float shoulder_reference_rad, float *shoulder_rad, float *elbow_relative_rad)
{
    if (shoulder_rad == NULL || elbow_relative_rad == NULL) return 0U;

    float l1 = ARM_LINK1_LENGTH_CM, l2 = ARM_LINK2_LENGTH_CM;
    float r2 = x_cm * x_cm + z_cm * z_cm;
    float cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

    if (cos_q2 > (1.0f + ARM_IK_EPSILON) || cos_q2 < (-1.0f - ARM_IK_EPSILON)) return 0U;
    if (cos_q2 > 1.0f) cos_q2 = 1.0f;
    else if (cos_q2 < -1.0f) cos_q2 = -1.0f;

    float q2 = acosf(cos_q2);
    if (elbow_reference_rad < 0.0f) q2 = -q2;

    float theta1 = atan2f(z_cm, x_cm) - atan2f(l2 * sinf(q2), l1 + l2 * cosf(q2));
    theta1 = Arm_WrapAngleNear(theta1, shoulder_reference_rad);

    *shoulder_rad = theta1;
    *elbow_relative_rad = q2;
    return 1U;
}

/* ============================================================
 * 实现：竖直抬升
 * ============================================================ */
static uint8_t Arm_StartVerticalLift(void)
{
    uint16_t shoulder_pwm = s_motion[ARM_SERVO_2_SHOULDER].current_us;
    uint16_t elbow_pwm = s_motion[ARM_SERVO_3_ELBOW].current_us;
    uint16_t wrist_pwm = s_motion[ARM_SERVO_4_WRIST].current_us;

    float shoulder_rad = Arm_ShoulderPwmToAngleRad(shoulder_pwm);
    float elbow_relative_rad = Arm_ElbowPwmToRelativeRad(elbow_pwm);
    float wrist_relative_rad = Arm_WristPwmToRelativeRad(wrist_pwm);

    float x_cm, z_cm;
    Arm_ForwardKinematics(shoulder_rad, elbow_relative_rad, &x_cm, &z_cm);

    float target_z_cm = z_cm + ARM_VERTICAL_LIFT_HEIGHT_CM;
    float test_shoulder_rad, test_elbow_rad;
    if (Arm_SolveIK(x_cm, target_z_cm, elbow_relative_rad, shoulder_rad, &test_shoulder_rad, &test_elbow_rad) == 0U)
        return 0U;

    Arm_StopServoMotion(ARM_SERVO_2_SHOULDER);
    Arm_StopServoMotion(ARM_SERVO_3_ELBOW);
    Arm_StopServoMotion(ARM_SERVO_4_WRIST);

    s_lift.active = 1U;
    s_lift.start_tick = HAL_GetTick();
    s_lift.duration_ms = ARM_VERTICAL_LIFT_TIME_MS;
    s_lift.start_x_cm = x_cm;
    s_lift.start_z_cm = z_cm;
    s_lift.lift_height_cm = ARM_VERTICAL_LIFT_HEIGHT_CM;
    s_lift.tool_angle_rad = shoulder_rad + elbow_relative_rad + wrist_relative_rad;
    s_lift.start_elbow_relative_rad = elbow_relative_rad;

    s_arm_state = ARM_STATE_LIFT_VERTICAL;
    return 1U;
}

static uint8_t Arm_UpdateVerticalLift(void)
{
    if (s_lift.active == 0U) return 1U;

    uint32_t elapsed = HAL_GetTick() - s_lift.start_tick;
    float progress = (s_lift.duration_ms == 0U) ? 1.0f :
                     (elapsed >= s_lift.duration_ms) ? 1.0f :
                     (float)elapsed / (float)s_lift.duration_ms;

    float current_z_cm = s_lift.start_z_cm + s_lift.lift_height_cm * progress;

    if (Arm_ApplyLiftPoint(s_lift.start_x_cm, current_z_cm) == 0U)
    {
        s_lift.active = 0U;
        return 1U;
    }

    if (progress >= 1.0f)
    {
        s_lift.active = 0U;
        return 1U;
    }
    return 0U;
}

static uint8_t Arm_ApplyLiftPoint(float x_cm, float z_cm)
{
    float shoulder_reference_rad = Arm_ShoulderPwmToAngleRad(s_motion[ARM_SERVO_2_SHOULDER].current_us);
    float shoulder_rad, elbow_relative_rad;

    if (Arm_SolveIK(x_cm, z_cm, s_lift.start_elbow_relative_rad, shoulder_reference_rad, &shoulder_rad, &elbow_relative_rad) == 0U)
        return 0U;

    float wrist_relative_rad = s_lift.tool_angle_rad - shoulder_rad - elbow_relative_rad;

    float shoulder_pwm_f = Arm_ShoulderAngleRadToPwm(shoulder_rad);
    float elbow_pwm_f = Arm_ElbowRelativeRadToPwm(elbow_relative_rad);
    float wrist_pwm_f = Arm_WristRelativeRadToPwm(wrist_relative_rad);

    uint16_t shoulder_pwm, elbow_pwm, wrist_pwm;
    if (!Arm_FloatPwmToUint16(shoulder_pwm_f, &shoulder_pwm)) return 0U;
    if (!Arm_FloatPwmToUint16(elbow_pwm_f, &elbow_pwm)) return 0U;
    if (!Arm_FloatPwmToUint16(wrist_pwm_f, &wrist_pwm)) return 0U;

    Arm_WriteServoUs(ARM_SERVO_2_SHOULDER, shoulder_pwm);
    Arm_WriteServoUs(ARM_SERVO_3_ELBOW, elbow_pwm);
    Arm_WriteServoUs(ARM_SERVO_4_WRIST, wrist_pwm);

    return 1U;
}

/* ============================================================
 * 实现：气泵/电磁阀控制（当前阶段仅预留，不实际使用）
 * ============================================================ */
static void Arm_PumpOn(void)
{
    // 预留：实际使用时取消注释
    // HAL_GPIO_WritePin(ARM_PUMP_GPIO_PORT, ARM_PUMP_GPIO_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(ARM_VALVE_GPIO_PORT, ARM_VALVE_GPIO_PIN, GPIO_PIN_RESET);
    s_pump_on_tick = HAL_GetTick();
    s_arm_state = ARM_STATE_PUMP_ON;
}

static void Arm_PumpOff(void)
{
    // 预留：实际使用时取消注释
    // HAL_GPIO_WritePin(ARM_VALVE_GPIO_PORT, ARM_VALVE_GPIO_PIN, GPIO_PIN_RESET);
    s_valve_off_tick = HAL_GetTick();
    s_arm_state = ARM_STATE_PUMP_OFF;
}

static void Arm_PumpStop(void)
{
    // 预留：实际使用时取消注释
    // HAL_GPIO_WritePin(ARM_PUMP_GPIO_PORT, ARM_PUMP_GPIO_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(ARM_VALVE_GPIO_PORT, ARM_VALVE_GPIO_PIN, GPIO_PIN_RESET);
}

static void Arm_StartPumpOff(void)
{
    Arm_PumpOff();
}

/* ============================================================
 * 实现：丝杆（推杆）
 * ============================================================ */
static void Arm_ScrewGPIOInit(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    gpio_init.Pin = ARM_SCREW_DIR_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ARM_SCREW_DIR_GPIO_PORT, &gpio_init);
    HAL_GPIO_WritePin(ARM_SCREW_DIR_GPIO_PORT, ARM_SCREW_DIR_GPIO_PIN, ARM_SCREW_DIR_POSITIVE_LEVEL);
}

static uint8_t Arm_ScrewConfigureTim10(uint32_t step_frequency_hz)
{
    if (step_frequency_hz == 0U) return 0U;

    RCC_ClkInitTypeDef clock_config;
    uint32_t flash_latency;
    uint32_t pclk2_hz = HAL_RCC_GetPCLK2Freq();
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);

    uint32_t tim10_clock_hz = (clock_config.APB2CLKDivider == RCC_HCLK_DIV1) ? pclk2_hz : pclk2_hz * 2U;

    uint32_t prescaler_div = (tim10_clock_hz + (ARM_SCREW_TIM_COUNTER_TARGET_HZ / 2U)) / ARM_SCREW_TIM_COUNTER_TARGET_HZ;
    if (prescaler_div == 0U) prescaler_div = 1U;
    if (prescaler_div > 65536U) return 0U;

    uint32_t timer_counter_hz = tim10_clock_hz / prescaler_div;
    uint32_t period_counts = (timer_counter_hz + (step_frequency_hz / 2U)) / step_frequency_hz;
    if (period_counts < 2U) period_counts = 2U;
    if (period_counts > 65536U) return 0U;

    uint32_t arr = period_counts - 1U;
    uint32_t pulse = period_counts / 2U;
    if (pulse == 0U) pulse = 1U;

    __HAL_TIM_DISABLE(&htim10);
    __HAL_TIM_SET_PRESCALER(&htim10, prescaler_div - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim10, arr);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COUNTER(&htim10, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    htim10.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    return 1U;
}

static void Arm_ScrewSetTarget(int32_t target_steps)
{
    HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);
    s_screw_motion.running = 0U;

    int32_t current_steps = s_screw_motion.current_position_steps;
    s_screw_motion.target_position_steps = target_steps;

    if (target_steps == current_steps) return;

    if (target_steps > current_steps)
    {
        s_screw_motion.step_direction = 1;
        HAL_GPIO_WritePin(ARM_SCREW_DIR_GPIO_PORT, ARM_SCREW_DIR_GPIO_PIN, ARM_SCREW_DIR_POSITIVE_LEVEL);
    }
    else
    {
        s_screw_motion.step_direction = -1;
        HAL_GPIO_WritePin(ARM_SCREW_DIR_GPIO_PORT, ARM_SCREW_DIR_GPIO_PIN, ARM_SCREW_DIR_NEGATIVE_LEVEL);
    }

    uint32_t first_counter = __HAL_TIM_GET_COMPARE(&htim10, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim10, first_counter);
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    s_screw_motion.running = 1U;
    if (HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
        s_screw_motion.running = 0U;
        s_screw_motion.target_position_steps = s_screw_motion.current_position_steps;
    }
}

static uint8_t Arm_ScrewUpdateMotion(void)
{
    if ((s_screw_motion.running == 0U) &&
        (s_screw_motion.current_position_steps == s_screw_motion.target_position_steps))
        return 1U;
    return 0U;
}

static void Arm_ScrewStopMotion(void)
{
    HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);
    s_screw_motion.running = 0U;
    s_screw_motion.target_position_steps = s_screw_motion.current_position_steps;
}

/* ============================================================
 * 实现：TIM10 计步回调
 * ============================================================ */
void Arm_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM10)) return;
    if (s_screw_motion.running == 0U) return;

    if (s_screw_motion.current_position_steps == s_screw_motion.target_position_steps)
    {
        Arm_ScrewStopMotion();
        return;
    }

    s_screw_motion.current_position_steps += (int32_t)s_screw_motion.step_direction;

    if (s_screw_motion.current_position_steps == s_screw_motion.target_position_steps)
    {
        Arm_ScrewStopMotion();
    }
}

/* ============================================================
 * 实现：动作启动函数
 * ============================================================ */
static void Arm_StartPickElbow(void)
{
    Arm_MoveServoTo(ARM_SERVO_3_ELBOW, ARM_ELBOW_PICK_US, ARM_PICK_MOVE_MS);
    s_arm_state = ARM_STATE_PICK_ELBOW;
}

static void Arm_StartPickShoulder(void)
{
    Arm_MoveServoTo(ARM_SERVO_2_SHOULDER, ARM_SHOULDER_PICK_US, ARM_PICK_MOVE_MS);
    s_arm_state = ARM_STATE_PICK_SHOULDER;
}

static void Arm_StartPickWrist(void)
{
    Arm_MoveServoTo(ARM_SERVO_4_WRIST, ARM_WRIST_PICK_US, ARM_PICK_MOVE_MS);
    s_arm_state = ARM_STATE_PICK_WRIST;
}

static void Arm_StartRotateBase(void)
{
    Arm_MoveServoTo(ARM_SERVO_1_BASE, ARM_BASE_STORE_US, ARM_BASE_ROTATE_MS);
    s_arm_state = ARM_STATE_ROTATE_BASE;
}

static void Arm_StartScrewExtend(void)
{
    Arm_ScrewSetTarget(ARM_SCREW_PUSH_POSITION_STEPS);
    s_arm_state = ARM_STATE_SCREW_EXTEND;
}

static void Arm_StartScrewRetract(void)
{
    Arm_ScrewSetTarget(ARM_SCREW_HOME_POSITION_STEPS);
    s_arm_state = ARM_STATE_SCREW_RETRACT;
}

static void Arm_StartHomeBase(void)
{
    Arm_MoveServoTo(ARM_SERVO_1_BASE, ARM_BASE_HOME_US, ARM_HOME_MOVE_MS);
    s_arm_state = ARM_STATE_HOME_BASE;
}

static void Arm_StartHomeWrist(void)
{
    Arm_MoveServoTo(ARM_SERVO_4_WRIST, ARM_WRIST_HOME_US, ARM_HOME_MOVE_MS);
    s_arm_state = ARM_STATE_HOME_WRIST;
}

static void Arm_StartHomeElbow(void)
{
    Arm_MoveServoTo(ARM_SERVO_3_ELBOW, ARM_ELBOW_HOME_US, ARM_HOME_MOVE_MS);
    s_arm_state = ARM_STATE_HOME_ELBOW;
}

static void Arm_StartHomeShoulder(void)
{
    Arm_MoveServoTo(ARM_SERVO_2_SHOULDER, ARM_SHOULDER_HOME_US, ARM_HOME_MOVE_MS);
    s_arm_state = ARM_STATE_HOME_SHOULDER;
}

static void Arm_FinishSequence(void)
{
    s_arm_state = ARM_STATE_FINISH;
}

/* ============================================================
 * 实现：公共接口
 * ============================================================ */
void Arm_Init(void)
{
    uint32_t i;

    /* 1. 初始化 GPIO（气泵 + 电磁阀）— 当前仅预留 */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = ARM_PUMP_GPIO_PIN | ARM_VALVE_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio_init);
    Arm_PumpStop();

    /* 2. 初始化丝杆 GPIO */
    Arm_ScrewGPIOInit();
    HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);
    if (Arm_ScrewConfigureTim10(ARM_SCREW_STEP_FREQUENCY_HZ) == 0U)
    {
        Error_Handler();
    }

    /* 3. 初始化 PWM 舵机运动结构 */
    for (i = 0U; i < (uint32_t)ARM_SERVO_COUNT; i++)
    {
        s_motion[i].current_us = 1500U;
        s_motion[i].start_us = 1500U;
        s_motion[i].target_us = 1500U;
        s_motion[i].start_tick = 0U;
        s_motion[i].duration_ms = 0U;
        s_motion[i].active = 0U;
    }

    /* 4. 初始化总线舵机串口 */
    extern UART_HandleTypeDef huart2;
    BusServo_Init(&huart2);

    /* 5. 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&ARM_BASE_PWM_TIM, ARM_BASE_PWM_CHANNEL);

    /* 6. 等待总线舵机完成上电初始化 */
    HAL_Delay(ARM_BUS_SERVO_BOOT_DELAY_MS);

    /* 7. 设置上电 HOME 位置 */
    Arm_WriteServoUs(ARM_SERVO_1_BASE, ARM_BASE_HOME_US);
    Arm_WriteServoUs(ARM_SERVO_2_SHOULDER, ARM_SHOULDER_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);
    Arm_WriteServoUs(ARM_SERVO_3_ELBOW, ARM_ELBOW_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);
    Arm_WriteServoUs(ARM_SERVO_4_WRIST, ARM_WRIST_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);

    /* 8. 初始化状态标志 */
    s_arm_busy = 0U;
    s_arm_finished = 0U;
    s_arm_state = ARM_STATE_IDLE;
    s_arm_start_tick = 0U;

    s_lift.active = 0U;
    s_lift.start_tick = 0U;
    s_lift.duration_ms = 0U;
    s_lift.start_x_cm = 0.0f;
    s_lift.start_z_cm = 0.0f;
    s_lift.lift_height_cm = 0.0f;
    s_lift.tool_angle_rad = 0.0f;
    s_lift.start_elbow_relative_rad = 0.0f;

    s_screw_motion.current_position_steps = ARM_SCREW_HOME_POSITION_STEPS;
    s_screw_motion.target_position_steps = ARM_SCREW_HOME_POSITION_STEPS;
    s_screw_motion.step_direction = 1;
    s_screw_motion.running = 0U;

    s_pump_on_tick = 0U;
    s_valve_off_tick = 0U;

    s_build_busy = 0U;
    s_build_finished = 0U;
    s_build_state = ARM_BUILD_STATE_IDLE;
}

uint8_t Arm_StartGrab(void)
{
    if (s_arm_busy != 0U) return 0U;

    s_arm_start_tick = HAL_GetTick();
    s_arm_finished = 0U;
    s_arm_busy = 1U;

    Arm_StartPickElbow();
    return 1U;
}

uint8_t Arm_StartBuild(void)
{
    if (s_arm_busy != 0U) return 0U;

    s_build_finished = 0U;
    s_build_busy = 1U;

    Arm_ScrewSetTarget(ARM_SCREW_PUSH_POSITION_STEPS);
    s_build_state = ARM_BUILD_STATE_MOVE_TO_BUILD;

    return 1U;
}

void Arm_Stop(void)
{
    uint32_t i;
    Arm_UpdateAllServoMotion();
    s_lift.active = 0U;
    for (i = 0U; i < (uint32_t)ARM_SERVO_COUNT; i++)
        Arm_StopServoMotion((Arm_Servo_t)i);
    Arm_ScrewStopMotion();
    Arm_PumpStop();
    s_arm_busy = 0U;
    s_arm_finished = 0U;
    s_arm_state = ARM_STATE_IDLE;
}

void Arm_Process(void)
{
    Arm_UpdateAllServoMotion();

    if (s_arm_busy == 0U) return;

    switch (s_arm_state)
    {
    case ARM_STATE_PICK_ELBOW:
        if (Arm_IsServoMoving(ARM_SERVO_3_ELBOW) == 0U)
            Arm_StartPickShoulder();
        break;

    case ARM_STATE_PICK_SHOULDER:
        if (Arm_IsServoMoving(ARM_SERVO_2_SHOULDER) == 0U)
            Arm_StartPickWrist();
        break;

    case ARM_STATE_PICK_WRIST:
        if (Arm_IsServoMoving(ARM_SERVO_4_WRIST) == 0U)
        {
            Arm_PumpOn();
        }
        break;

    case ARM_STATE_PUMP_ON:
        if ((HAL_GetTick() - s_pump_on_tick) >= ARM_PUMP_ON_DELAY_MS)
        {
            if (Arm_StartVerticalLift() == 0U)
                Arm_StartRotateBase();
        }
        break;

    case ARM_STATE_LIFT_VERTICAL:
        if (Arm_UpdateVerticalLift() != 0U)
            Arm_StartRotateBase();
        break;

    case ARM_STATE_ROTATE_BASE:
        if (Arm_IsServoMoving(ARM_SERVO_1_BASE) == 0U)
        {
            Arm_StartPumpOff();
        }
        break;

    case ARM_STATE_PUMP_OFF:
        if ((HAL_GetTick() - s_valve_off_tick) >= ARM_VALVE_OFF_DELAY_MS)
        {
            Arm_StartScrewExtend();
        }
        break;

    case ARM_STATE_SCREW_EXTEND:
        if (Arm_ScrewUpdateMotion() != 0U)
        {
            HAL_Delay(ARM_SCREW_EXTEND_TIME_MS);
            Arm_StartScrewRetract();
        }
        break;

    case ARM_STATE_SCREW_RETRACT:
        if (Arm_ScrewUpdateMotion() != 0U)
        {
            Arm_StartHomeBase();
        }
        break;

    case ARM_STATE_HOME_BASE:
        if (Arm_IsServoMoving(ARM_SERVO_1_BASE) == 0U)
            Arm_StartHomeWrist();
        break;

    case ARM_STATE_HOME_WRIST:
        if (Arm_IsServoMoving(ARM_SERVO_4_WRIST) == 0U)
            Arm_StartHomeElbow();
        break;

    case ARM_STATE_HOME_ELBOW:
        if (Arm_IsServoMoving(ARM_SERVO_3_ELBOW) == 0U)
            Arm_StartHomeShoulder();
        break;

    case ARM_STATE_HOME_SHOULDER:
        if (Arm_IsServoMoving(ARM_SERVO_2_SHOULDER) == 0U)
            Arm_FinishSequence();
        break;

    case ARM_STATE_FINISH:
        s_arm_busy = 0U;
        s_arm_finished = 1U;
        s_arm_state = ARM_STATE_IDLE;
        Arm_PumpStop();
        break;

    default:
        break;
    }
}

/* ============================================================
 * 状态接口
 * ============================================================ */
uint8_t Arm_IsBusy(void)
{
    return s_arm_busy;
}

uint8_t Arm_TakeFinished(void)
{
    uint8_t finished = s_arm_finished;
    s_arm_finished = 0U;
    return finished;
}

Arm_State_t Arm_GetState(void)
{
    return s_arm_state;
}

uint32_t Arm_GetElapsedMs(void)
{
    return (s_arm_busy != 0U) ? (HAL_GetTick() - s_arm_start_tick) : 0U;
}

uint8_t Arm_IsBuildBusy(void)
{
    return s_build_busy;
}

uint8_t Arm_TakeBuildFinished(void)
{
    uint8_t finished = s_build_finished;
    s_build_finished = 0U;
    return finished;
}

Arm_BuildState_t Arm_GetBuildState(void)
{
    return s_build_state;
}

int32_t Arm_GetScrewPositionSteps(void)
{
    return s_screw_motion.current_position_steps;
}

/* ============================================================
 * 调试接口
 * ============================================================ */
uint8_t Arm_SetServoPulseUs(Arm_Servo_t servo, uint16_t pulse_us)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return 0U;
    s_lift.active = 0U;
    Arm_StopServoMotion(servo);
    Arm_WriteServoUs(servo, pulse_us);
    return 1U;
}

uint16_t Arm_GetServoPulseUs(Arm_Servo_t servo)
{
    uint32_t index = (uint32_t)servo;
    if (index >= (uint32_t)ARM_SERVO_COUNT) return 0U;
    return s_motion[index].current_us;
}

void Arm_SetPoseImmediate(uint16_t base_us, uint16_t shoulder_us, uint16_t elbow_us, uint16_t wrist_us)
{
    s_lift.active = 0U;
    Arm_WriteServoUs(ARM_SERVO_1_BASE, base_us);
    Arm_WriteServoUs(ARM_SERVO_2_SHOULDER, shoulder_us);
    Arm_WriteServoUs(ARM_SERVO_3_ELBOW, elbow_us);
    Arm_WriteServoUs(ARM_SERVO_4_WRIST, wrist_us);
}

/* ============================================================
 * 已弃用接口（保留兼容性）
 * ============================================================ */
void Arm_SetPickWristPosition(uint16_t position) { (void)position; }
uint16_t Arm_GetPickWristPosition(void) { return ARM_WRIST_PICK_US; }