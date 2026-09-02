#include "arm.h"
#include "BusServo.h"

#include "tim.h"
#include <math.h>
#include <stddef.h>

/*
 * ============================================================
 * 机械臂 PWM 硬件映射
 * ============================================================
 *
 * Servo 1：底座 Base      -> TIM5_CH2
 * Servo 2：大臂 Shoulder  -> TIM2_CH2//usart
 * Servo 3：小臂 Elbow     -> TIM2_CH3//usart
 * Servo 4：腕部 Wrist     -> TIM2_CH4//usart
 * Servo 5：夹爪 Gripper   -> TIM5_CH1
 * 开关舵机 Switch Servo -> TIM12_CH1
 *
 * TIM3_CH2 / CH3 / CH4 不在本文件中启动或修改。
 *
 * TIM2 / TIM3：
 *   PSC = 83
 *   ARR = 19999
 *
 * 当前 84 MHz APB1 Timer Clock 下：
 *   1 count = 1 us
 *   PWM period = 20 ms
 *   PWM frequency = 50 Hz
 */

// 舵机类型映射表：哪些是总线舵机
// 注意：Arm_Servo_t 的枚举顺序是 BASE=0, SHOULDER=1, ELBOW=2, WRIST=3, GRIPPER=4
const Arm_ServoType_t g_servo_type_map[ARM_SERVO_COUNT] = {
    SERVO_TYPE_PWM,   // 底座 → PWM
    SERVO_TYPE_BUS,   // 大臂 → 总线 (ID=1)
    SERVO_TYPE_BUS,   // 小臂 → 总线 (ID=2)
    SERVO_TYPE_BUS,   // 腕部 → 总线 (ID=3)
    SERVO_TYPE_PWM,   // 夹爪 → PWM
};

// 总线舵机ID映射
static const uint8_t g_bus_servo_id_map[ARM_SERVO_COUNT] = {
    0,   // 底座不是总线舵机，此项不使用
    1,   // 大臂 ID=1
    2,   // 小臂 ID=0（测试出厂默认 ID）
    3,   // 腕部 ID=3
    0,   // 夹爪不是总线舵机，此项不使用
};

// 总线舵机运动时间（每个舵机独立可调）
static const uint16_t g_bus_servo_time_ms[ARM_SERVO_COUNT] = {
    0,
    300,  // 大臂运动时间
    300,  // 小臂运动时间
    300,  // 腕部运动时间
    0,
};

/*
 * 总线舵机上电后需要一定时间完成内部初始化。
 * MCU 下载/复位后通常比舵机更早开始执行程序，如果 HOME 指令发送过早，
 * 舵机虽然已经供电，但可能还没有进入可接收串口指令的状态。
 */
#define ARM_BUS_SERVO_BOOT_DELAY_MS             1200U

/*
 * 连续给多个总线舵机发送命令时留一点间隔，避免某些总线转换板/舵机
 * 对背靠背数据帧处理不及时。
 */
#define ARM_BUS_SERVO_COMMAND_GAP_MS              30U

/*
 * ============================================================
 * 丝杆搭建电机：TIM10 PWM + DIR
 * ============================================================
 *
 * STEP：
 *   由 TIM10_CH1 硬件 PWM 输出，不再由 GPIO 软件翻转。
 *   STEP 引脚必须在 CubeMX 中配置为 TIM10_CH1 复用功能。
 *
 * DIR：
 *   仍使用普通 GPIO，当前为 PF9。
 *
 * 重要：
 * 当前没有原点限位开关，因此上电时默认机械机构已经实际位于 HOME。
 * 如果上电位置不确定，后续建议增加 HOME 限位开关做回零。
 */
#define ARM_SCREW_DIR_GPIO_PORT                  GPIOF
#define ARM_SCREW_DIR_GPIO_PIN                   GPIO_PIN_9

/*
 * 正、反方向使用两个明确的电平定义。
 * 不再通过“判断一个电平后再取反”来得到另一个方向。
 *
 * 如果实机方向相反，只交换下面两个定义即可。
 */
#define ARM_SCREW_DIR_POSITIVE_LEVEL             GPIO_PIN_SET
#define ARM_SCREW_DIR_NEGATIVE_LEVEL             GPIO_PIN_RESET

/*
 * ============================================================
 * 丝杆位置参数
 * ============================================================
 *
 * 单位：TIM10 输出的 STEP 脉冲数。
 */
#define ARM_SCREW_HOME_POSITION_STEPS            0L
#define ARM_SCREW_BUILD_POSITION_STEPS           130000L

/*
 * ============================================================
 * TIM10 STEP 频率
 * ============================================================
 *
 * 这里直接指定目标 STEP 频率。
 * Arm_Init() 会根据当前 APB2 定时器时钟自动计算 PSC / ARR / CCR1。
 *
 * 5000 Hz 表示每秒输出 5000 个 STEP 脉冲。
 * 后续想提速时优先改这个值。
 */
#define ARM_SCREW_STEP_FREQUENCY_HZ              5000U

/*
 * TIM10 计数器目标基准时钟。
 * 优先让 TIM10 计数频率接近 2 MHz，既有足够分辨率，
 * 又能覆盖常见步进驱动器的 STEP 频率范围。
 */
#define ARM_SCREW_TIM_COUNTER_TARGET_HZ           2000000U

/*
 * ============================================================
 * 开关舵机（TIM12_CH1）
 * ============================================================
 *
 * TIM12 配置为 1 count = 1 us，周期 20000 us（50 Hz）。
 * 如果实机方向相反，交换下面两个位置宏即可。
 */
#define ARM_SWITCH_SERVO_CLOSED_US              1900U
#define ARM_SWITCH_SERVO_OPEN_US                1620U

/*s
 * 普通 PWM 舵机没有到位反馈，用此时间估算动作完成。
 * 若实机还未到位丝杆就回程，增大此值。
 */
#define ARM_SWITCH_SERVO_MOVE_TIME_MS            700U

/*
 * ============================================================
 * PWM 安全范围
 * ============================================================
 */
#define ARM_PWM_MIN_US                         500U
#define ARM_PWM_MAX_US                         2500U

/*
 * ============================================================
 * 你当前已经调整过的位置参数
 * ============================================================
 *
 * 尽量原样保留。
 */

/* Servo 1：底座 Base */
#define ARM_S1_BASE_HOME_US                    1500U

/* Servo 2：大臂 Shoulder */
#define ARM_S2_SHOULDER_HOME_US                2200U
#define ARM_S2_SHOULDER_PICK_US                1400U
#define ARM_S2_SHOULDER_STORE_US               2200U

/* Servo 3：小臂 Elbow */
#define ARM_S3_ELBOW_HOME_US                   1000U
#define ARM_S3_ELBOW_PICK_US                   1450U
#define ARM_S3_ELBOW_STORE_US                  1200U

/* Servo 4：腕部 Wrist */
#define ARM_S4_WRIST_HOME_US                   1200U
#define ARM_S4_WRIST_PICK_DEFAULT_US           1200U
#define ARM_S4_WRIST_STORE_US                  1600U

/* Servo 5：夹爪 Gripper */
#define ARM_S5_GRIPPER_OPEN_US                 1200U
#define ARM_S5_GRIPPER_CLOSE_US                1600U

/*
 * ============================================================
 * 原有每一步运动时间
 * ============================================================
 */

/* 收到夹取命令后，先松开夹爪 */
#define ARM_PREPARE_OPEN_GRIPPER_MOVE_MS       500U

/* 夹取 */
#define ARM_PICK_ELBOW_MOVE_MS                 900U
#define ARM_PICK_SHOULDER_MOVE_MS              900U
#define ARM_PICK_WRIST_MOVE_MS                 700U
#define ARM_PICK_GRIPPER_MOVE_MS               500U

/* 存放 */
#define ARM_STORE_SHOULDER_MOVE_MS             4500U
#define ARM_STORE_ELBOW_MOVE_MS                1100U
#define ARM_STORE_WRIST_MOVE_MS                1000U
#define ARM_STORE_GRIPPER_MOVE_MS              500U

/* 归位 */
#define ARM_HOME_WRIST_MOVE_MS                 700U
#define ARM_HOME_ELBOW_MOVE_MS                 900U
#define ARM_HOME_SHOULDER_MOVE_MS              900U

/*
 * ============================================================
 * 新增：二维机械臂几何参数
 * ============================================================
 *
 * L1：大臂舵盘中心 -> 小臂舵盘中心 = 10.4 cm
 * L2：小臂舵盘中心 -> 腕部舵盘中心 = 7.3 cm
 *
 * 因为竖直抬升期间夹爪姿态保持不变，
 * 所以腕部之后任意固定点（包括方块）也会近似竖直平移同样距离。
 */
#define ARM_LINK1_LENGTH_CM                    10.4f
#define ARM_LINK2_LENGTH_CM                     7.3f

/* 要求的竖直上升距离 */
#define ARM_VERTICAL_LIFT_HEIGHT_CM             7.0f

/*
 * 竖直上升时间。
 * 想慢一点可以改成 2500~3000 ms。
 */
#define ARM_VERTICAL_LIFT_TIME_MS              2000U

/*
 * 为防止浮点误差导致 acos 输入略超 [-1,1]。
*/
#define ARM_IK_EPSILON                         0.0001f

#define ARM_PI_F                               3.14159265358979323846f
#define ARM_TWO_PI_F                           (2.0f * ARM_PI_F)

/*
 * ============================================================
 * PWM <-> 角度标定
 * ============================================================
 *
 * 坐标定义：
 *   X：地面水平方向，2400 us 对应的大臂方向为 0°
 *   Z：竖直向上为正
 *   逆时针为正方向
 *
 * ------------------------------------------------------------
 * 1. 大臂 Shoulder
 *
 * 2400 us -> 0°
 * 1800 us -> 90°
 *
 * 所以：
 * theta1_deg = (2400 - P1) * 0.15
 * ------------------------------------------------------------
 */
#define ARM_SHOULDER_PWM_AT_0_DEG             2400.0f
#define ARM_SHOULDER_DEG_PER_US                  0.15f

/*
 * ------------------------------------------------------------
 * 2. 小臂 Elbow（相对于大臂的关节角）
 *
 * 1200 us -> 相对大臂 0°
 * 2000 us -> 相对大臂 +90°
 *
 * 所以：
 * q2_deg = (P2 - 1200) * 0.1125
 * ------------------------------------------------------------
 */
#define ARM_ELBOW_PWM_AT_0_DEG                1200.0f
#define ARM_ELBOW_DEG_PER_US                     0.1125f

/*
 * ------------------------------------------------------------
 * 3. 腕部 Wrist（相对于小臂）
 *
 * 1000 us -> 相对小臂 0°
 * 1800 us -> 相对小臂 -90°（顺时针）
 *
 * 所以：
 * q3_deg = -(P3 - 1000) * 0.1125
 * ------------------------------------------------------------
 */
#define ARM_WRIST_PWM_AT_0_DEG                1000.0f
#define ARM_WRIST_DEG_PER_US                     0.1125f

/*
 * ============================================================
 * 单舵机平滑运动结构
 * ============================================================
 */
typedef struct
{
    uint16_t current_us;
    uint16_t start_us;
    uint16_t target_us;

    uint32_t start_tick;
    uint32_t duration_ms;

    uint8_t active;

} Arm_ServoMotion_t;

/*
 * ============================================================
 * 竖直抬升状态
 * ============================================================
 */
typedef struct
{
    uint8_t active;

    uint32_t start_tick;
    uint32_t duration_ms;

    /*
     * 腕部中心在抬升开始时的位置。
     * 抬升过程中 X 不变，只改变 Z。
     */
    float start_x_cm;
    float start_z_cm;

    /*
     * 实际目标抬升高度。
     * 正常情况下为 7.0 cm。
     */
    float lift_height_cm;

    /*
     * 夹爪相对地面的绝对角度。
     * 抬升过程中始终保持该值不变。
     */
    float tool_angle_rad;

    /*
     * 用来选择正确的肘部 IK 分支。
     */
    float start_elbow_relative_rad;

} Arm_VerticalLift_t;

/*
 * ============================================================
 * 丝杆 STEP 运动状态
 * ============================================================
 */
typedef struct
{
    int32_t current_position_steps;
    int32_t target_position_steps;
    int8_t step_direction;
    uint8_t running;

} Arm_ScrewMotion_t;

/*
 * ============================================================
 * 内部状态
 * ============================================================
 */

static volatile uint8_t s_arm_busy = 0U;
static volatile uint8_t s_arm_finished = 0U;

static Arm_State_t s_arm_state = ARM_STATE_IDLE;
static uint32_t s_arm_start_tick = 0U;

static uint16_t s_wrist_pick_position =
    ARM_S4_WRIST_PICK_DEFAULT_US;

static Arm_ServoMotion_t s_servo_motion[ARM_SERVO_COUNT];

static Arm_VerticalLift_t s_vertical_lift;

static volatile uint8_t s_build_busy = 0U;
static volatile uint8_t s_build_finished = 0U;

static Arm_BuildState_t s_build_state =
    ARM_BUILD_STATE_IDLE;

static uint32_t s_switch_servo_move_start_tick = 0U;

static volatile Arm_ScrewMotion_t s_screw_motion;

/*
 * ============================================================
 * 内部函数声明
 * ============================================================
 */

static uint16_t Arm_ClampPulseUs(uint16_t value);

//static void Arm_WriteServoUs(Arm_Servo_t servo,
  //                           uint16_t pulse_us);

static void Arm_WriteServoUs(Arm_Servo_t servo, uint16_t pulse_us);
static void Arm_UpdateServoMotion(Arm_Servo_t servo);
static void Arm_UpdateAllServoMotion(void);
static void Arm_StopServoMotion(Arm_Servo_t servo);
static uint8_t Arm_IsServoMoving(Arm_Servo_t servo);

/* 角度 / PWM */
static float Arm_DegToRad(float deg);
static float Arm_RadToDeg(float rad);

static float Arm_ShoulderPwmToAngleRad(uint16_t pwm_us);
static float Arm_ElbowPwmToRelativeRad(uint16_t pwm_us);
static float Arm_WristPwmToRelativeRad(uint16_t pwm_us);

static float Arm_ShoulderAngleRadToPwm(float angle_rad);
static float Arm_ElbowRelativeRadToPwm(float angle_rad);
static float Arm_WristRelativeRadToPwm(float angle_rad);

static float Arm_WrapAngleNear(float angle_rad,
                               float reference_rad);

static uint8_t Arm_FloatPwmToUint16(float pwm,
                                    uint16_t *out_pwm);

/* 二连杆运动学 */
static void Arm_ForwardKinematics(float shoulder_rad,
                                  float elbow_relative_rad,
                                  float *x_cm,
                                  float *z_cm);

static uint8_t Arm_SolveIK(float x_cm,
                           float z_cm,
                           float elbow_reference_rad,
                           float shoulder_reference_rad,
                           float *shoulder_rad,
                           float *elbow_relative_rad);

/* 竖直抬升 */
static uint8_t Arm_StartVerticalLift(void);
static uint8_t Arm_UpdateVerticalLift(void);
static uint8_t Arm_ApplyLiftPoint(float x_cm,
                                  float z_cm);

/* 丝杆搭建 */
static void Arm_ScrewGPIOInit(void);
static uint8_t Arm_ScrewConfigureTim10(uint32_t step_frequency_hz);
static void Arm_ScrewSetTarget(int32_t target_steps);
static uint8_t Arm_ScrewUpdateMotion(void);
static void Arm_ScrewStopMotion(void);
static void Arm_FinishBuildSequence(void);
static void Arm_SwitchServoWriteUs(uint16_t pulse_us);

/* 准备动作 */
static void Arm_StartPrepareOpenGripper(void);

/* 夹取 */
static void Arm_StartPickElbow(void);
static void Arm_StartPickShoulder(void);
static void Arm_StartPickWrist(void);
static void Arm_StartPickGripper(void);

/* 存放 */
static void Arm_StartStoreShoulder(void);
static void Arm_StartStoreElbow(void);
static void Arm_StartStoreWrist(void);
static void Arm_StartStoreRelease(void);

/* 归位 */
static void Arm_StartHomeWrist(void);
static void Arm_StartHomeElbow(void);
static void Arm_StartHomeShoulder(void);

static void Arm_FinishSequence(void);

/*
 * ============================================================
 * 初始化
 * ============================================================
 */

void Arm_Init(void)
{
    uint32_t i;

    // ============================================================
    // 1. 初始化丝杆 STEP / DIR GPIO
    // ============================================================
    Arm_ScrewGPIOInit();

    /*
     * 丝杆 STEP 由 TIM10_CH1 硬件 PWM 输出。
     * 平时关闭；运动时由 Arm_ScrewSetTarget() 启动。
     */
    HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);

    if (Arm_ScrewConfigureTim10(ARM_SCREW_STEP_FREQUENCY_HZ) == 0U)
    {
        Error_Handler();
    }

    /*
     * TIM12_CH1 是开关舵机。先写打开位比较值，再启动 PWM，
     * 避免刚启动时输出 0 us 的无效脉宽。
     */
    Arm_SwitchServoWriteUs(ARM_SWITCH_SERVO_OPEN_US);

    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    // ============================================================
    // 2. 初始化PWM舵机（底座、夹爪）
    // ============================================================
    for (i = 0U; i < (uint32_t)ARM_SERVO_COUNT; i++)
    {
        s_servo_motion[i].current_us = 1500U;
        s_servo_motion[i].start_us = 1500U;
        s_servo_motion[i].target_us = 1500U;
        s_servo_motion[i].start_tick = 0U;
        s_servo_motion[i].duration_ms = 0U;
        s_servo_motion[i].active = 0U;
    }

    // ============================================================
    // 3. 初始化总线舵机串口（USART2）
    // ============================================================
    // main.c 中 MX_USART2_UART_Init() 已经先执行。
    // 这里必须在第一次调用 Arm_WriteServoUs() 控制总线舵机之前
    // 把 huart2 交给 BusServo 模块。
    extern UART_HandleTypeDef huart2;
    BusServo_Init(&huart2);

    // ============================================================
    // 4. 启动 PWM 输出（底座、夹爪）
    // ============================================================
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);   // 底座
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);   // 夹爪

    /*
     * 大臂、小臂、腕部已经改为总线舵机，
     * TIM2_CH2 / CH3 / CH4 不再用于这三个关节。
     */

    // ============================================================
    // 5. 等待总线舵机完成自身上电初始化
    // ============================================================
    HAL_Delay(ARM_BUS_SERVO_BOOT_DELAY_MS);

    // ============================================================
    // 6. 设置上电 HOME 位置
    // ============================================================
    // PWM 舵机
    Arm_WriteServoUs(ARM_SERVO_1_BASE,
                     ARM_S1_BASE_HOME_US);

    Arm_WriteServoUs(ARM_SERVO_5_GRIPPER,
                     ARM_S5_GRIPPER_CLOSE_US);

    // 总线舵机 ID=1
    Arm_WriteServoUs(ARM_SERVO_2_SHOULDER,
                     ARM_S2_SHOULDER_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);

    // 总线舵机 ID=0 —— 测试小臂是否仍为出厂默认 ID，
    // 下载/复位时就是通过这里发送给小臂。
    Arm_WriteServoUs(ARM_SERVO_3_ELBOW,
                     ARM_S3_ELBOW_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);

    // 总线舵机 ID=3
    Arm_WriteServoUs(ARM_SERVO_4_WRIST,
                     ARM_S4_WRIST_HOME_US);
    HAL_Delay(ARM_BUS_SERVO_COMMAND_GAP_MS);

    // ============================================================
    // 7. 初始化状态标志
    // ============================================================
    s_arm_busy = 0U;
    s_arm_finished = 0U;
    s_arm_state = ARM_STATE_IDLE;
    s_arm_start_tick = 0U;

    s_wrist_pick_position = ARM_S4_WRIST_PICK_DEFAULT_US;

    s_vertical_lift.active = 0U;
    s_vertical_lift.start_tick = 0U;
    s_vertical_lift.duration_ms = 0U;
    s_vertical_lift.start_x_cm = 0.0f;
    s_vertical_lift.start_z_cm = 0.0f;
    s_vertical_lift.lift_height_cm = 0.0f;
    s_vertical_lift.tool_angle_rad = 0.0f;
    s_vertical_lift.start_elbow_relative_rad = 0.0f;

    // ============================================================
    // 8. 初始化丝杆搭建状态
    // ============================================================
    s_build_busy = 0U;
    s_build_finished = 0U;
    s_build_state = ARM_BUILD_STATE_IDLE;
    s_switch_servo_move_start_tick = 0U;

    s_screw_motion.current_position_steps = ARM_SCREW_HOME_POSITION_STEPS;
    s_screw_motion.target_position_steps = ARM_SCREW_HOME_POSITION_STEPS;
    s_screw_motion.step_direction = 1;
    s_screw_motion.running = 0U;

}

/*
 * ============================================================
 * 启动一次完整任务
 * ============================================================
 */

uint8_t Arm_StartGrab(void)
{
    if ((s_arm_busy != 0U) || (s_build_busy != 0U))
    {
        return 0U;
    }

    s_arm_start_tick = HAL_GetTick();

    s_arm_finished = 0U;
    s_arm_busy = 1U;

    /*
     * 收到夹取命令后：
     * 第一步先松开夹爪。
     */
    Arm_StartPrepareOpenGripper();

    return 1U;
}

/*
 * ============================================================
 * 启动一次丝杆搭建任务
 * ============================================================
 */
uint8_t Arm_StartBuild(void)
{
    if ((s_arm_busy != 0U) || (s_build_busy != 0U))
    {
        return 0U;
    }

    s_build_finished = 0U;
    s_build_busy = 1U;

    Arm_SwitchServoWriteUs(
        ARM_SWITCH_SERVO_CLOSED_US);

    s_switch_servo_move_start_tick = HAL_GetTick();

    s_build_state =
        ARM_BUILD_STATE_SWITCH_SERVO_CLOSING;

    return 1U;
}

/*
 * ============================================================
 * 主状态机
 * ============================================================
 */

void Arm_Process(void)
{
    /*
     * 普通 PWM 插值更新。
     * 竖直抬升阶段的三关节由 IK 单独实时写入。
     */
    Arm_UpdateAllServoMotion();

    /*
     * ========================================================
     * 丝杆搭建任务
     * ========================================================
     *
     * 与机械臂夹取互斥。
     * 全程只依赖主循环重复调用，不使用长时间阻塞延时。
     */
    if (s_build_busy != 0U)
    {
        switch (s_build_state)
        {
        case ARM_BUILD_STATE_SWITCH_SERVO_CLOSING:
        {
            if ((uint32_t)(
                    HAL_GetTick() -
                    s_switch_servo_move_start_tick) >=
                ARM_SWITCH_SERVO_MOVE_TIME_MS)
            {
                Arm_ScrewSetTarget(
                    ARM_SCREW_BUILD_POSITION_STEPS);

                s_build_state =
                    ARM_BUILD_STATE_MOVE_TO_BUILD;
            }

            break;
        }

        case ARM_BUILD_STATE_MOVE_TO_BUILD:
        {
            if (Arm_ScrewUpdateMotion() != 0U)
            {
                Arm_SwitchServoWriteUs(
                    ARM_SWITCH_SERVO_OPEN_US);

                s_switch_servo_move_start_tick = HAL_GetTick();

                s_build_state =
                    ARM_BUILD_STATE_SWITCH_SERVO_OPENING;
            }

            break;
        }

        case ARM_BUILD_STATE_SWITCH_SERVO_OPENING:
        {
            if ((uint32_t)(
                    HAL_GetTick() -
                    s_switch_servo_move_start_tick) >=
                ARM_SWITCH_SERVO_MOVE_TIME_MS)
            {
                Arm_ScrewSetTarget(
                    ARM_SCREW_HOME_POSITION_STEPS);

                s_build_state =
                    ARM_BUILD_STATE_RETURN_HOME;
            }

            break;
        }

        case ARM_BUILD_STATE_RETURN_HOME:
        {
            if (Arm_ScrewUpdateMotion() != 0U)
            {
                Arm_FinishBuildSequence();
            }

            break;
        }

        case ARM_BUILD_STATE_IDLE:
        default:
        {
            Arm_ScrewStopMotion();

            s_build_busy = 0U;
            s_build_state =
                ARM_BUILD_STATE_IDLE;

            break;
        }
        }

        return;
    }

    if (s_arm_busy == 0U)
    {
        return;
    }

    switch (s_arm_state)
    {
    /*
     * ========================================================
     * 准备夹取
     * ========================================================
     */

    case ARM_STATE_PREPARE_OPEN_GRIPPER:
    {
        if (Arm_IsServoMoving(ARM_SERVO_5_GRIPPER) == 0U)
        {
            Arm_StartPickElbow();
        }

        break;
    }

    /*
     * ========================================================
     * 夹取
     * ========================================================
     */

    case ARM_STATE_PICK_ELBOW:
    {
        if (Arm_IsServoMoving(ARM_SERVO_3_ELBOW) == 0U)
        {
            Arm_StartPickShoulder();
        }

        break;
    }

    case ARM_STATE_PICK_SHOULDER:
    {
        if (Arm_IsServoMoving(ARM_SERVO_2_SHOULDER) == 0U)
        {
            Arm_StartPickWrist();
        }

        break;
    }

    case ARM_STATE_PICK_WRIST:
    {
        if (Arm_IsServoMoving(ARM_SERVO_4_WRIST) == 0U)
        {
            Arm_StartPickGripper();
        }

        break;
    }

    case ARM_STATE_PICK_GRIPPER:
    {
        if (Arm_IsServoMoving(ARM_SERVO_5_GRIPPER) == 0U)
        {
            /*
             * 方块已经夹紧。
             *
             * 先执行竖直直线抬升，再进入原来的存放流程。
             * 如果当前姿态下目标抬升高度不可达，则跳过抬升，
             * 直接从存放第 1 步（Elbow -> STORE）继续。
             */
            if (Arm_StartVerticalLift() == 0U)
            {
                Arm_StartStoreElbow();
            }
        }

        break;
    }

    /*
     * ========================================================
     * 竖直抬升
     * ========================================================
     */
    case ARM_STATE_LIFT_VERTICAL:
    {
        if (Arm_UpdateVerticalLift() != 0U)
        {
            /*
             * 抬升完成后，从完整存放流程的第 1 步开始：
             * Elbow -> Wrist -> Shoulder -> Release。
             */
            Arm_StartStoreElbow();
        }

        break;
    }

    /*
     * ========================================================
     * 存放
     * ========================================================
     */

    case ARM_STATE_STORE_SHOULDER:
    {
        /*
         * 存放第 3 步：
         * 大臂动作完成后，最后松开夹爪。
         */
        if (Arm_IsServoMoving(ARM_SERVO_2_SHOULDER) == 0U)
        {
            Arm_StartStoreRelease();
        }

        break;
    }

    case ARM_STATE_STORE_ELBOW:
    {
        /*
         * 存放第 1 步：
         * 小臂动作完成后，再动腕部。
         */
        if (Arm_IsServoMoving(ARM_SERVO_3_ELBOW) == 0U)
        {
            Arm_StartStoreWrist();
        }

        break;
    }

    case ARM_STATE_STORE_WRIST:
    {
        /*
         * 存放第 2 步：
         * 腕部动作完成后，再动大臂。
         */
        if (Arm_IsServoMoving(ARM_SERVO_4_WRIST) == 0U)
        {
            Arm_StartStoreShoulder();
        }

        break;
    }

    case ARM_STATE_STORE_RELEASE:
    {
        /*
         * 存放第 4 步：
         * 夹爪松开后进入归位。
         */
        if (Arm_IsServoMoving(ARM_SERVO_5_GRIPPER) == 0U)
        {
            Arm_StartHomeWrist();
        }

        break;
    }

    /*
     * ========================================================
     * 归位
     * ========================================================
     */

    case ARM_STATE_HOME_WRIST:
    {
        if (Arm_IsServoMoving(ARM_SERVO_4_WRIST) == 0U)
        {
            Arm_StartHomeElbow();
        }

        break;
    }

    case ARM_STATE_HOME_ELBOW:
    {
        if (Arm_IsServoMoving(ARM_SERVO_3_ELBOW) == 0U)
        {
            Arm_StartHomeShoulder();
        }

        break;
    }

    case ARM_STATE_HOME_SHOULDER:
    {
        if (Arm_IsServoMoving(ARM_SERVO_2_SHOULDER) == 0U)
        {
            Arm_FinishSequence();
        }

        break;
    }

    case ARM_STATE_IDLE:
    default:
    {
        s_arm_busy = 0U;
        s_arm_state = ARM_STATE_IDLE;

        break;
    }
    }
}

/*
 * ============================================================
 * 停止 / 状态接口
 * ============================================================
 */

void Arm_Stop(void)
{
    uint32_t i;

    Arm_UpdateAllServoMotion();

    /*
     * 停止 IK 抬升。
     */
    s_vertical_lift.active = 0U;

    for (i = 0U; i < (uint32_t)ARM_SERVO_COUNT; i++)
    {
        Arm_StopServoMotion((Arm_Servo_t)i);
    }

    /*
     * 同时停止丝杆，当前位置保留为已经走到的软件步数。
     */
    Arm_ScrewStopMotion();

    s_arm_busy = 0U;
    s_arm_finished = 0U;

    s_build_busy = 0U;
    s_build_finished = 0U;

    s_arm_state = ARM_STATE_IDLE;
    s_build_state = ARM_BUILD_STATE_IDLE;
}

uint8_t Arm_IsBusy(void)
{
    return s_arm_busy;
}

uint8_t Arm_TakeFinished(void)
{
    uint8_t finished;

    finished = s_arm_finished;
    s_arm_finished = 0U;

    return finished;
}

Arm_State_t Arm_GetState(void)
{
    return s_arm_state;
}

uint32_t Arm_GetElapsedMs(void)
{
    if (s_arm_busy == 0U)
    {
        return 0U;
    }

    return (uint32_t)(
        HAL_GetTick() - s_arm_start_tick);
}

uint8_t Arm_IsBuildBusy(void)
{
    return s_build_busy;
}

uint8_t Arm_TakeBuildFinished(void)
{
    uint8_t finished;

    finished = s_build_finished;
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

/*
 * ============================================================
 * 腕部抓取参数
 * ============================================================
 */

void Arm_SetPickWristPosition(uint16_t position)
{
    s_wrist_pick_position =
        Arm_ClampPulseUs(position);
}

uint16_t Arm_GetPickWristPosition(void)
{
    return s_wrist_pick_position;
}

/*
 * ============================================================
 * 对外调试接口
 * ============================================================
 */

uint8_t Arm_SetServoPulseUs(Arm_Servo_t servo,
                            uint16_t pulse_us)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    /*
     * 手动调舵机时取消自动 IK 抬升。
     */
    s_vertical_lift.active = 0U;

    Arm_StopServoMotion(servo);

    Arm_WriteServoUs(
        servo,
        pulse_us);

    return 1U;
}

uint16_t Arm_GetServoPulseUs(Arm_Servo_t servo)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    return s_servo_motion[index].current_us;
}

/*uint8_t Arm_MoveServoTo(Arm_Servo_t servo,
                        uint16_t target_us,
                        uint32_t duration_ms)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    Arm_UpdateServoMotion(servo);

    target_us =
        Arm_ClampPulseUs(target_us);

    if (duration_ms == 0U)
    {
        Arm_StopServoMotion(servo);

        Arm_WriteServoUs(
            servo,
            target_us);

        return 1U;
    }

    s_servo_motion[index].start_us =
        s_servo_motion[index].current_us;

    s_servo_motion[index].target_us =
        target_us;

    s_servo_motion[index].start_tick =
        HAL_GetTick();

    s_servo_motion[index].duration_ms =
        duration_ms;

    s_servo_motion[index].active =
        1U;

    return 1U;
}*/

/*
 * ============================================================
 * 启动单舵机运动
 * ============================================================
 */
uint8_t Arm_MoveServoTo(Arm_Servo_t servo, uint16_t target_us, uint32_t duration_ms)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    target_us = Arm_ClampPulseUs(target_us);

    if (duration_ms == 0U)
    {
        Arm_StopServoMotion(servo);
        Arm_WriteServoUs(servo, target_us);
        return 1U;
    }

    // ============================================================
    // 总线舵机：直接发串口指令，舵机内部平滑运动
    // ============================================================
    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        uint8_t bus_id = g_bus_servo_id_map[servo];

        /*
         * ID=0 是众灵总线舵机的出厂默认 ID，属于合法地址。
         * 这里直接发送，不再使用 “bus_id != 0” 判断。
         */
        BusServo_SetAngle(bus_id, target_us, duration_ms);

        s_servo_motion[index].start_us    = s_servo_motion[index].current_us;
        s_servo_motion[index].target_us   = target_us;
        s_servo_motion[index].start_tick  = HAL_GetTick();
        s_servo_motion[index].duration_ms = duration_ms;
        s_servo_motion[index].active      = 1U;
        s_servo_motion[index].current_us  = target_us;

        return 1U;
    }

    // ============================================================
    // PWM舵机：原来的软件插值逻辑
    // ============================================================
    s_servo_motion[index].start_us = s_servo_motion[index].current_us;
    s_servo_motion[index].target_us = target_us;
    s_servo_motion[index].start_tick = HAL_GetTick();
    s_servo_motion[index].duration_ms = duration_ms;
    s_servo_motion[index].active = 1U;

    return 1U;
}

void Arm_SetPoseImmediate(uint16_t base_us,
                          uint16_t shoulder_us,
                          uint16_t elbow_us,
                          uint16_t wrist_us,
                          uint16_t gripper_us)
{
    s_vertical_lift.active = 0U;

    Arm_SetServoPulseUs(
        ARM_SERVO_1_BASE,
        base_us);

    Arm_SetServoPulseUs(
        ARM_SERVO_2_SHOULDER,
        shoulder_us);

    Arm_SetServoPulseUs(
        ARM_SERVO_3_ELBOW,
        elbow_us);

    Arm_SetServoPulseUs(
        ARM_SERVO_4_WRIST,
        wrist_us);

    Arm_SetServoPulseUs(
        ARM_SERVO_5_GRIPPER,
        gripper_us);
}

/*
 * ============================================================
 * 丝杆 TIM10 PWM / DIR 底层
 * ============================================================
 */

static void Arm_ScrewGPIOInit(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();

    /*
     * STEP 不再在这里配置为普通 GPIO。
     * STEP 由 CubeMX 生成的 TIM10_CH1 复用功能负责。
     *
     * 这里只配置 DIR。
     */
    gpio_init.Pin = ARM_SCREW_DIR_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        ARM_SCREW_DIR_GPIO_PORT,
        &gpio_init);

    HAL_GPIO_WritePin(
        ARM_SCREW_DIR_GPIO_PORT,
        ARM_SCREW_DIR_GPIO_PIN,
        ARM_SCREW_DIR_POSITIVE_LEVEL);
}

/*
 * 根据当前 APB2 时钟自动配置 TIM10。
 *
 * STM32F4 中，如果 APB2 分频不为 1，则 APB2 上的定时器时钟
 * 等于 PCLK2 的 2 倍。
 *
 * 返回：
 *   1 = 配置成功
 *   0 = 参数非法 / 无法得到有效 PSC、ARR
 */
static uint8_t Arm_ScrewConfigureTim10(uint32_t step_frequency_hz)
{
    RCC_ClkInitTypeDef clock_config;
    uint32_t flash_latency;
    uint32_t pclk2_hz;
    uint32_t tim10_clock_hz;
    uint32_t prescaler_div;
    uint32_t timer_counter_hz;
    uint32_t period_counts;
    uint32_t arr;
    uint32_t pulse;

    if (step_frequency_hz == 0U)
    {
        return 0U;
    }

    HAL_RCC_GetClockConfig(
        &clock_config,
        &flash_latency);

    pclk2_hz = HAL_RCC_GetPCLK2Freq();

    if (clock_config.APB2CLKDivider == RCC_HCLK_DIV1)
    {
        tim10_clock_hz = pclk2_hz;
    }
    else
    {
        tim10_clock_hz = pclk2_hz * 2U;
    }

    /*
     * 让 TIM10 计数时钟尽量接近 2 MHz。
     * prescaler_div 对应 (PSC + 1)。
     */
    prescaler_div =
        (tim10_clock_hz +
         (ARM_SCREW_TIM_COUNTER_TARGET_HZ / 2U)) /
        ARM_SCREW_TIM_COUNTER_TARGET_HZ;

    if (prescaler_div == 0U)
    {
        prescaler_div = 1U;
    }

    if (prescaler_div > 65536U)
    {
        return 0U;
    }

    timer_counter_hz =
        tim10_clock_hz / prescaler_div;

    period_counts =
        (timer_counter_hz +
         (step_frequency_hz / 2U)) /
        step_frequency_hz;

    /*
     * PWM 至少需要两个计数：
     * 一个高电平区间 + 一个低电平区间。
     */
    if (period_counts < 2U)
    {
        period_counts = 2U;
    }

    if (period_counts > 65536U)
    {
        return 0U;
    }

    arr = period_counts - 1U;

    /*
     * 约 50% 占空比。
     */
    pulse = period_counts / 2U;

    if (pulse == 0U)
    {
        pulse = 1U;
    }

    __HAL_TIM_DISABLE(&htim10);

    __HAL_TIM_SET_PRESCALER(
        &htim10,
        prescaler_div - 1U);

    __HAL_TIM_SET_AUTORELOAD(
        &htim10,
        arr);

    __HAL_TIM_SET_COMPARE(
        &htim10,
        TIM_CHANNEL_1,
        pulse);

    __HAL_TIM_SET_COUNTER(
        &htim10,
        0U);

    __HAL_TIM_CLEAR_FLAG(
        &htim10,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    /*
     * PSC 的新值需要通过更新事件装载。
     * 此时 PWM 通道尚未启动，因此不会额外输出 STEP。
     */
    htim10.Instance->EGR = TIM_EGR_UG;

    __HAL_TIM_CLEAR_FLAG(
        &htim10,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    return 1U;
}

static void Arm_ScrewSetTarget(int32_t target_steps)
{
    int32_t current_steps;
    uint32_t first_counter;

    /*
     * 先停止上一段 PWM，保证改变 DIR 时 STEP 不在输出。
     */
    HAL_TIM_PWM_Stop_IT(
        &htim10,
        TIM_CHANNEL_1);

    s_screw_motion.running = 0U;

    current_steps =
        s_screw_motion.current_position_steps;

    s_screw_motion.target_position_steps =
        target_steps;

    if (target_steps == current_steps)
    {
        return;
    }

    if (target_steps > current_steps)
    {
        s_screw_motion.step_direction = 1;

        HAL_GPIO_WritePin(
            ARM_SCREW_DIR_GPIO_PORT,
            ARM_SCREW_DIR_GPIO_PIN,
            ARM_SCREW_DIR_POSITIVE_LEVEL);
    }
    else
    {
        s_screw_motion.step_direction = -1;

        HAL_GPIO_WritePin(
            ARM_SCREW_DIR_GPIO_PORT,
            ARM_SCREW_DIR_GPIO_PIN,
            ARM_SCREW_DIR_NEGATIVE_LEVEL);
    }

    /*
     * 从 CCR1 位置启动：
     * PWM Mode 1 下此时输出处于低电平区间，
     * 会先等待大约半个 STEP 周期再产生第一个上升沿。
     *
     * 这样给 DIR 留出建立时间，避免改变方向后立刻出现 STEP 上升沿。
     */
    first_counter =
        __HAL_TIM_GET_COMPARE(
            &htim10,
            TIM_CHANNEL_1);

    __HAL_TIM_SET_COUNTER(
        &htim10,
        first_counter);

    __HAL_TIM_CLEAR_FLAG(
        &htim10,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE);

    s_screw_motion.running = 1U;

    /*
     * TIM10_CH1 自动输出 STEP；
     * CC1 中断每个 PWM 周期只计一次。
     */
    if (HAL_TIM_PWM_Start_IT(
            &htim10,
            TIM_CHANNEL_1) != HAL_OK)
    {
        s_screw_motion.running = 0U;

        s_screw_motion.target_position_steps =
            s_screw_motion.current_position_steps;
    }
}

static uint8_t Arm_ScrewUpdateMotion(void)
{
    if ((s_screw_motion.running == 0U) &&
        (s_screw_motion.current_position_steps ==
         s_screw_motion.target_position_steps))
    {
        return 1U;
    }

    return 0U;
}

static void Arm_ScrewStopMotion(void)
{
    HAL_TIM_PWM_Stop_IT(
        &htim10,
        TIM_CHANNEL_1);

    s_screw_motion.running = 0U;

    /*
     * 手动 STOP 时取消尚未完成的目标。
     * 已经输出过的 STEP 数保留为当前位置。
     */
    s_screw_motion.target_position_steps =
        s_screw_motion.current_position_steps;
}

static void Arm_FinishBuildSequence(void)
{
    Arm_ScrewStopMotion();

    /*
     * 正常结束时理论上已经回到 HOME。
     * 再明确写回 HOME，便于调试查看。
     */
    s_screw_motion.current_position_steps =
        ARM_SCREW_HOME_POSITION_STEPS;

    s_screw_motion.target_position_steps =
        ARM_SCREW_HOME_POSITION_STEPS;

    s_build_busy = 0U;
    s_build_finished = 1U;

    s_build_state =
        ARM_BUILD_STATE_IDLE;
}

/*
 * TIM12 已由 CubeMX 配置为 1 us/count，因此 CCR1 值就是脉宽 us。
 */
static void Arm_SwitchServoWriteUs(uint16_t pulse_us)
{
    pulse_us = Arm_ClampPulseUs(pulse_us);

    __HAL_TIM_SET_COMPARE(
        &htim12,
        TIM_CHANNEL_1,
        pulse_us);
}

/*
 * ============================================================
 * PWM 底层
 * ============================================================
 */

static uint16_t Arm_ClampPulseUs(uint16_t value)
{
    if (value < ARM_PWM_MIN_US)
    {
        return ARM_PWM_MIN_US;
    }

    if (value > ARM_PWM_MAX_US)
    {
        return ARM_PWM_MAX_US;
    }

    return value;
}

static void Arm_WriteServoUs(Arm_Servo_t servo,
                             uint16_t pulse_us)
{
     uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return;
    }

    pulse_us = Arm_ClampPulseUs(pulse_us);

    // ============================================================
    // 关键改动：判断是总线舵机还是PWM舵机
    // ============================================================
    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        // 总线舵机：通过串口发指令
        uint8_t bus_id = g_bus_servo_id_map[servo];
        uint16_t move_time = g_bus_servo_time_ms[servo];
        
        /*
         * 众灵总线舵机合法 ID 范围为 0~254。
         * ID=0 是出厂默认 ID，因此不能把 0 当作“无效 ID”。
         */
        if (move_time > 0U)
        {
            BusServo_SetAngle(bus_id, pulse_us, move_time);
        }
        else
        {
            BusServo_SetAngle(bus_id, pulse_us, 500U);
        }
        
        // 更新当前值（用于状态跟踪）
        s_servo_motion[index].current_us = pulse_us;
        return;  // 总线舵机不需要写PWM寄存器
    }

    // ============================================================
    // PWM舵机：原来的逻辑不变
    // ============================================================
    switch (servo)
    {
    case ARM_SERVO_1_BASE:
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse_us);
        break;
    }
    // ... 其他PWM舵机保持不变 ...
    case ARM_SERVO_5_GRIPPER:
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse_us);
        break;
    }
    default:
    {
        return;
    }
    }

    s_servo_motion[index].current_us = pulse_us;
}


/*
 * ============================================================
 * 普通单舵机线性插值
 * ============================================================
 */

/*
 * ============================================================
 * 单舵机插值更新
 * ============================================================
 */
static void Arm_UpdateServoMotion(Arm_Servo_t servo)
{
    uint32_t index;
    uint32_t elapsed;
    int32_t start;
    int32_t target;
    int32_t delta;
    int32_t value;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return;
    }

    // ============================================================
    // 总线舵机：不需要软件插值，直接跳过
    // ============================================================
    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        return;  // 总线舵机内部自己平滑运动
    }

    // ============================================================
    // PWM舵机：原来的软件插值逻辑
    // ============================================================
    if (s_servo_motion[index].active == 0U)
    {
        return;
    }

    elapsed = (uint32_t)(HAL_GetTick() - s_servo_motion[index].start_tick);

    if (elapsed >= s_servo_motion[index].duration_ms)
    {
        Arm_WriteServoUs(servo, s_servo_motion[index].target_us);
        s_servo_motion[index].active = 0U;
        return;
    }

    start  = (int32_t)s_servo_motion[index].start_us;
    target = (int32_t)s_servo_motion[index].target_us;
    delta  = target - start;

    value = start + (int32_t)((delta * (int32_t)elapsed) / (int32_t)s_servo_motion[index].duration_ms);

    Arm_WriteServoUs(servo, (uint16_t)value);
}

static void Arm_UpdateAllServoMotion(void)
{
    uint32_t i;

    for (i = 0U;
         i < (uint32_t)ARM_SERVO_COUNT;
         i++)
    {
        Arm_UpdateServoMotion(
            (Arm_Servo_t)i);
    }
}

/*static void Arm_StopServoMotion(Arm_Servo_t servo)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return;
    }

    s_servo_motion[index].active = 0U;
}*/

static void Arm_StopServoMotion(Arm_Servo_t servo)
{
    uint32_t index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return;
    }

    // ============================================================
    // 总线舵机：发送停止指令（可选），清除active标志
    // ============================================================
    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        // 如果需要紧急停止总线舵机，可以发暂停指令
        // BusServo_Pause(g_bus_servo_id_map[servo]);
        s_servo_motion[index].active = 0U;
        return;
    }

    // ============================================================
    // PWM舵机：原来的逻辑
    // ============================================================
    s_servo_motion[index].active = 0U;
}

/*static uint8_t Arm_IsServoMoving(Arm_Servo_t servo)
{
    uint32_t index;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    return s_servo_motion[index].active;
}*/

/*
 * ============================================================
 * 判断舵机是否在运动中
 * ============================================================
 */
static uint8_t Arm_IsServoMoving(Arm_Servo_t servo)
{
    uint32_t index;
    uint32_t elapsed;

    index = (uint32_t)servo;

    if (index >= (uint32_t)ARM_SERVO_COUNT)
    {
        return 0U;
    }

    // ============================================================
    // 总线舵机：用超时判断
    // ============================================================
    if (g_servo_type_map[servo] == SERVO_TYPE_BUS)
    {
        if (s_servo_motion[index].active == 0U)
        {
            return 0U;
        }

        elapsed = HAL_GetTick() - s_servo_motion[index].start_tick;

        // 运动时间 + 100ms 余量
        if (elapsed >= (s_servo_motion[index].duration_ms + 100U))
        {
            s_servo_motion[index].active = 0U;
            return 0U;
        }

        return 1U;
    }

    // ============================================================
    // PWM舵机：原来的逻辑
    // ============================================================
    return s_servo_motion[index].active;
}
/*
 * ============================================================
 * PWM / 角度转换
 * ============================================================
 */

static float Arm_DegToRad(float deg)
{
    return deg * ARM_PI_F / 180.0f;
}

static float Arm_RadToDeg(float rad)
{
    return rad * 180.0f / ARM_PI_F;
}

static float Arm_ShoulderPwmToAngleRad(uint16_t pwm_us)
{
    float angle_deg;

    angle_deg =
        (ARM_SHOULDER_PWM_AT_0_DEG -
         (float)pwm_us) *
        ARM_SHOULDER_DEG_PER_US;

    return Arm_DegToRad(angle_deg);
}

static float Arm_ElbowPwmToRelativeRad(uint16_t pwm_us)
{
    float angle_deg;

    angle_deg =
        ((float)pwm_us -
         ARM_ELBOW_PWM_AT_0_DEG) *
        ARM_ELBOW_DEG_PER_US;

    return Arm_DegToRad(angle_deg);
}

static float Arm_WristPwmToRelativeRad(uint16_t pwm_us)
{
    float angle_deg;

    angle_deg =
        -((float)pwm_us -
          ARM_WRIST_PWM_AT_0_DEG) *
        ARM_WRIST_DEG_PER_US;

    return Arm_DegToRad(angle_deg);
}

static float Arm_ShoulderAngleRadToPwm(float angle_rad)
{
    float angle_deg;

    angle_deg =
        Arm_RadToDeg(angle_rad);

    return
        ARM_SHOULDER_PWM_AT_0_DEG -
        angle_deg /
        ARM_SHOULDER_DEG_PER_US;
}

static float Arm_ElbowRelativeRadToPwm(float angle_rad)
{
    float angle_deg;

    angle_deg =
        Arm_RadToDeg(angle_rad);

    return
        ARM_ELBOW_PWM_AT_0_DEG +
        angle_deg /
        ARM_ELBOW_DEG_PER_US;
}

static float Arm_WristRelativeRadToPwm(float angle_rad)
{
    float angle_deg;

    angle_deg =
        Arm_RadToDeg(angle_rad);

    return
        ARM_WRIST_PWM_AT_0_DEG -
        angle_deg /
        ARM_WRIST_DEG_PER_US;
}

/*
 * 把 atan2 得到的 [-pi, pi] 角度，
 * 转换到与 reference 最接近的等效角度。
 *
 * 例如：
 * IK 可能算出 -180°，
 * 但当前机械臂实际是 +180°，
 * 这两个方向相同。
 */
static float Arm_WrapAngleNear(float angle_rad,
                               float reference_rad)
{
    while ((angle_rad - reference_rad) > ARM_PI_F)
    {
        angle_rad -= ARM_TWO_PI_F;
    }

    while ((angle_rad - reference_rad) < -ARM_PI_F)
    {
        angle_rad += ARM_TWO_PI_F;
    }

    return angle_rad;
}

static uint8_t Arm_FloatPwmToUint16(float pwm,
                                    uint16_t *out_pwm)
{
    if (out_pwm == NULL)
    {
        return 0U;
    }

    /*
     * 不在这里强行夹到 500/2500，
     * 因为强行夹值会破坏“竖直轨迹 + 姿态保持”。
     *
     * 如果超出允许范围，就认为该轨迹点不可用。
     */
    if ((pwm < (float)ARM_PWM_MIN_US) ||
        (pwm > (float)ARM_PWM_MAX_US))
    {
        return 0U;
    }

    *out_pwm =
        (uint16_t)(pwm + 0.5f);

    return 1U;
}

/*
 * ============================================================
 * 二连杆正运动学
 * ============================================================
 *
 * shoulder_rad：
 *   大臂相对地面的绝对角
 *
 * elbow_relative_rad：
 *   小臂相对大臂的角度
 *
 * 小臂绝对角：
 *   shoulder + elbow_relative
 */
static void Arm_ForwardKinematics(float shoulder_rad,
                                  float elbow_relative_rad,
                                  float *x_cm,
                                  float *z_cm)
{
    float forearm_abs_rad;

    forearm_abs_rad =
        shoulder_rad +
        elbow_relative_rad;

    if (x_cm != NULL)
    {
        *x_cm =
            ARM_LINK1_LENGTH_CM * cosf(shoulder_rad) +
            ARM_LINK2_LENGTH_CM * cosf(forearm_abs_rad);
    }

    if (z_cm != NULL)
    {
        *z_cm =
            ARM_LINK1_LENGTH_CM * sinf(shoulder_rad) +
            ARM_LINK2_LENGTH_CM * sinf(forearm_abs_rad);
    }
}

/*
 * ============================================================
 * 二连杆逆运动学
 * ============================================================
 */
static uint8_t Arm_SolveIK(float x_cm,
                           float z_cm,
                           float elbow_reference_rad,
                           float shoulder_reference_rad,
                           float *shoulder_rad,
                           float *elbow_relative_rad)
{
    float l1;
    float l2;
    float r2;
    float cos_q2;
    float q2;
    float theta1;

    if ((shoulder_rad == NULL) ||
        (elbow_relative_rad == NULL))
    {
        return 0U;
    }

    l1 = ARM_LINK1_LENGTH_CM;
    l2 = ARM_LINK2_LENGTH_CM;

    r2 =
        x_cm * x_cm +
        z_cm * z_cm;

    cos_q2 =
        (r2 - l1 * l1 - l2 * l2) /
        (2.0f * l1 * l2);

    /*
     * 明显超出工作空间。
     */
    if ((cos_q2 > (1.0f + ARM_IK_EPSILON)) ||
        (cos_q2 < (-1.0f - ARM_IK_EPSILON)))
    {
        return 0U;
    }

    /*
     * 只处理浮点计算产生的极小越界。
     */
    if (cos_q2 > 1.0f)
    {
        cos_q2 = 1.0f;
    }
    else if (cos_q2 < -1.0f)
    {
        cos_q2 = -1.0f;
    }

    q2 = acosf(cos_q2);

    /*
     * 保持与当前肘关节同一 IK 分支。
     */
    if (elbow_reference_rad < 0.0f)
    {
        q2 = -q2;
    }

    theta1 =
        atan2f(z_cm, x_cm) -
        atan2f(
            l2 * sinf(q2),
            l1 + l2 * cosf(q2));

    /*
     * 例如当前大臂是 +180°，
     * atan2 可能给出 -180°，
     * 这里统一到最接近当前角度的表达。
     */
    theta1 =
        Arm_WrapAngleNear(
            theta1,
            shoulder_reference_rad);

    *shoulder_rad = theta1;
    *elbow_relative_rad = q2;

    return 1U;
}

/*
 * ============================================================
 * 竖直抬升：开始
 * ============================================================
 */
static uint8_t Arm_StartVerticalLift(void)
{
    uint16_t shoulder_pwm;
    uint16_t elbow_pwm;
    uint16_t wrist_pwm;

    float shoulder_rad;
    float elbow_relative_rad;
    float wrist_relative_rad;

    float x_cm;
    float z_cm;

    float target_z_cm;
    float test_shoulder_rad;
    float test_elbow_rad;

    shoulder_pwm =
        s_servo_motion[ARM_SERVO_2_SHOULDER].current_us;

    elbow_pwm =
        s_servo_motion[ARM_SERVO_3_ELBOW].current_us;

    wrist_pwm =
        s_servo_motion[ARM_SERVO_4_WRIST].current_us;

    shoulder_rad =
        Arm_ShoulderPwmToAngleRad(shoulder_pwm);

    elbow_relative_rad =
        Arm_ElbowPwmToRelativeRad(elbow_pwm);

    wrist_relative_rad =
        Arm_WristPwmToRelativeRad(wrist_pwm);

    /*
     * 当前腕部中心坐标。
     */
    Arm_ForwardKinematics(
        shoulder_rad,
        elbow_relative_rad,
        &x_cm,
        &z_cm);

    target_z_cm =
        z_cm +
        ARM_VERTICAL_LIFT_HEIGHT_CM;

    /*
     * 先验证最终 7 cm 目标是否可达。
     */
    if (Arm_SolveIK(
            x_cm,
            target_z_cm,
            elbow_relative_rad,
            shoulder_rad,
            &test_shoulder_rad,
            &test_elbow_rad) == 0U)
    {
        return 0U;
    }

    /*
     * 在抬升前停止 Shoulder / Elbow / Wrist
     * 可能残留的普通插值。
     */
    Arm_StopServoMotion(ARM_SERVO_2_SHOULDER);
    Arm_StopServoMotion(ARM_SERVO_3_ELBOW);
    Arm_StopServoMotion(ARM_SERVO_4_WRIST);

    s_vertical_lift.active = 1U;

    s_vertical_lift.start_tick =
        HAL_GetTick();

    s_vertical_lift.duration_ms =
        ARM_VERTICAL_LIFT_TIME_MS;

    s_vertical_lift.start_x_cm =
        x_cm;

    s_vertical_lift.start_z_cm =
        z_cm;

    s_vertical_lift.lift_height_cm =
        ARM_VERTICAL_LIFT_HEIGHT_CM;

    /*
     * 工具绝对角：
     *
     * tool =
     * shoulder +
     * elbow_relative +
     * wrist_relative
     *
     * 之后每个 IK 点都反算 wrist，
     * 保证这个值不变。
     */
    s_vertical_lift.tool_angle_rad =
        shoulder_rad +
        elbow_relative_rad +
        wrist_relative_rad;

    s_vertical_lift.start_elbow_relative_rad =
        elbow_relative_rad;

    s_arm_state =
        ARM_STATE_LIFT_VERTICAL;

    return 1U;
}

/*
 * ============================================================
 * 竖直抬升：实时更新
 * ============================================================
 *
 * 不是直接把 Shoulder/Elbow 从起点线性插到终点。
 *
 * 每次 Arm_Process() 都重新生成：
 *
 *   X = 常数
 *   Z = Z0 + progress * 7 cm
 *
 * 然后重新进行 IK。
 *
 * 因此末端轨迹近似为真正的竖直直线。
 */
static uint8_t Arm_UpdateVerticalLift(void)
{
    uint32_t elapsed;

    float progress;
    float current_z_cm;

    if (s_vertical_lift.active == 0U)
    {
        return 1U;
    }

    elapsed =
        (uint32_t)(
            HAL_GetTick() -
            s_vertical_lift.start_tick);

    if (s_vertical_lift.duration_ms == 0U)
    {
        progress = 1.0f;
    }
    else if (elapsed >= s_vertical_lift.duration_ms)
    {
        progress = 1.0f;
    }
    else
    {
        progress =
            (float)elapsed /
            (float)s_vertical_lift.duration_ms;
    }

    current_z_cm =
        s_vertical_lift.start_z_cm +
        s_vertical_lift.lift_height_cm *
        progress;

    /*
     * 当前路径点不可解时停止继续写异常 PWM。
     * 当前标定和 PICK 参数下正常不会进入这里。
     */
    if (Arm_ApplyLiftPoint(
            s_vertical_lift.start_x_cm,
            current_z_cm) == 0U)
    {
        s_vertical_lift.active = 0U;
        return 1U;
    }

    if (progress >= 1.0f)
    {
        s_vertical_lift.active = 0U;
        return 1U;
    }

    return 0U;
}

/*
 * ============================================================
 * 竖直抬升：计算并输出某一个轨迹点
 * ============================================================
 */
static uint8_t Arm_ApplyLiftPoint(float x_cm,
                                  float z_cm)
{
    float shoulder_reference_rad;

    float shoulder_rad;
    float elbow_relative_rad;
    float wrist_relative_rad;

    float shoulder_pwm_f;
    float elbow_pwm_f;
    float wrist_pwm_f;

    uint16_t shoulder_pwm;
    uint16_t elbow_pwm;
    uint16_t wrist_pwm;

    /*
     * 当前 Shoulder PWM 只用于选择与当前姿态最接近的角度表达。
     */
    shoulder_reference_rad =
        Arm_ShoulderPwmToAngleRad(
            s_servo_motion[ARM_SERVO_2_SHOULDER].current_us);

    if (Arm_SolveIK(
            x_cm,
            z_cm,
            s_vertical_lift.start_elbow_relative_rad,
            shoulder_reference_rad,
            &shoulder_rad,
            &elbow_relative_rad) == 0U)
    {
        return 0U;
    }

    /*
     * 保持夹爪相对地面角度恒定：
     *
     * tool_angle =
     * shoulder +
     * elbow_relative +
     * wrist_relative
     *
     * 所以：
     * wrist_relative =
     * tool_angle -
     * shoulder -
     * elbow_relative
     */
    wrist_relative_rad =
        s_vertical_lift.tool_angle_rad -
        shoulder_rad -
        elbow_relative_rad;

    shoulder_pwm_f =
        Arm_ShoulderAngleRadToPwm(
            shoulder_rad);

    elbow_pwm_f =
        Arm_ElbowRelativeRadToPwm(
            elbow_relative_rad);

    wrist_pwm_f =
        Arm_WristRelativeRadToPwm(
            wrist_relative_rad);

    /*
     * 三个关节中任何一个需要超出 PWM 安全范围，
     * 就认为这个轨迹点不可执行。
     */
    if (Arm_FloatPwmToUint16(
            shoulder_pwm_f,
            &shoulder_pwm) == 0U)
    {
        return 0U;
    }

    if (Arm_FloatPwmToUint16(
            elbow_pwm_f,
            &elbow_pwm) == 0U)
    {
        return 0U;
    }

    if (Arm_FloatPwmToUint16(
            wrist_pwm_f,
            &wrist_pwm) == 0U)
    {
        return 0U;
    }

    /*
     * 同一轮循环快速更新三路 CCR。
     *
     * PWM 本身仍然是 50 Hz，
     * 三路在实际机械效果上可以认为同步联动。
     */
    Arm_WriteServoUs(
        ARM_SERVO_2_SHOULDER,
        shoulder_pwm);

    Arm_WriteServoUs(
        ARM_SERVO_3_ELBOW,
        elbow_pwm);

    Arm_WriteServoUs(
        ARM_SERVO_4_WRIST,
        wrist_pwm);

    return 1U;
}

/*
 * ============================================================
 * 收到夹取命令后的准备动作
 * ============================================================
 */

static void Arm_StartPrepareOpenGripper(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_5_GRIPPER,
        ARM_S5_GRIPPER_OPEN_US,
        ARM_PREPARE_OPEN_GRIPPER_MOVE_MS);

    s_arm_state =
        ARM_STATE_PREPARE_OPEN_GRIPPER;
}

/*
 * ============================================================
 * 夹取
 * ============================================================
 */

static void Arm_StartPickElbow(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_3_ELBOW,
        ARM_S3_ELBOW_PICK_US,
        ARM_PICK_ELBOW_MOVE_MS);

    s_arm_state =
        ARM_STATE_PICK_ELBOW;
}

static void Arm_StartPickShoulder(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_2_SHOULDER,
        ARM_S2_SHOULDER_PICK_US,
        ARM_PICK_SHOULDER_MOVE_MS);

    s_arm_state =
        ARM_STATE_PICK_SHOULDER;
}

static void Arm_StartPickWrist(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_4_WRIST,
        s_wrist_pick_position,
        ARM_PICK_WRIST_MOVE_MS);

    s_arm_state =
        ARM_STATE_PICK_WRIST;
}

static void Arm_StartPickGripper(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_5_GRIPPER,
        ARM_S5_GRIPPER_CLOSE_US,
        ARM_PICK_GRIPPER_MOVE_MS);

    s_arm_state =
        ARM_STATE_PICK_GRIPPER;
}

/*
 * ============================================================
 * 存放
 * ============================================================
 *
 * 当前存放顺序：
 * 1. 小臂 Elbow
 * 2. 腕部 Wrist
 * 3. 大臂 Shoulder
 * 4. 夹爪松开
 */

static void Arm_StartStoreShoulder(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_2_SHOULDER,
        ARM_S2_SHOULDER_STORE_US,
        ARM_STORE_SHOULDER_MOVE_MS);

    s_arm_state =
        ARM_STATE_STORE_SHOULDER;
}

static void Arm_StartStoreElbow(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_3_ELBOW,
        ARM_S3_ELBOW_STORE_US,
        ARM_STORE_ELBOW_MOVE_MS);

    s_arm_state =
        ARM_STATE_STORE_ELBOW;
}

static void Arm_StartStoreWrist(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_4_WRIST,
        ARM_S4_WRIST_STORE_US,
        ARM_STORE_WRIST_MOVE_MS);

    s_arm_state =
        ARM_STATE_STORE_WRIST;
}

static void Arm_StartStoreRelease(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_5_GRIPPER,
        ARM_S5_GRIPPER_OPEN_US,
        ARM_STORE_GRIPPER_MOVE_MS);

    s_arm_state =
        ARM_STATE_STORE_RELEASE;
}

/*
 * ============================================================
 * 归位
 * ============================================================
 */

static void Arm_StartHomeWrist(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_4_WRIST,
        ARM_S4_WRIST_HOME_US,
        ARM_HOME_WRIST_MOVE_MS);

    s_arm_state =
        ARM_STATE_HOME_WRIST;
}

static void Arm_StartHomeElbow(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_3_ELBOW,
        ARM_S3_ELBOW_HOME_US,
        ARM_HOME_ELBOW_MOVE_MS);

    s_arm_state =
        ARM_STATE_HOME_ELBOW;
}

static void Arm_StartHomeShoulder(void)
{
    Arm_MoveServoTo(
        ARM_SERVO_2_SHOULDER,
        ARM_S2_SHOULDER_HOME_US,
        ARM_HOME_SHOULDER_MOVE_MS);

    s_arm_state =
        ARM_STATE_HOME_SHOULDER;
}

/*
 * ============================================================
 * 任务完成
 * ============================================================
 */
static void Arm_FinishSequence(void)
{
    /*
     * 最终状态：
     *
     * Base      = HOME
     * Shoulder  = HOME
     * Elbow     = HOME
     * Wrist     = HOME
     * Gripper   = OPEN
     *
     * 上电初始化时 Gripper 是 CLOSE；
     * 收到下一次夹取命令时仍会先执行 OPEN。
     */
    s_arm_busy = 0U;
    s_arm_finished = 1U;

    s_arm_state =
        ARM_STATE_IDLE;
}


void Arm_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) ||
        (htim->Instance != TIM10))
    {
        return;
    }

    if (s_screw_motion.running == 0U)
    {
        return;
    }

    if (s_screw_motion.current_position_steps ==
        s_screw_motion.target_position_steps)
    {
        Arm_ScrewStopMotion();
        return;
    }

    /*
     * HAL_TIM_PWM_Start_IT() 为 TIM10_CH1 开启 CC1 中断。
     *
     * PWM Mode 1 下，每个 PWM 周期只有一次 CC1 比较事件，
     * 因此这里每进入一次就代表已经输出了一个完整的 STEP 高电平。
     */
    s_screw_motion.current_position_steps +=
        (int32_t)s_screw_motion.step_direction;

    if (s_screw_motion.current_position_steps ==
        s_screw_motion.target_position_steps)
    {
        /*
         * 当前 STEP 已经完成，立即关闭后续 PWM 周期。
         * 因此不会多输出一个脉冲。
         */
        Arm_ScrewStopMotion();
    }
}
