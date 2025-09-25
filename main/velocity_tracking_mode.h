/**
 * @file velocity_tracking_mode.h
 * @brief 速度跟踪模式控制头文件
 *
 * 基于位置环形缓存区的智能运动检测，实现轮流工作周期的速度跟踪模式：
 *
 * 节律控制工作原理：
 * 1. 启动条件：基于智能运动检测的爬楼升档判断(位置变化>=0.65)，模式从ENABLED状态切换到ACTIVE状态
 * 2. 速度方向检测：持续监测电机速度方向（绝对值>0.20才有效）
 * 3. 实时MIT执行：
 *    - 首次检测到速度方向时立即执行相应MIT动作（持续1.2秒）
 *    - 后续方向变化时立即执行MIT动作（持续时间=上次间隔+200ms）
 * 4. 动作映射：
 *    - 1号电机：检测到-v时立即执行抬腿MIT，检测到+v时立即执行放腿MIT
 *    - 2号电机：检测到+v时立即执行抬腿MIT，检测到-v时立即执行放腿MIT
 * 5. 自适应控制：MIT动作持续时间基于相邻速度变化的时间间隔自动调整
 * 6. 持续运行：一旦激活，持续进行节律控制，不再检查运动模式
 * 7. 超时保护：工作电机超时3秒未抬腿则自动重置到ENABLED状态，清空缓存区
 * 8. 抬腿超时处理：抬腿MIT超时800ms后强制切换到600ms放腿MIT
 *
 * 固定时长抬腿MIT交替工作机制：
 *
 * 电机工作模式：
 *   1号电机工作时: 检测到-v→抬腿0.6s→切换到2号工作→1号执行压腿0.6s
 *   2号电机工作时: 检测到+v→抬腿0.6s→切换到1号工作→2号执行压腿0.6s
 *
 * 切换时机：抬腿MIT完成后立即切换工作电机，同时开始压腿动作
 * 检测延迟：新工作电机延迟200ms后开始速度检测，避免误触发
 * 工作流程：1号工作→检测-v→抬腿0.6s→切换到2号→1号压腿0.6s(同时2号延迟200ms后检测+v)→循环
 *
 * 关键特性：
 * - 实时响应：检测到对应速度方向时立即执行抬腿MIT，无延迟
 * - 固定时长：抬腿MIT和压腿MIT都固定执行600ms
 * - 交替工作：任何时刻只有一个电机在工作检测，另一个在压腿或休息
 * - 完成切换：抬腿MIT完成后立即切换工作电机并开始压腿
 * - 检测延迟：新工作电机延迟200ms后开始检测，避免压腿期间误触发
 * - 智能启动：基于爬楼升档判断自动激活（位置变化>=0.65弧度）
 * - 超时保护：工作电机3秒未抬腿则自动重置，防止系统卡滞
 */

#ifndef VELOCITY_TRACKING_MODE_H
#define VELOCITY_TRACKING_MODE_H

#include <stdint.h>
#include <stdbool.h>
#include "continuous_torque_velocity_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

// 速度跟踪模式配置参数
#define VELOCITY_TRACKING_ENABLE_THRESHOLD 0.65f     // 启用阈值（复用爬楼升档阈值）
#define VELOCITY_TRACKING_MIN_VELOCITY 0.25f        // 最小有效速度阈值
#define VELOCITY_TRACKING_UPDATE_INTERVAL_MS 50     // 速度跟踪更新间隔(50ms)
#define LIFT_LEG_FIXED_DURATION_MS 600             // 固定抬腿MIT持续时间(600ms)
#define DROP_LEG_DELAY_MS 100                      // 压腿动作延时(200ms)
#define DROP_LEG_FIXED_DURATION_MS 600             // 固定压腿MIT持续时间(600ms)
#define LIFT_LEG_MAX_DURATION_MS 800              // 抬腿MIT最大持续时间(超时保护)
#define TIMEOUT_DROP_LEG_DURATION_MS 600           // 抬腿超时后的放腿持续时间(600ms)

// 工作周期超时管理
#define DEFAULT_CYCLE_DURATION_MS 2000             // 默认工作周期持续时间(2秒)
#define CYCLE_TIMEOUT_MULTIPLIER 1.0f              // 超时阈值倍数(1.5倍预期时间)

// 电机动作参数 - 抬腿
#define LIFT_LEG_TORQUE 6.0f        // 抬腿力矩 (Nm)
#define LIFT_LEG_POSITION 0.0f      // 抬腿位置 (rad)
#define LIFT_LEG_SPEED 2.75f         // 抬腿速度 (rad/s)
#define LIFT_LEG_KP 0.0f            // 抬腿Kp参数
#define LIFT_LEG_KD 1.0f            // 抬腿Kd参数

// 电机动作参数 - 放腿
#define DROP_LEG_TORQUE -1.0f       // 放腿力矩 (Nm)
#define DROP_LEG_POSITION 0.0f      // 放腿位置 (rad)
#define DROP_LEG_SPEED -1.5f        // 放腿速度 (rad/s)
#define DROP_LEG_KP 0.0f            // 放腿Kp参数
#define DROP_LEG_KD 1.0f            // 放腿Kd参数

// 速度跟踪模式状态枚举
typedef enum {
    VELOCITY_TRACKING_DISABLED = 0,     // 禁用状态
    VELOCITY_TRACKING_ENABLED = 1,      // 启用状态
    VELOCITY_TRACKING_ACTIVE = 2        // 激活状态（正在执行动作）
} velocity_tracking_state_t;

// 电机动作类型枚举
typedef enum {
    MOTOR_ACTION_IDLE = 0,      // 空闲
    MOTOR_ACTION_LIFT_LEG = 1,  // 抬腿
    MOTOR_ACTION_DROP_LEG = 2   // 放腿
} motor_action_t;

// 速度方向枚举
typedef enum {
    VELOCITY_DIR_UNKNOWN = 0,   // 未知方向
    VELOCITY_DIR_POSITIVE = 1,  // 正方向 (+v)
    VELOCITY_DIR_NEGATIVE = 2   // 负方向 (-v)
} velocity_direction_t;

// 工作周期枚举
typedef enum {
    WORK_CYCLE_MOTOR1 = 1,      // 1号电机工作周期
    WORK_CYCLE_MOTOR2 = 2       // 2号电机工作周期
} work_cycle_t;

// 节律控制状态结构体
typedef struct {
    velocity_direction_t current_dir;       // 当前速度方向
    velocity_direction_t last_dir;          // 上次速度方向
    uint32_t last_change_time;              // 上次方向变化时间
    uint32_t control_duration_ms;           // 当前控制持续时间
    uint32_t action_start_time;             // 动作开始时间
    bool action_active;                     // 动作是否正在执行
    motor_action_t current_action;          // 当前执行的动作
    bool detection_blocked;                 // 检测是否被阻止（互锁机制）
    bool is_resting;                        // 是否处于休息状态

    // 检测延迟控制
    bool detection_delayed;                 // 检测是否延迟中
    uint32_t detection_delay_start_time;    // 检测延迟开始时间

    // 节律统计
    uint32_t total_cycles;                  // 总循环次数
    uint32_t avg_cycle_time_ms;             // 平均循环时间
    uint32_t last_cycle_times[5];           // 最近5次循环时间（用于平滑）
    uint8_t cycle_time_index;               // 循环时间数组索引
} rhythm_control_t;

// 速度跟踪模式状态管理结构体
typedef struct {
    velocity_tracking_state_t state;            // 当前状态
    bool enabled;                               // 是否启用
    uint32_t last_update_time;                  // 上次更新时间戳

    // 运动状态管理（复用爬楼升降档逻辑作为启动条件）
    motion_mode_state_t motion_state;           // 运动状态管理

    // 轮流工作周期管理
    work_cycle_t current_work_cycle;            // 当前工作周期（1号或2号）
    bool cycle_completed;                       // 当前周期是否完成
    uint32_t cycle_start_time;                  // 当前周期开始时间
    uint32_t expected_cycle_duration_ms;        // 预期工作周期持续时间
    uint32_t cycle_timeout_threshold_ms;        // 周期超时阈值（2.5倍预期时间）

    // 切换保护机制
    uint32_t last_switch_time;                  // 上次工作电机切换时间
    uint32_t switch_protection_duration_ms;     // 切换保护持续时间

    // 节律控制状态
    rhythm_control_t motor1_rhythm;             // 电机1节律控制
    rhythm_control_t motor2_rhythm;             // 电机2节律控制

    // 统计信息
    uint32_t activation_count;                  // 激活次数
    uint32_t total_lift_actions;                // 总抬腿次数
    uint32_t total_drop_actions;                // 总放腿次数
    uint32_t total_work_cycles;                 // 总工作周期数
} velocity_tracking_context_t;

// 全局变量声明
extern velocity_tracking_context_t velocity_tracking_context;
extern bool velocity_tracking_mode_enabled;

// 可调参数全局变量（用于网页调整）
extern float velocity_tracking_lift_leg_torque;       // 抬腿力矩
extern float velocity_tracking_lift_leg_speed;        // 抬腿速度
extern float velocity_tracking_drop_leg_torque;       // 放腿力矩
extern float velocity_tracking_drop_leg_speed;        // 放腿速度
extern uint32_t velocity_tracking_lift_leg_max_duration;  // 抬腿MIT最大持续时间(ms)

// 新增可调参数全局变量
extern uint32_t velocity_tracking_lift_leg_fixed_duration_ms;  // 固定抬腿持续时间(ms)
extern uint32_t velocity_tracking_drop_leg_delay_ms;          // 压腿动作延时(ms)
extern uint32_t velocity_tracking_drop_leg_fixed_duration_ms; // 固定压腿持续时间(ms)
extern uint32_t velocity_tracking_default_cycle_duration_ms;  // 默认工作周期持续时间(ms)
extern float velocity_tracking_enable_threshold;              // 启用阈值
extern float velocity_tracking_min_velocity;                  // 最小有效速度阈值

// 函数声明

/**
 * @brief 初始化速度跟踪模式
 */
void velocity_tracking_mode_init(void);

/**
 * @brief 启用/禁用速度跟踪模式
 * @param enable true启用，false禁用
 */
void velocity_tracking_mode_set_enabled(bool enable);

/**
 * @brief 获取速度跟踪模式启用状态
 * @return true启用，false禁用
 */
bool velocity_tracking_mode_is_enabled(void);

/**
 * @brief 获取速度跟踪模式当前状态
 * @return 当前状态枚举值
 */
velocity_tracking_state_t velocity_tracking_mode_get_state(void);

/**
 * @brief 更新速度跟踪模式（主要处理函数，支持双电机升档判断）
 * @param buffer_motor1 1号电机位置环形缓存区指针
 * @param buffer_motor2 2号电机位置环形缓存区指针
 * @param motor1_velocity 电机1当前速度 (rad/s)
 * @param motor2_velocity 电机2当前速度 (rad/s)
 * @param timestamp 当前时间戳 (ms)
 * @return true如果状态发生变化，false无变化
 */
bool velocity_tracking_mode_update(position_ring_buffer_t *buffer_motor1,
                                  position_ring_buffer_t *buffer_motor2,
                                  float motor1_velocity,
                                  float motor2_velocity,
                                  uint32_t timestamp);

/**
 * @brief 检查是否应该启用速度跟踪模式（支持双电机升档判断）
 * @param buffer_motor1 1号电机位置环形缓存区指针
 * @param buffer_motor2 2号电机位置环形缓存区指针
 * @param timestamp 时间戳
 * @param triggered_motor 输出参数：触发启动的电机ID（1或2），如果未触发则为0
 * @return true应该启用，false不应启用
 */
bool velocity_tracking_should_enable(position_ring_buffer_t *buffer_motor1,
                                    position_ring_buffer_t *buffer_motor2,
                                    uint32_t timestamp,
                                    int *triggered_motor);

/**
 * @brief 根据速度决定电机动作
 * @param velocity 电机速度 (rad/s)
 * @param motor_id 电机ID (1或2)
 * @return 应该执行的动作类型
 */
motor_action_t velocity_tracking_get_motor_action(float velocity, int motor_id);

/**
 * @brief 执行电机动作
 * @param motor_id 电机ID (0为电机1，1为电机2)
 * @param action 要执行的动作
 */
void velocity_tracking_execute_motor_action(int motor_id, motor_action_t action);

/**
 * @brief 带时间控制的电机动作执行
 * @param motor_id 电机ID (0为电机1，1为电机2)
 * @param action 要执行的动作
 * @param duration_ms 动作持续时间（毫秒）
 */
void velocity_tracking_execute_timed_motor_action(int motor_id, motor_action_t action, uint32_t duration_ms);

/**
 * @brief 获取速度方向
 * @param velocity 速度值
 * @return 速度方向枚举
 */
velocity_direction_t velocity_tracking_get_direction(float velocity);

/**
 * @brief 检测速度方向变化并更新节律控制
 * @param rhythm 节律控制结构体指针
 * @param velocity 当前速度
 * @param timestamp 当前时间戳
 * @param motor_id 电机ID (1或2)
 * @return true如果检测到方向变化，false无变化
 */
bool velocity_tracking_update_rhythm(rhythm_control_t *rhythm, float velocity, uint32_t timestamp, int motor_id);

/**
 * @brief 初始化节律控制结构体
 * @param rhythm 节律控制结构体指针
 */
void velocity_tracking_init_rhythm(rhythm_control_t *rhythm);

/**
 * @brief 处理节律控制的定时动作
 * @param rhythm 节律控制结构体指针
 * @param motor_id 电机ID (0为电机1，1为电机2)
 * @param timestamp 当前时间戳
 * @return true如果动作状态发生变化，false无变化
 */
bool velocity_tracking_process_rhythm_timing(rhythm_control_t *rhythm, int motor_id, uint32_t timestamp);

/**
 * @brief 获取速度跟踪模式统计信息
 * @param context 返回的上下文信息
 */
void velocity_tracking_get_statistics(velocity_tracking_context_t *context);

/**
 * @brief 重置速度跟踪模式统计信息
 */
void velocity_tracking_reset_statistics(void);

/**
 * @brief 重置速度跟踪模式到启用状态（用于重新检测激活条件）
 */
void velocity_tracking_reset_to_enabled(void);

/**
 * @brief 检查工作周期是否超时并处理超时重置
 * @param buffer 位置环形缓存区指针
 * @param timestamp 当前时间戳
 * @return true如果发生了超时重置，false正常运行
 */
bool velocity_tracking_check_cycle_timeout(position_ring_buffer_t *buffer_motor1,
                                          position_ring_buffer_t *buffer_motor2,
                                          uint32_t timestamp);

/**
 * @brief 启动速度跟踪模式任务
 */
void velocity_tracking_start_task(void);

/**
 * @brief 停止速度跟踪模式任务
 */
void velocity_tracking_stop_task(void);

// 包含必要的结构体定义 - 避免重复定义
#ifndef MOTOR_CONTROL_PARAMS_T_DEFINED
#define MOTOR_CONTROL_PARAMS_T_DEFINED
typedef struct {
    float torque;      // 力矩设定 (Nm)
    float position;    // 位置设定 (rad)
    float speed;       // 速度设定 (rad/s)
    float kp;          // Kp参数
    float kd;          // Kd参数
} motor_control_params_t;
#endif // MOTOR_CONTROL_PARAMS_T_DEFINED

// 外部函数声明 - 来自main.c的统一电机控制函数
extern void unified_motor_control(int motor_id, const motor_control_params_t* params);

#ifdef __cplusplus
}
#endif

#endif // VELOCITY_TRACKING_MODE_H