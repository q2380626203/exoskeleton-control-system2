/**
 * @file velocity_tracking_mode.h
 * @brief 速度跟踪模式控制头文件
 *
 * 基于位置环形缓存区的波峰波谷差值检测，实现智能速度跟踪模式：
 *
 * 节律控制工作原理：
 * 1. 启动条件：当波峰波谷差值>=0.75时，模式从ENABLED状态切换到ACTIVE状态
 * 2. 速度方向变化检测：持续监测电机速度方向变化（绝对值>0.35才有效）
 * 3. 时间间隔学习：记录相邻速度方向变化的时间间隔，用作下次动作的持续时间
 * 4. 动作映射：
 *    - 1号电机：检测到-v时执行抬腿MIT，检测到+v时执行放腿MIT
 *    - 2号电机：检测到+v时执行抬腿MIT，检测到-v时执行放腿MIT
 * 5. 自适应控制：每次MIT动作的持续时间 = 上一次相同方向变化的时间间隔
 * 6. 持续运行：一旦激活，持续进行节律控制，不再检查波峰波谷差值
 * 7. 重置机制：可调用reset函数重置到ENABLED状态，重新等待激活条件
 *
 * 交替工作机制：
 *
 * 阶段1 - 1号工作，2号休息：
 *   1号电机: -v→抬腿MIT→+v→放腿MIT(间隔+200ms)→完成后进入休息
 *   2号电机: 保持空闲休息状态
 *
 * 阶段2 - 2号工作，1号休息：
 *   2号电机: +v→抬腿MIT→-v→放腿MIT(间隔+200ms)→完成后进入休息
 *   1号电机: 保持空闲休息状态
 *
 * 时序：1号工作2号休息 → 2号工作1号休息 → 1号工作2号休息 → ...
 *
 * 关键特性：
 * - 任何时刻只有一个电机在工作，另一个在休息
 * - 放腿MIT完成后立即切换工作状态
 * - 休息的电机保持全0参数，确保不干扰
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
#define VELOCITY_TRACKING_ENABLE_THRESHOLD 0.75f    // 启用波峰波谷差值阈值
#define VELOCITY_TRACKING_MIN_VELOCITY 0.35f        // 最小有效速度阈值
#define VELOCITY_TRACKING_UPDATE_INTERVAL_MS 50     // 速度跟踪更新间隔(50ms)
#define LIFT_LEG_MAX_DURATION_MS 1000              // 抬腿MIT最大持续时间(1200ms)
#define TIMEOUT_DROP_LEG_DURATION_MS 600           // 抬腿超时后的放腿持续时间(600ms)

// 电机动作参数 - 抬腿
#define LIFT_LEG_TORQUE 6.0f        // 抬腿力矩 (Nm)
#define LIFT_LEG_POSITION 0.0f      // 抬腿位置 (rad)
#define LIFT_LEG_SPEED 2.75f         // 抬腿速度 (rad/s)
#define LIFT_LEG_KP 0.0f            // 抬腿Kp参数
#define LIFT_LEG_KD 1.0f            // 抬腿Kd参数

// 电机动作参数 - 放腿
#define DROP_LEG_TORQUE -2.0f       // 放腿力矩 (Nm)
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

    // 轮流工作周期管理
    work_cycle_t current_work_cycle;            // 当前工作周期（1号或2号）
    bool cycle_completed;                       // 当前周期是否完成
    uint32_t cycle_start_time;                  // 当前周期开始时间

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
extern float velocity_tracking_lift_leg_torque;    // 抬腿力矩
extern float velocity_tracking_lift_leg_speed;     // 抬腿速度
extern float velocity_tracking_drop_leg_torque;    // 放腿力矩
extern float velocity_tracking_drop_leg_speed;     // 放腿速度

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
 * @brief 更新速度跟踪模式（主要处理函数）
 * @param buffer 位置环形缓存区指针
 * @param motor1_velocity 电机1当前速度 (rad/s)
 * @param motor2_velocity 电机2当前速度 (rad/s)
 * @param timestamp 当前时间戳 (ms)
 * @return true如果状态发生变化，false无变化
 */
bool velocity_tracking_mode_update(position_ring_buffer_t *buffer,
                                  float motor1_velocity,
                                  float motor2_velocity,
                                  uint32_t timestamp);

/**
 * @brief 检查是否应该启用速度跟踪模式
 * @param buffer 位置环形缓存区指针
 * @return true应该启用，false不应启用
 */
bool velocity_tracking_should_enable(position_ring_buffer_t *buffer);

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