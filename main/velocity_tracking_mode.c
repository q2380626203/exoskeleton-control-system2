/**
 * @file velocity_tracking_mode.c
 * @brief 速度跟踪模式控制实现
 *
 * 基于位置环形缓存区的波峰波谷差值检测，实现智能速度跟踪模式
 */

#include "velocity_tracking_mode.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "velocity_tracking";

// 全局变量定义
velocity_tracking_context_t velocity_tracking_context;
bool velocity_tracking_mode_enabled = false;

// 可调参数全局变量（用于网页调整）
float velocity_tracking_lift_leg_torque = LIFT_LEG_TORQUE;       // 抬腿力矩
float velocity_tracking_lift_leg_speed = LIFT_LEG_SPEED;         // 抬腿速度
float velocity_tracking_drop_leg_torque = DROP_LEG_TORQUE;       // 放腿力矩
float velocity_tracking_drop_leg_speed = DROP_LEG_SPEED;         // 放腿速度
uint32_t velocity_tracking_lift_leg_max_duration = LIFT_LEG_MAX_DURATION_MS;  // 抬腿MIT最大持续时间(ms)

// 新增可调参数全局变量
uint32_t velocity_tracking_lift_leg_fixed_duration_ms = LIFT_LEG_FIXED_DURATION_MS;  // 固定抬腿持续时间(ms)
uint32_t velocity_tracking_drop_leg_delay_ms = DROP_LEG_DELAY_MS;                    // 压腿动作延时(ms)
uint32_t velocity_tracking_drop_leg_fixed_duration_ms = DROP_LEG_FIXED_DURATION_MS;  // 固定压腿持续时间(ms)
uint32_t velocity_tracking_default_cycle_duration_ms = DEFAULT_CYCLE_DURATION_MS;    // 默认工作周期持续时间(ms)
float velocity_tracking_enable_threshold = VELOCITY_TRACKING_ENABLE_THRESHOLD;       // 启用阈值
float velocity_tracking_min_velocity = VELOCITY_TRACKING_MIN_VELOCITY;               // 最小有效速度阈值

// 任务句柄
static TaskHandle_t velocity_tracking_task_handle = NULL;

// 前向声明
void velocity_tracking_task(void* pvParameters);

/**
 * @brief 初始化节律控制结构体
 */
void velocity_tracking_init_rhythm(rhythm_control_t *rhythm) {
    if (rhythm == NULL) return;

    memset(rhythm, 0, sizeof(rhythm_control_t));
    rhythm->current_dir = VELOCITY_DIR_UNKNOWN;
    rhythm->last_dir = VELOCITY_DIR_UNKNOWN;
    rhythm->current_action = MOTOR_ACTION_IDLE;
    rhythm->control_duration_ms = 1000; // 默认1秒
    rhythm->detection_blocked = false;
    rhythm->is_resting = false;
    rhythm->detection_delayed = false;
    rhythm->detection_delay_start_time = 0;
}

/**
 * @brief 初始化速度跟踪模式
 */
void velocity_tracking_mode_init(void) {
    // 初始化上下文结构体
    memset(&velocity_tracking_context, 0, sizeof(velocity_tracking_context_t));

    velocity_tracking_context.state = VELOCITY_TRACKING_DISABLED;
    velocity_tracking_context.enabled = false;
    velocity_tracking_context.last_update_time = 0;

    // 初始化运动状态管理（复用爬楼升降档逻辑）
    motion_mode_state_init(&velocity_tracking_context.motion_state);

    // 初始化轮流工作周期管理
    velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1; // 从1号电机开始
    velocity_tracking_context.cycle_completed = false;
    velocity_tracking_context.cycle_start_time = 0;
    velocity_tracking_context.expected_cycle_duration_ms = velocity_tracking_default_cycle_duration_ms;
    velocity_tracking_context.cycle_timeout_threshold_ms = (uint32_t)(velocity_tracking_default_cycle_duration_ms * CYCLE_TIMEOUT_MULTIPLIER);

    // 初始化切换保护机制
    velocity_tracking_context.last_switch_time = 0;
    velocity_tracking_context.switch_protection_duration_ms = 0;

    // 初始化节律控制
    velocity_tracking_init_rhythm(&velocity_tracking_context.motor1_rhythm);
    velocity_tracking_init_rhythm(&velocity_tracking_context.motor2_rhythm);

    // 设置初始工作状态：1号工作，2号休息
    velocity_tracking_context.motor1_rhythm.is_resting = false;
    velocity_tracking_context.motor2_rhythm.is_resting = true;

    velocity_tracking_mode_enabled = false;

    ESP_LOGI(TAG, "速度跟踪模式初始化完成（轮流工作周期版本）");
}

/**
 * @brief 启用/禁用速度跟踪模式
 */
void velocity_tracking_mode_set_enabled(bool enable) {
    velocity_tracking_mode_enabled = enable;
    velocity_tracking_context.enabled = enable;

    if (enable) {
        velocity_tracking_context.state = VELOCITY_TRACKING_ENABLED;
        ESP_LOGI(TAG, "速度跟踪模式已启用");
    } else {
        velocity_tracking_context.state = VELOCITY_TRACKING_DISABLED;
        // 重置节律控制状态
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor1_rhythm);
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor2_rhythm);
        // 重置切换保护机制
        velocity_tracking_context.last_switch_time = 0;
        velocity_tracking_context.switch_protection_duration_ms = 0;
        ESP_LOGI(TAG, "速度跟踪模式已禁用");
    }
}

/**
 * @brief 获取速度跟踪模式启用状态
 */
bool velocity_tracking_mode_is_enabled(void) {
    return velocity_tracking_mode_enabled;
}

/**
 * @brief 获取速度跟踪模式当前状态
 */
velocity_tracking_state_t velocity_tracking_mode_get_state(void) {
    return velocity_tracking_context.state;
}

/**
 * @brief 检查是否应该启用速度跟踪模式（支持双电机升档判断）
 */
bool velocity_tracking_should_enable(position_ring_buffer_t *buffer_motor1,
                                    position_ring_buffer_t *buffer_motor2,
                                    uint32_t timestamp,
                                    int *triggered_motor) {
    if (buffer_motor1 == NULL || buffer_motor2 == NULL || triggered_motor == NULL) {
        if (triggered_motor) *triggered_motor = 0;
        return false;
    }

    // 检查1号电机的升档条件
    motion_mode_t motor1_mode = detect_motion_mode(buffer_motor1, timestamp, &velocity_tracking_context.motion_state);
    bool motor1_should_enable = (motor1_mode == MOTION_MODE_CLIMBING);

    // 检查2号电机的升档条件（使用独立的motion_state副本避免干扰）
    motion_mode_state_t motor2_motion_state;
    motion_mode_state_init(&motor2_motion_state);
    motion_mode_t motor2_mode = detect_motion_mode(buffer_motor2, timestamp, &motor2_motion_state);
    bool motor2_should_enable = (motor2_mode == MOTION_MODE_CLIMBING);

    // 决定触发电机和启动状态
    bool should_enable = motor1_should_enable || motor2_should_enable;

    if (should_enable) {
        // 优先选择1号电机，如果1号不满足但2号满足，则选择2号
        if (motor1_should_enable) {
            *triggered_motor = 1;
        } else {
            *triggered_motor = 2;
        }
    } else {
        *triggered_motor = 0;
    }

    ESP_LOGD(TAG, "双电机升档检测: 1号电机=%s(%s), 2号电机=%s(%s) -> velocity_tracking %s (触发电机: %d)",
             motor1_mode == MOTION_MODE_STATIC ? "静止" :
             motor1_mode == MOTION_MODE_WALKING ? "行走" : "爬楼",
             motor1_should_enable ? "满足" : "不满足",
             motor2_mode == MOTION_MODE_STATIC ? "静止" :
             motor2_mode == MOTION_MODE_WALKING ? "行走" : "爬楼",
             motor2_should_enable ? "满足" : "不满足",
             should_enable ? "启用" : "禁用",
             *triggered_motor);

    return should_enable;
}

/**
 * @brief 获取速度方向
 */
velocity_direction_t velocity_tracking_get_direction(float velocity) {
    if (fabsf(velocity) < velocity_tracking_min_velocity) {
        return VELOCITY_DIR_UNKNOWN;
    }
    return (velocity > 0) ? VELOCITY_DIR_POSITIVE : VELOCITY_DIR_NEGATIVE;
}

/**
 * @brief 检测速度方向并触发抬腿MIT（带检测延迟版本）
 */
bool velocity_tracking_update_rhythm(rhythm_control_t *rhythm, float velocity, uint32_t timestamp, int motor_id) {
    if (rhythm == NULL) return false;

    // 如果检测被阻止，跳过处理
    if (rhythm->detection_blocked) {
        ESP_LOGD(TAG, "电机%d检测被阻止，跳过节律更新", motor_id);
        return false;
    }

    // 检查检测延迟状态
    if (rhythm->detection_delayed) {
        uint32_t delay_elapsed = timestamp - rhythm->detection_delay_start_time;
        if (delay_elapsed < velocity_tracking_drop_leg_delay_ms) {
            // 仍在延迟期内，跳过检测
            ESP_LOGD(TAG, "电机%d检测延迟中，已过%lu ms，还需%lu ms",
                     motor_id, delay_elapsed, velocity_tracking_drop_leg_delay_ms - delay_elapsed);
            return false;
        } else {
            // 延迟期结束，恢复检测
            rhythm->detection_delayed = false;
            // 检测延迟结束，开始新的工作周期计时
            velocity_tracking_context.cycle_start_time = timestamp;
            ESP_LOGI(TAG, "电机%d检测延迟结束，恢复速度检测，开始工作周期计时", motor_id);
        }
    }

    // 如果当前已有动作在执行，跳过处理
    if (rhythm->action_active) {
        return false;
    }

    velocity_direction_t new_dir = velocity_tracking_get_direction(velocity);

    // 如果速度方向未知，跳过处理
    if (new_dir == VELOCITY_DIR_UNKNOWN) {
        return false;
    }

    // 检查是否应该触发抬腿动作
    bool should_lift = false;
    if (motor_id == 1 && new_dir == VELOCITY_DIR_NEGATIVE) {
        // 1号电机：检测到-v触发抬腿
        should_lift = true;
    } else if (motor_id == 2 && new_dir == VELOCITY_DIR_POSITIVE) {
        // 2号电机：检测到+v触发抬腿
        should_lift = true;
    }

    if (should_lift) {
        // 执行抬腿动作
        rhythm->current_action = MOTOR_ACTION_LIFT_LEG;
        rhythm->action_start_time = timestamp;
        rhythm->action_active = true;
        rhythm->control_duration_ms = velocity_tracking_lift_leg_fixed_duration_ms; // 固定时长
        rhythm->current_dir = new_dir;

        ESP_LOGI(TAG, "电机%d检测到%s，执行抬腿MIT（固定时长: %lu ms）",
                 motor_id,
                 new_dir == VELOCITY_DIR_POSITIVE ? "+v" : "-v",
                 rhythm->control_duration_ms);

        // 实际执行电机动作
        velocity_tracking_execute_motor_action(motor_id - 1, MOTOR_ACTION_LIFT_LEG);

        // 更新统计
        rhythm->total_cycles++;
        rhythm->last_change_time = timestamp;
        rhythm->last_dir = new_dir;

        return true; // 状态发生了变化
    }

    return false;
}

/**
 * @brief 根据速度决定电机动作（保留原有接口）
 */
motor_action_t velocity_tracking_get_motor_action(float velocity, int motor_id) {
    velocity_direction_t dir = velocity_tracking_get_direction(velocity);
    if (dir == VELOCITY_DIR_UNKNOWN) {
        return MOTOR_ACTION_IDLE;
    }

    if (motor_id == 1) {
        return (dir == VELOCITY_DIR_NEGATIVE) ? MOTOR_ACTION_LIFT_LEG : MOTOR_ACTION_DROP_LEG;
    } else if (motor_id == 2) {
        return (dir == VELOCITY_DIR_POSITIVE) ? MOTOR_ACTION_LIFT_LEG : MOTOR_ACTION_DROP_LEG;
    }

    return MOTOR_ACTION_IDLE;
}

/**
 * @brief 执行电机动作
 */
void velocity_tracking_execute_motor_action(int motor_id, motor_action_t action) {
    motor_control_params_t params = {0};

    switch (action) {
        case MOTOR_ACTION_LIFT_LEG:
            // 抬腿参数（使用可调参数）
            params.torque = velocity_tracking_lift_leg_torque;
            params.position = LIFT_LEG_POSITION;
            params.speed = velocity_tracking_lift_leg_speed;
            params.kp = LIFT_LEG_KP;
            params.kd = LIFT_LEG_KD;

            // 根据电机ID调整速度方向
            if (motor_id == 0) { // 电机1 (ID=0)
                params.speed = -fabsf(params.speed); // 负速度
                params.torque = -fabsf(params.torque); // 负力矩
            } else { // 电机2 (ID=1)
                params.speed = fabsf(params.speed); // 正速度
                params.torque = fabsf(params.torque); // 正力矩
            }

            velocity_tracking_context.total_lift_actions++;
            ESP_LOGD(TAG, "电机%d执行抬腿动作: 力矩=%.1f, 速度=%.1f",
                     motor_id+1, params.torque, params.speed);
            break;

        case MOTOR_ACTION_DROP_LEG:
            // 放腿参数（使用可调参数）
            params.torque = velocity_tracking_drop_leg_torque;
            params.position = DROP_LEG_POSITION;
            params.speed = velocity_tracking_drop_leg_speed;
            params.kp = DROP_LEG_KP;
            params.kd = DROP_LEG_KD;

            // 根据电机ID调整速度方向
            if (motor_id == 0) { // 电机1 (ID=0)
                params.speed = fabsf(params.speed); // 正速度
                params.torque = fabsf(params.torque); // 正力矩
            } else { // 电机2 (ID=1)
                params.speed = -fabsf(params.speed); // 负速度
                params.torque = -fabsf(params.torque); // 负力矩
            }

            velocity_tracking_context.total_drop_actions++;
            ESP_LOGD(TAG, "电机%d执行放腿动作: 力矩=%.1f, 速度=%.1f",
                     motor_id+1, params.torque, params.speed);
            break;

        case MOTOR_ACTION_IDLE:
        default:
            // 空闲状态，设置为零力矩零速度
            params.torque = 0.0f;
            params.position = 0.0f;
            params.speed = 0.0f;
            params.kp = 0.0f;
            params.kd = 0.0f;
            ESP_LOGD(TAG, "电机%d设置为空闲状态", motor_id+1);
            break;
    }

    // 调用统一电机控制函数
    unified_motor_control(motor_id, &params);
}

/**
 * @brief 带时间控制的电机动作执行
 */
void velocity_tracking_execute_timed_motor_action(int motor_id, motor_action_t action, uint32_t duration_ms) {
    // 首先执行动作
    velocity_tracking_execute_motor_action(motor_id, action);

    // 记录动作开始时间和持续时间（在调用方的rhythm结构中管理）
    ESP_LOGI(TAG, "电机%d执行定时动作: %s，持续%lu ms",
             motor_id+1,
             action == MOTOR_ACTION_LIFT_LEG ? "抬腿" :
             action == MOTOR_ACTION_DROP_LEG ? "放腿" : "空闲",
             duration_ms);
}

/**
 * @brief 处理抬腿MIT完成后的切换（带压腿动作版本）
 */
bool velocity_tracking_process_rhythm_timing(rhythm_control_t *rhythm, int motor_id, uint32_t timestamp) {
    if (rhythm == NULL || !rhythm->action_active) {
        return false;
    }

    uint32_t elapsed_time = timestamp - rhythm->action_start_time;

    // 检查抬腿MIT是否完成
    if (rhythm->current_action == MOTOR_ACTION_LIFT_LEG && elapsed_time >= rhythm->control_duration_ms) {
        // 立即切换工作电机，然后当前电机开始执行压腿动作
        if (motor_id == 0) { // 1号电机完成抬腿
            velocity_tracking_context.motor1_rhythm.is_resting = true;
            velocity_tracking_context.motor2_rhythm.is_resting = false;
            velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR2;
            velocity_tracking_context.total_work_cycles++;

            // 为2号电机设置检测延迟
            velocity_tracking_context.motor2_rhythm.detection_delayed = true;
            velocity_tracking_context.motor2_rhythm.detection_delay_start_time = timestamp;

            ESP_LOGI(TAG, "1号电机抬腿完成，切换到2号电机工作，2号检测延迟%lu ms (总周期数: %lu)",
                     velocity_tracking_drop_leg_delay_ms, velocity_tracking_context.total_work_cycles);
        } else if (motor_id == 1) { // 2号电机完成抬腿
            velocity_tracking_context.motor1_rhythm.is_resting = false;
            velocity_tracking_context.motor2_rhythm.is_resting = true;
            velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1;
            velocity_tracking_context.total_work_cycles++;

            // 为1号电机设置检测延迟
            velocity_tracking_context.motor1_rhythm.detection_delayed = true;
            velocity_tracking_context.motor1_rhythm.detection_delay_start_time = timestamp;

            ESP_LOGI(TAG, "2号电机抬腿完成，切换到1号电机工作，1号检测延迟%lu ms (总周期数: %lu)",
                     velocity_tracking_drop_leg_delay_ms, velocity_tracking_context.total_work_cycles);
        }

        // 抬腿MIT完成，当前电机开始压腿动作
        rhythm->current_action = MOTOR_ACTION_DROP_LEG;
        rhythm->action_start_time = timestamp; // 重新计时
        rhythm->control_duration_ms = velocity_tracking_drop_leg_fixed_duration_ms; // 压腿固定时长

        ESP_LOGI(TAG, "电机%d抬腿MIT完成，开始执行压腿MIT（固定时长: %lu ms）",
                 motor_id+1, rhythm->control_duration_ms);

        // 实际执行压腿动作（在完成抬腿的电机上）
        velocity_tracking_execute_motor_action(motor_id, MOTOR_ACTION_DROP_LEG);

        return true; // 状态发生了变化

    } else if (rhythm->current_action == MOTOR_ACTION_DROP_LEG && elapsed_time >= rhythm->control_duration_ms) {
        // 压腿MIT完成
        rhythm->action_active = false;
        rhythm->current_action = MOTOR_ACTION_IDLE;

        ESP_LOGI(TAG, "电机%d压腿MIT完成，耗时: %lu ms，设置为空闲状态", motor_id+1, elapsed_time);

        // 设置电机为空闲状态
        velocity_tracking_execute_motor_action(motor_id, MOTOR_ACTION_IDLE);

        return true; // 状态发生了变化
    }

    return false; // 未完成
}

/**
 * @brief 更新速度跟踪模式（主要处理函数，支持双电机升档判断）
 */
bool velocity_tracking_mode_update(position_ring_buffer_t *buffer_motor1,
                                  position_ring_buffer_t *buffer_motor2,
                                  float motor1_velocity,
                                  float motor2_velocity,
                                  uint32_t timestamp) {
    if (!velocity_tracking_mode_enabled || buffer_motor1 == NULL || buffer_motor2 == NULL) {
        return false;
    }

    bool state_changed = false;

    // 检查更新间隔
    if ((timestamp - velocity_tracking_context.last_update_time) < VELOCITY_TRACKING_UPDATE_INTERVAL_MS) {
        return false;
    }

    velocity_tracking_context.last_update_time = timestamp;

    // 检查工作周期超时并处理超时重置
    if (velocity_tracking_check_cycle_timeout(buffer_motor1, buffer_motor2, timestamp)) {
        return true; // 发生了超时重置，状态已改变
    }

    // 只有在ENABLED状态时才检查是否应该激活，一旦激活就保持激活状态
    if (velocity_tracking_context.state == VELOCITY_TRACKING_ENABLED) {
        // 检查是否应该启用速度跟踪（使用双电机爬楼升档判断逻辑）
        int triggered_motor = 0;
        bool should_enable = velocity_tracking_should_enable(buffer_motor1, buffer_motor2, timestamp, &triggered_motor);

        if (should_enable) {
            // 激活速度跟踪模式
            velocity_tracking_context.state = VELOCITY_TRACKING_ACTIVE;
            velocity_tracking_context.activation_count++;
            // 注意：cycle_start_time将在检测延迟结束时设置

            // 根据触发电机设置初始工作状态
            if (triggered_motor == 1) {
                // 1号电机触发，1号工作，2号休息
                velocity_tracking_context.motor1_rhythm.is_resting = false;
                velocity_tracking_context.motor2_rhythm.is_resting = true;
                velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1;
                ESP_LOGI(TAG, "速度跟踪模式激活 (激活次数: %lu) - 1号电机触发启动，1号工作，2号休息", velocity_tracking_context.activation_count);
            } else if (triggered_motor == 2) {
                // 2号电机触发，2号工作，1号休息
                velocity_tracking_context.motor1_rhythm.is_resting = true;
                velocity_tracking_context.motor2_rhythm.is_resting = false;
                velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR2;
                ESP_LOGI(TAG, "速度跟踪模式激活 (激活次数: %lu) - 2号电机触发启动，2号工作，1号休息", velocity_tracking_context.activation_count);
            } else {
                // 默认情况（不应该发生）
                velocity_tracking_context.motor1_rhythm.is_resting = false;
                velocity_tracking_context.motor2_rhythm.is_resting = true;
                velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1;
                ESP_LOGI(TAG, "速度跟踪模式激活 (激活次数: %lu) - 未知触发电机，默认1号工作", velocity_tracking_context.activation_count);
            }

            state_changed = true;
        }
    }
    // 注意：一旦激活后，不再检查运动模式，保持激活状态
    // 如果需要停止，需要手动调用禁用函数

    // 如果处于激活状态，处理交替工作控制
    if (velocity_tracking_context.state == VELOCITY_TRACKING_ACTIVE) {
        bool motor1_rhythm_changed = false;
        bool motor2_rhythm_changed = false;

        // 简化模式：切换只在抬腿MIT完成后发生，这里不需要切换逻辑

        // 交替工作逻辑：同时只有一个电机工作，另一个休息

        // 1号电机处理（只在1号工作且检测到负速度时处理）
        if (!velocity_tracking_context.motor1_rhythm.is_resting) {
            // 1号电机工作，2号电机休息
            velocity_tracking_context.motor1_rhythm.detection_blocked = false;
            velocity_tracking_context.motor2_rhythm.detection_blocked = true;

            // 1号电机检测自己的速度数据区，只处理负速度(-v)
            velocity_direction_t current_dir = velocity_tracking_get_direction(motor1_velocity);
            if (current_dir == VELOCITY_DIR_NEGATIVE) {
                motor1_rhythm_changed = velocity_tracking_update_rhythm(
                    &velocity_tracking_context.motor1_rhythm, motor1_velocity, timestamp, 1);
            }
        } else {
            // 1号电机休息，但如果正在执行压腿动作则不打断
            if (velocity_tracking_context.motor1_rhythm.action_active &&
                velocity_tracking_context.motor1_rhythm.current_action != MOTOR_ACTION_DROP_LEG) {
                velocity_tracking_context.motor1_rhythm.action_active = false;
                velocity_tracking_context.motor1_rhythm.current_action = MOTOR_ACTION_IDLE;
                velocity_tracking_execute_motor_action(0, MOTOR_ACTION_IDLE);
            }
        }

        // 2号电机处理（只在2号工作且检测到正速度时处理）
        if (!velocity_tracking_context.motor2_rhythm.is_resting) {
            // 2号电机工作，1号电机休息
            velocity_tracking_context.motor2_rhythm.detection_blocked = false;
            velocity_tracking_context.motor1_rhythm.detection_blocked = true;

            // 2号电机检测自己的速度数据区，只处理正速度(+v)
            velocity_direction_t current_dir = velocity_tracking_get_direction(motor2_velocity);
            if (current_dir == VELOCITY_DIR_POSITIVE) {
                motor2_rhythm_changed = velocity_tracking_update_rhythm(
                    &velocity_tracking_context.motor2_rhythm, motor2_velocity, timestamp, 2);
            }
        } else {
            // 2号电机休息，但如果正在执行压腿动作则不打断
            if (velocity_tracking_context.motor2_rhythm.action_active &&
                velocity_tracking_context.motor2_rhythm.current_action != MOTOR_ACTION_DROP_LEG) {
                velocity_tracking_context.motor2_rhythm.action_active = false;
                velocity_tracking_context.motor2_rhythm.current_action = MOTOR_ACTION_IDLE;
                velocity_tracking_execute_motor_action(1, MOTOR_ACTION_IDLE);
            }
        }

        // 处理电机1的定时动作
        bool motor1_timing_changed = velocity_tracking_process_rhythm_timing(
            &velocity_tracking_context.motor1_rhythm, 0, timestamp);

        // 处理电机2的定时动作
        bool motor2_timing_changed = velocity_tracking_process_rhythm_timing(
            &velocity_tracking_context.motor2_rhythm, 1, timestamp);

        // 如果检测到新的速度方向变化，立即执行对应动作
        if (motor1_rhythm_changed) {
            velocity_tracking_execute_timed_motor_action(0,
                velocity_tracking_context.motor1_rhythm.current_action,
                velocity_tracking_context.motor1_rhythm.control_duration_ms);
            velocity_tracking_context.total_lift_actions +=
                (velocity_tracking_context.motor1_rhythm.current_action == MOTOR_ACTION_LIFT_LEG) ? 1 : 0;
            velocity_tracking_context.total_drop_actions +=
                (velocity_tracking_context.motor1_rhythm.current_action == MOTOR_ACTION_DROP_LEG) ? 1 : 0;
        }

        if (motor2_rhythm_changed) {
            velocity_tracking_execute_timed_motor_action(1,
                velocity_tracking_context.motor2_rhythm.current_action,
                velocity_tracking_context.motor2_rhythm.control_duration_ms);
            velocity_tracking_context.total_lift_actions +=
                (velocity_tracking_context.motor2_rhythm.current_action == MOTOR_ACTION_LIFT_LEG) ? 1 : 0;
            velocity_tracking_context.total_drop_actions +=
                (velocity_tracking_context.motor2_rhythm.current_action == MOTOR_ACTION_DROP_LEG) ? 1 : 0;
        }

        // 记录状态变化
        state_changed = motor1_rhythm_changed || motor2_rhythm_changed || motor1_timing_changed || motor2_timing_changed;
    }

    return state_changed;
}

/**
 * @brief 获取速度跟踪模式统计信息
 */
void velocity_tracking_get_statistics(velocity_tracking_context_t *context) {
    if (context != NULL) {
        *context = velocity_tracking_context;
    }
}

/**
 * @brief 重置速度跟踪模式统计信息
 */
void velocity_tracking_reset_statistics(void) {
    velocity_tracking_context.activation_count = 0;
    velocity_tracking_context.total_lift_actions = 0;
    velocity_tracking_context.total_drop_actions = 0;
    ESP_LOGI(TAG, "速度跟踪模式统计信息已重置");
}

/**
 * @brief 重置速度跟踪模式到启用状态（用于重新检测激活条件）
 */
void velocity_tracking_reset_to_enabled(void) {
    if (velocity_tracking_context.enabled) {
        velocity_tracking_context.state = VELOCITY_TRACKING_ENABLED;

        // 重置轮流工作周期管理
        velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1; // 重新从1号电机开始
        velocity_tracking_context.cycle_completed = false;
        velocity_tracking_context.cycle_start_time = 0;

        // 重置节律控制状态
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor1_rhythm);
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor2_rhythm);

        // 重置切换保护机制
        velocity_tracking_context.last_switch_time = 0;
        velocity_tracking_context.switch_protection_duration_ms = 0;

        // 设置电机为空闲状态
        velocity_tracking_execute_motor_action(0, MOTOR_ACTION_IDLE);
        velocity_tracking_execute_motor_action(1, MOTOR_ACTION_IDLE);

        ESP_LOGI(TAG, "速度跟踪模式已重置到启用状态，轮流工作周期已初始化，等待波峰波谷差值>=0.75重新激活");
    } else {
        ESP_LOGW(TAG, "速度跟踪模式未启用，无法重置到启用状态");
    }
}

/**
 * @brief 速度跟踪模式任务
 */
void velocity_tracking_task(void* pvParameters) {
    ESP_LOGI(TAG, "速度跟踪模式任务启动");

    TickType_t last_update = xTaskGetTickCount();
    const TickType_t update_interval = pdMS_TO_TICKS(VELOCITY_TRACKING_UPDATE_INTERVAL_MS);

    while (1) {
        TickType_t current_time = xTaskGetTickCount();

        if (velocity_tracking_mode_enabled) {
            // 这里可以添加定期的状态检查或维护逻辑
            // 目前主要的处理逻辑在velocity_tracking_mode_update中

            // 定期输出状态信息
            if ((current_time - last_update) >= pdMS_TO_TICKS(5000)) { // 每5秒
                ESP_LOGI(TAG, "轮流工作周期状态报告:");
                ESP_LOGI(TAG, "  模式: %s, 激活次数: %lu, 总工作周期: %lu",
                         velocity_tracking_context.state == VELOCITY_TRACKING_DISABLED ? "禁用" :
                         velocity_tracking_context.state == VELOCITY_TRACKING_ENABLED ? "启用" : "激活",
                         velocity_tracking_context.activation_count,
                         velocity_tracking_context.total_work_cycles);

                ESP_LOGI(TAG, "  工作状态: 1号电机=%s, 2号电机=%s",
                         velocity_tracking_context.motor1_rhythm.is_resting ? "休息" : "工作",
                         velocity_tracking_context.motor2_rhythm.is_resting ? "休息" : "工作");

                ESP_LOGI(TAG, "  抬腿次数: %lu, 放腿次数: %lu",
                         velocity_tracking_context.total_lift_actions,
                         velocity_tracking_context.total_drop_actions);

                if (velocity_tracking_context.state == VELOCITY_TRACKING_ACTIVE) {
                    ESP_LOGI(TAG, "  电机1: 方向=%s, 循环数=%lu, 平均周期=%lu ms, 当前动作=%s",
                             velocity_tracking_context.motor1_rhythm.current_dir == VELOCITY_DIR_POSITIVE ? "+v" :
                             velocity_tracking_context.motor1_rhythm.current_dir == VELOCITY_DIR_NEGATIVE ? "-v" : "未知",
                             velocity_tracking_context.motor1_rhythm.total_cycles,
                             velocity_tracking_context.motor1_rhythm.avg_cycle_time_ms,
                             velocity_tracking_context.motor1_rhythm.action_active ?
                             (velocity_tracking_context.motor1_rhythm.current_action == MOTOR_ACTION_LIFT_LEG ? "抬腿" : "放腿") : "空闲");

                    ESP_LOGI(TAG, "  电机2: 方向=%s, 循环数=%lu, 平均周期=%lu ms, 当前动作=%s",
                             velocity_tracking_context.motor2_rhythm.current_dir == VELOCITY_DIR_POSITIVE ? "+v" :
                             velocity_tracking_context.motor2_rhythm.current_dir == VELOCITY_DIR_NEGATIVE ? "-v" : "未知",
                             velocity_tracking_context.motor2_rhythm.total_cycles,
                             velocity_tracking_context.motor2_rhythm.avg_cycle_time_ms,
                             velocity_tracking_context.motor2_rhythm.action_active ?
                             (velocity_tracking_context.motor2_rhythm.current_action == MOTOR_ACTION_LIFT_LEG ? "抬腿" : "放腿") : "空闲");
                }

                last_update = current_time;
            }
        }

        // 任务延时
        vTaskDelay(update_interval);
    }

    ESP_LOGI(TAG, "速度跟踪模式任务退出");
    velocity_tracking_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 检查工作周期是否超时并处理超时重置
 */
bool velocity_tracking_check_cycle_timeout(position_ring_buffer_t *buffer_motor1,
                                          position_ring_buffer_t *buffer_motor2,
                                          uint32_t timestamp) {
    // 只在ACTIVE状态时检查超时
    if (velocity_tracking_context.state != VELOCITY_TRACKING_ACTIVE) {
        return false;
    }

    // 检查当前是否有工作周期在进行
    if (velocity_tracking_context.cycle_start_time == 0) {
        return false; // 没有活跃的工作周期
    }

    // 计算当前工作周期已运行时间
    uint32_t cycle_elapsed_time = timestamp - velocity_tracking_context.cycle_start_time;

    // 检查是否超过超时阈值
    if (cycle_elapsed_time >= velocity_tracking_context.cycle_timeout_threshold_ms) {
        // 确定当前应该工作的电机
        int working_motor_id = (velocity_tracking_context.current_work_cycle == WORK_CYCLE_MOTOR1) ? 0 : 1;
        rhythm_control_t *working_rhythm = (working_motor_id == 0) ?
            &velocity_tracking_context.motor1_rhythm : &velocity_tracking_context.motor2_rhythm;

        ESP_LOGW(TAG, "【超时重置】%d号电机工作周期超时！", working_motor_id + 1);
        ESP_LOGW(TAG, "  运行时间: %lu ms >= 超时阈值: %lu ms",
                 cycle_elapsed_time, velocity_tracking_context.cycle_timeout_threshold_ms);
        ESP_LOGW(TAG, "  预期周期时间: %lu ms (%.1f倍)",
                 velocity_tracking_context.expected_cycle_duration_ms, CYCLE_TIMEOUT_MULTIPLIER);

        // 检查工作电机是否还在等待抬腿动作
        if (!working_rhythm->action_active && working_rhythm->current_action == MOTOR_ACTION_IDLE) {
            ESP_LOGW(TAG, "  原因: %d号电机在预期时间%.1f倍内未检测到抬腿动作",
                     working_motor_id + 1, CYCLE_TIMEOUT_MULTIPLIER);
        } else {
            ESP_LOGW(TAG, "  原因: %d号电机动作执行异常或卡滞", working_motor_id + 1);
        }

        // 执行超时重置：回到ENABLED状态
        velocity_tracking_context.state = VELOCITY_TRACKING_ENABLED;
        velocity_tracking_context.cycle_start_time = 0;
        velocity_tracking_context.cycle_completed = false;

        // 重置节律控制状态
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor1_rhythm);
        velocity_tracking_init_rhythm(&velocity_tracking_context.motor2_rhythm);

        // 重置切换保护机制
        velocity_tracking_context.last_switch_time = 0;
        velocity_tracking_context.switch_protection_duration_ms = 0;

        // 重置为默认工作状态（重新等待触发，智能选择工作电机）
        velocity_tracking_context.motor1_rhythm.is_resting = false;
        velocity_tracking_context.motor2_rhythm.is_resting = true;
        velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1;

        // 清空两个位置缓存区，等待重新激活
        if (buffer_motor1 != NULL && buffer_motor2 != NULL) {
            position_ring_buffer_clear(buffer_motor1);
            position_ring_buffer_clear(buffer_motor2);
            ESP_LOGI(TAG, "【超时重置】清空双电机位置缓存区，等待重新激活");
        }

        // 设置电机为空闲状态
        velocity_tracking_execute_motor_action(0, MOTOR_ACTION_IDLE);
        velocity_tracking_execute_motor_action(1, MOTOR_ACTION_IDLE);

        ESP_LOGI(TAG, "【超时重置】velocity_tracking模式已重置到ENABLED状态，等待重新激活");

        return true; // 发生了超时重置
    }

    return false; // 正常运行
}

/**
 * @brief 启动速度跟踪模式任务
 */
void velocity_tracking_start_task(void) {
    if (velocity_tracking_task_handle == NULL) {
        BaseType_t result = xTaskCreate(
            velocity_tracking_task,
            "VelocityTrack",
            4096,  // 栈大小4096字节
            NULL,
            3,     // 中等优先级
            &velocity_tracking_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "速度跟踪模式任务创建成功");
        } else {
            ESP_LOGE(TAG, "速度跟踪模式任务创建失败");
        }
    } else {
        ESP_LOGW(TAG, "速度跟踪模式任务已在运行");
    }
}

/**
 * @brief 停止速度跟踪模式任务
 */
void velocity_tracking_stop_task(void) {
    if (velocity_tracking_task_handle != NULL) {
        vTaskDelete(velocity_tracking_task_handle);
        velocity_tracking_task_handle = NULL;
        ESP_LOGI(TAG, "速度跟踪模式任务已停止");
    }
}