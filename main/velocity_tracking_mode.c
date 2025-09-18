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
float velocity_tracking_lift_leg_torque = LIFT_LEG_TORQUE;    // 抬腿力矩
float velocity_tracking_lift_leg_speed = LIFT_LEG_SPEED;      // 抬腿速度
float velocity_tracking_drop_leg_torque = DROP_LEG_TORQUE;    // 放腿力矩
float velocity_tracking_drop_leg_speed = DROP_LEG_SPEED;      // 放腿速度

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

    // 初始化轮流工作周期管理
    velocity_tracking_context.current_work_cycle = WORK_CYCLE_MOTOR1; // 从1号电机开始
    velocity_tracking_context.cycle_completed = false;
    velocity_tracking_context.cycle_start_time = 0;

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
 * @brief 检查是否应该启用速度跟踪模式
 */
bool velocity_tracking_should_enable(position_ring_buffer_t *buffer) {
    if (buffer == NULL) {
        return false;
    }

    // 使用现有的波峰波谷差值检测函数
    float peak_valley_diff = detect_peak_valley_difference(buffer);

    if (peak_valley_diff < 0) {
        // 检测失败，尝试使用最大最小值方法
        float min_pos, max_pos;
        if (!position_ring_buffer_get_min_max(buffer, &min_pos, &max_pos)) {
            return false;
        }
        peak_valley_diff = max_pos - min_pos;
    }

    bool should_enable = peak_valley_diff >= VELOCITY_TRACKING_ENABLE_THRESHOLD;

    ESP_LOGD(TAG, "波峰波谷差值检测: %.3f (阈值: %.3f) -> %s",
             peak_valley_diff, VELOCITY_TRACKING_ENABLE_THRESHOLD,
             should_enable ? "启用" : "禁用");

    return should_enable;
}

/**
 * @brief 获取速度方向
 */
velocity_direction_t velocity_tracking_get_direction(float velocity) {
    if (fabsf(velocity) < VELOCITY_TRACKING_MIN_VELOCITY) {
        return VELOCITY_DIR_UNKNOWN;
    }
    return (velocity > 0) ? VELOCITY_DIR_POSITIVE : VELOCITY_DIR_NEGATIVE;
}

/**
 * @brief 检测速度方向变化并更新节律控制
 */
bool velocity_tracking_update_rhythm(rhythm_control_t *rhythm, float velocity, uint32_t timestamp, int motor_id) {
    if (rhythm == NULL) return false;

    // 如果检测被阻止，跳过处理
    if (rhythm->detection_blocked) {
        ESP_LOGD(TAG, "电机%d检测被阻止，跳过节律更新", motor_id);
        return false;
    }

    velocity_direction_t new_dir = velocity_tracking_get_direction(velocity);

    // 如果速度方向未知，跳过处理
    if (new_dir == VELOCITY_DIR_UNKNOWN) {
        return false;
    }

    // 更新当前方向
    rhythm->current_dir = new_dir;

    // 检测方向变化
    if (rhythm->last_dir != VELOCITY_DIR_UNKNOWN && rhythm->last_dir != new_dir) {
        // 发生了方向变化
        uint32_t time_interval = timestamp - rhythm->last_change_time;

        ESP_LOGI(TAG, "电机%d速度方向变化: %s -> %s, 时间间隔: %lu ms",
                 motor_id,
                 rhythm->last_dir == VELOCITY_DIR_POSITIVE ? "+v" : "-v",
                 new_dir == VELOCITY_DIR_POSITIVE ? "+v" : "-v",
                 time_interval);

        // 更新控制持续时间（使用上次的时间间隔+200ms）
        if (time_interval > 100 && time_interval < 10000) { // 合理的时间范围 0.1-10秒
            rhythm->control_duration_ms = time_interval + 200; // 添加200ms

            // 更新循环时间统计
            rhythm->last_cycle_times[rhythm->cycle_time_index] = time_interval;
            rhythm->cycle_time_index = (rhythm->cycle_time_index + 1) % 5;
            rhythm->total_cycles++;

            // 计算平均时间
            uint32_t sum = 0;
            for (int i = 0; i < 5; i++) {
                sum += rhythm->last_cycle_times[i];
            }
            rhythm->avg_cycle_time_ms = sum / 5;
        }

        // 决定要执行的动作
        motor_action_t new_action = MOTOR_ACTION_IDLE;
        if (motor_id == 1) {
            // 1号电机：-v抬腿，+v放腿
            new_action = (new_dir == VELOCITY_DIR_NEGATIVE) ? MOTOR_ACTION_LIFT_LEG : MOTOR_ACTION_DROP_LEG;
        } else if (motor_id == 2) {
            // 2号电机：+v抬腿，-v放腿
            new_action = (new_dir == VELOCITY_DIR_POSITIVE) ? MOTOR_ACTION_LIFT_LEG : MOTOR_ACTION_DROP_LEG;
        }

        // 执行动作
        rhythm->current_action = new_action;
        rhythm->action_start_time = timestamp;
        rhythm->action_active = true;

        ESP_LOGI(TAG, "电机%d开始执行%s，持续时间: %lu ms (间隔%lu + 200ms)",
                 motor_id,
                 new_action == MOTOR_ACTION_LIFT_LEG ? "抬腿" : "放腿",
                 rhythm->control_duration_ms, time_interval);

        // 记录这次变化时间
        rhythm->last_change_time = timestamp;
        rhythm->last_dir = new_dir;

        return true;
    } else if (rhythm->last_dir == VELOCITY_DIR_UNKNOWN) {
        // 第一次检测到有效速度方向
        rhythm->last_dir = new_dir;
        rhythm->last_change_time = timestamp;
        ESP_LOGI(TAG, "电机%d首次检测到速度方向: %s", motor_id, new_dir == VELOCITY_DIR_POSITIVE ? "+v" : "-v");
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
 * @brief 处理节律控制的定时动作
 */
bool velocity_tracking_process_rhythm_timing(rhythm_control_t *rhythm, int motor_id, uint32_t timestamp) {
    if (rhythm == NULL || !rhythm->action_active) {
        return false;
    }

    uint32_t elapsed_time = timestamp - rhythm->action_start_time;
    bool action_completed = false;
    bool force_switch = false;

    // 检查抬腿MIT超时（0.65秒限制）
    if (rhythm->current_action == MOTOR_ACTION_LIFT_LEG && elapsed_time >= LIFT_LEG_MAX_DURATION_MS) {
        // 抬腿超时，强制切换到放腿
        force_switch = true;
        ESP_LOGW(TAG, "电机%d抬腿MIT超时(%lu ms >= %d ms)，强制切换到放腿MIT",
                 motor_id+1, elapsed_time, LIFT_LEG_MAX_DURATION_MS);
    }

    // 检查正常动作时间是否到达
    if (elapsed_time >= rhythm->control_duration_ms) {
        action_completed = true;
    }

    // 如果抬腿超时或正常完成，处理动作结束
    if (force_switch || action_completed) {
        if (force_switch && rhythm->current_action == MOTOR_ACTION_LIFT_LEG) {
            // 抬腿超时，强制切换到放腿MIT
            rhythm->current_action = MOTOR_ACTION_DROP_LEG;
            rhythm->action_start_time = timestamp; // 重新计时

            // 抬腿超时后使用较短的放腿时间（600ms）
            uint32_t drop_duration = TIMEOUT_DROP_LEG_DURATION_MS;
            rhythm->control_duration_ms = drop_duration;

            // 更新速度方向为正向（放腿阶段）
            rhythm->current_dir = VELOCITY_DIR_POSITIVE;
            rhythm->last_dir = VELOCITY_DIR_POSITIVE;

            // 执行放腿动作
            velocity_tracking_execute_motor_action(motor_id, MOTOR_ACTION_DROP_LEG);

            ESP_LOGI(TAG, "电机%d强制执行放腿MIT，持续时间: %lu ms（超时后固定时间）", motor_id+1, drop_duration);

            return true; // 状态发生了变化
        } else {
            // 正常完成动作
            rhythm->action_active = false;
            rhythm->current_action = MOTOR_ACTION_IDLE;

            ESP_LOGI(TAG, "电机%d动作完成，耗时: %lu ms，切换到空闲状态", motor_id+1, elapsed_time);

            // 设置电机为空闲状态
            velocity_tracking_execute_motor_action(motor_id, MOTOR_ACTION_IDLE);

            // 如果完成的是放腿动作，立即进行交替切换
            if (rhythm->last_dir == VELOCITY_DIR_POSITIVE && motor_id == 0) {
                // 1号电机完成放腿，切换到休息，2号电机开始工作
                velocity_tracking_context.motor1_rhythm.is_resting = true;
                velocity_tracking_context.motor2_rhythm.is_resting = false;
                velocity_tracking_context.total_work_cycles++;
                ESP_LOGI(TAG, "1号电机完成放腿，进入休息状态，2号电机开始工作周期 (总周期数: %lu)",
                         velocity_tracking_context.total_work_cycles);
            } else if (rhythm->last_dir == VELOCITY_DIR_NEGATIVE && motor_id == 1) {
                // 2号电机完成放腿，切换到休息，1号电机开始工作
                velocity_tracking_context.motor2_rhythm.is_resting = true;
                velocity_tracking_context.motor1_rhythm.is_resting = false;
                velocity_tracking_context.total_work_cycles++;
                ESP_LOGI(TAG, "2号电机完成放腿，进入休息状态，1号电机开始工作周期 (总周期数: %lu)",
                         velocity_tracking_context.total_work_cycles);
            }

            return true; // 状态发生了变化
        }
    }

    return false; // 无变化
}

/**
 * @brief 更新速度跟踪模式（主要处理函数）
 */
bool velocity_tracking_mode_update(position_ring_buffer_t *buffer,
                                  float motor1_velocity,
                                  float motor2_velocity,
                                  uint32_t timestamp) {
    if (!velocity_tracking_mode_enabled || buffer == NULL) {
        return false;
    }

    bool state_changed = false;

    // 检查更新间隔
    if ((timestamp - velocity_tracking_context.last_update_time) < VELOCITY_TRACKING_UPDATE_INTERVAL_MS) {
        return false;
    }

    velocity_tracking_context.last_update_time = timestamp;

    // 检查是否应该启用速度跟踪 - 暂时注释掉启动条件
    // bool should_enable = velocity_tracking_should_enable(buffer);
    bool should_enable = true; // 暂时强制启用，跳过缓存区检查

    // 只有在ENABLED状态时才检查是否应该激活，一旦激活就保持激活状态
    if (should_enable && velocity_tracking_context.state == VELOCITY_TRACKING_ENABLED) {
        // 激活速度跟踪模式
        velocity_tracking_context.state = VELOCITY_TRACKING_ACTIVE;
        velocity_tracking_context.activation_count++;
        state_changed = true;
        ESP_LOGI(TAG, "速度跟踪模式激活 (激活次数: %lu) - 波峰波谷差值达到启动阈值", velocity_tracking_context.activation_count);
    }
    // 注意：一旦激活后，不再检查波峰波谷差值，保持激活状态
    // 如果需要停止，需要手动调用禁用函数

    // 如果处于激活状态，处理交替工作控制
    if (velocity_tracking_context.state == VELOCITY_TRACKING_ACTIVE) {
        bool motor1_rhythm_changed = false;
        bool motor2_rhythm_changed = false;

        // 交替工作逻辑：同时只有一个电机工作，另一个休息

        // 1号电机处理
        if (!velocity_tracking_context.motor1_rhythm.is_resting) {
            // 1号电机工作，2号电机休息
            velocity_tracking_context.motor1_rhythm.detection_blocked = false;
            velocity_tracking_context.motor2_rhythm.detection_blocked = true;

            motor1_rhythm_changed = velocity_tracking_update_rhythm(
                &velocity_tracking_context.motor1_rhythm, motor1_velocity, timestamp, 1);
        } else {
            // 1号电机休息，确保保持空闲状态
            if (velocity_tracking_context.motor1_rhythm.action_active) {
                velocity_tracking_context.motor1_rhythm.action_active = false;
                velocity_tracking_context.motor1_rhythm.current_action = MOTOR_ACTION_IDLE;
                velocity_tracking_execute_motor_action(0, MOTOR_ACTION_IDLE);
            }
        }

        // 2号电机处理
        if (!velocity_tracking_context.motor2_rhythm.is_resting) {
            // 2号电机工作，1号电机休息
            velocity_tracking_context.motor2_rhythm.detection_blocked = false;
            velocity_tracking_context.motor1_rhythm.detection_blocked = true;

            motor2_rhythm_changed = velocity_tracking_update_rhythm(
                &velocity_tracking_context.motor2_rhythm, motor2_velocity, timestamp, 2);
        } else {
            // 2号电机休息，确保保持空闲状态
            if (velocity_tracking_context.motor2_rhythm.action_active) {
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