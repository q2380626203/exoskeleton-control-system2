#include "continuous_torque_velocity_mode.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "position_recorder";

// 调试信息打印控制
static uint32_t last_debug_print_time = 0;
#define DEBUG_PRINT_INTERVAL_MS 2000  // 每2秒打印一次调试信息

// 可配置的运动模式参数
float climbing_mode_torque = 6.0f;  // 爬楼模式力矩大小 (Nm)

void position_ring_buffer_init(position_ring_buffer_t *buffer) {
    if (buffer == NULL) {
        ESP_LOGE(TAG, "缓存区指针为空");
        return;
    }

    memset(buffer->buffer, 0, sizeof(buffer->buffer));
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->last_recorded_position = 0.0f;
    buffer->initialized = true;

    ESP_LOGD(TAG, "环形缓存区初始化完成");
}

bool position_ring_buffer_add_if_changed(position_ring_buffer_t *buffer,
                                        float new_position,
                                        uint32_t timestamp) {
    if (buffer == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "缓存区未初始化");
        return false;
    }

    // 检查位置变化是否超过阈值
    float position_change = fabsf(new_position - buffer->last_recorded_position);

    if (position_change >= POSITION_CHANGE_THRESHOLD) {
        position_ring_buffer_add(buffer, new_position, timestamp);
        buffer->last_recorded_position = new_position;

        ESP_LOGD(TAG, "位置变化 %.3f 超过阈值，记录新位置: %.3f",
                position_change, new_position);
        return true;
    }

    return false;
}

void position_ring_buffer_add(position_ring_buffer_t *buffer,
                             float position,
                             uint32_t timestamp) {
    if (buffer == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "缓存区未初始化");
        return;
    }

    // 添加新数据到头部
    buffer->buffer[buffer->head].position = position;
    buffer->buffer[buffer->head].timestamp = timestamp;
    buffer->buffer[buffer->head].valid = true;

    // 更新头指针
    buffer->head = (buffer->head + 1) % POSITION_BUFFER_SIZE;

    // 如果缓存区已满，更新尾指针
    if (buffer->count == POSITION_BUFFER_SIZE) {
        buffer->tail = (buffer->tail + 1) % POSITION_BUFFER_SIZE;
    } else {
        buffer->count++;
    }

    ESP_LOGV(TAG, "添加位置点: %.3f (时间戳: %lu), 缓存区大小: %d",
            position, timestamp, buffer->count);
}

bool position_ring_buffer_get_latest(position_ring_buffer_t *buffer,
                                    position_point_t *point) {
    if (buffer == NULL || point == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "参数无效");
        return false;
    }

    if (buffer->count == 0) {
        return false;
    }

    // 最新数据在头指针的前一个位置
    uint16_t latest_index = (buffer->head - 1 + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE;
    *point = buffer->buffer[latest_index];

    return point->valid;
}

bool position_ring_buffer_get_at_index(position_ring_buffer_t *buffer,
                                      int16_t index,
                                      position_point_t *point) {
    if (buffer == NULL || point == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "参数无效");
        return false;
    }

    if (buffer->count == 0 || abs(index) >= buffer->count) {
        return false;
    }

    uint16_t actual_index;
    if (index >= 0) {
        // 正索引：0为最新，1为次新...
        actual_index = (buffer->head - 1 - index + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE;
    } else {
        // 负索引：-1为最旧，-2为次旧...
        actual_index = (buffer->tail - index - 1 + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE;
    }

    *point = buffer->buffer[actual_index];
    return point->valid;
}

uint16_t position_ring_buffer_get_count(position_ring_buffer_t *buffer) {
    if (buffer == NULL || !buffer->initialized) {
        return 0;
    }
    return buffer->count;
}

bool position_ring_buffer_is_empty(position_ring_buffer_t *buffer) {
    if (buffer == NULL || !buffer->initialized) {
        return true;
    }
    return buffer->count == 0;
}

bool position_ring_buffer_is_full(position_ring_buffer_t *buffer) {
    if (buffer == NULL || !buffer->initialized) {
        return false;
    }
    return buffer->count == POSITION_BUFFER_SIZE;
}

/**
 * @brief 检测波峰波谷差值
 * @param buffer 缓存区指针
 * @return 波峰波谷差值，失败返回-1
 */
float detect_peak_valley_difference(position_ring_buffer_t *buffer) {
    if (buffer == NULL || !buffer->initialized || buffer->count < 3) {
        return -1.0f; // 数据不足，至少需要3个点
    }

    float max_peak = -999.0f;
    float min_valley = 999.0f;
    bool found_peak = false;
    bool found_valley = false;

    // 遍历缓冲区寻找波峰和波谷
    for (uint16_t i = 1; i < buffer->count - 1; i++) {
        uint16_t prev_index = (buffer->tail + i - 1) % POSITION_BUFFER_SIZE;
        uint16_t curr_index = (buffer->tail + i) % POSITION_BUFFER_SIZE;
        uint16_t next_index = (buffer->tail + i + 1) % POSITION_BUFFER_SIZE;

        if (!buffer->buffer[prev_index].valid ||
            !buffer->buffer[curr_index].valid ||
            !buffer->buffer[next_index].valid) {
            continue;
        }

        float prev_pos = buffer->buffer[prev_index].position;
        float curr_pos = buffer->buffer[curr_index].position;
        float next_pos = buffer->buffer[next_index].position;

        // 检测波峰：当前点大于前后两点
        if (curr_pos > prev_pos && curr_pos > next_pos) {
            if (curr_pos > max_peak) {
                max_peak = curr_pos;
                found_peak = true;
            }
        }
        // 检测波谷：当前点小于前后两点
        else if (curr_pos < prev_pos && curr_pos < next_pos) {
            if (curr_pos < min_valley) {
                min_valley = curr_pos;
                found_valley = true;
            }
        }
    }

    // 如果找到了波峰和波谷，计算差值
    if (found_peak && found_valley) {
        float diff = max_peak - min_valley;
        ESP_LOGD(TAG, "检测到波峰: %.3f, 波谷: %.3f, 差值: %.3f",
                max_peak, min_valley, diff);
        return diff;
    }

    // 如果只找到波峰或波谷，使用端点值补充
    if (found_peak || found_valley) {
        // 获取首尾两个有效点作为参考
        float first_pos = buffer->buffer[buffer->tail].position;
        float last_pos = buffer->buffer[(buffer->head - 1 + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE].position;

        float effective_max = found_peak ? max_peak : fmaxf(first_pos, last_pos);
        float effective_min = found_valley ? min_valley : fminf(first_pos, last_pos);

        float diff = effective_max - effective_min;
        ESP_LOGD(TAG, "部分波峰波谷检测: 最大=%.3f, 最小=%.3f, 差值=%.3f",
                effective_max, effective_min, diff);
        return diff;
    }

    ESP_LOGD(TAG, "未检测到明显波峰波谷");
    return -1.0f; // 未找到波峰波谷
}

void position_ring_buffer_clear(position_ring_buffer_t *buffer) {
    if (buffer == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "缓存区未初始化");
        return;
    }

    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->last_recorded_position = 0.0f;

    ESP_LOGD(TAG, "环形缓存区已清空");
}

bool position_ring_buffer_get_min_max(position_ring_buffer_t *buffer,
                                      float *min_pos, float *max_pos) {
    if (buffer == NULL || min_pos == NULL || max_pos == NULL || !buffer->initialized) {
        ESP_LOGE(TAG, "参数无效或缓存区未初始化");
        return false;
    }

    if (buffer->count == 0) {
        ESP_LOGW(TAG, "缓存区为空，无法获取最大最小值");
        return false;
    }

    float min_value = buffer->buffer[buffer->tail].position;
    float max_value = buffer->buffer[buffer->tail].position;

    // 遍历所有有效数据
    for (uint16_t i = 0; i < buffer->count; i++) {
        uint16_t index = (buffer->tail + i) % POSITION_BUFFER_SIZE;
        if (buffer->buffer[index].valid) {
            float pos = buffer->buffer[index].position;
            if (pos < min_value) {
                min_value = pos;
            }
            if (pos > max_value) {
                max_value = pos;
            }
        }
    }

    *min_pos = min_value;
    *max_pos = max_value;

    ESP_LOGV(TAG, "缓存区最小值: %.3f, 最大值: %.3f", min_value, max_value);
    return true;
}

motion_mode_t detect_motion_mode(position_ring_buffer_t *buffer,
                                uint32_t current_timestamp,
                                motion_mode_state_t *state) {
    if (buffer == NULL || !buffer->initialized || state == NULL) {
        ESP_LOGE(TAG, "缓存区未初始化");
        return MOTION_MODE_STATIC;
    }

    // 检查是否为空或数据不足
    if (buffer->count == 0) {
        ESP_LOGD(TAG, "缓存区为空，默认为静止模式");
        return MOTION_MODE_STATIC;
    }

    // 获取最新的位置点
    position_point_t latest_point;
    if (!position_ring_buffer_get_latest(buffer, &latest_point)) {
        ESP_LOGW(TAG, "无法获取最新位置点");
        return MOTION_MODE_STATIC;
    }

    // 检查是否超过静止超时时间（3秒没有更新）
    uint32_t time_since_last_update = current_timestamp - latest_point.timestamp;
    if (time_since_last_update > STATIC_TIMEOUT_MS) {
        ESP_LOGD(TAG, "超过%dms无位置更新，切换到静止模式", STATIC_TIMEOUT_MS);
        return MOTION_MODE_STATIC;
    }

    // 波峰波谷检测算法
    float peak_valley_diff = detect_peak_valley_difference(buffer);

    // 如果检测失败，使用最大最小值作为备用
    if (peak_valley_diff < 0) {
        float min_pos, max_pos;
        if (!position_ring_buffer_get_min_max(buffer, &min_pos, &max_pos)) {
            ESP_LOGW(TAG, "无法获取位置范围");
            return MOTION_MODE_STATIC;
        }
        peak_valley_diff = max_pos - min_pos;
        ESP_LOGD(TAG, "使用最大最小值备用检测: %.3f", peak_valley_diff);
    }

    float position_range = peak_valley_diff;

    // 控制调试信息打印频率
    bool should_print_debug = (current_timestamp - last_debug_print_time) >= DEBUG_PRINT_INTERVAL_MS;

    if (should_print_debug) {
        ESP_LOGI(TAG, "【运动检测】波峰波谷差值: %.3f (数据点数: %d)",
                 position_range, buffer->count);
        last_debug_print_time = current_timestamp;

        // 如果是爬楼模式但差值小于阈值，输出详细调试信息
        if (position_range < MODE_DETECTION_THRESHOLD && position_range >= STATIC_DETECTION_THRESHOLD) {
            ESP_LOGI(TAG, "【调试】差值 %.3f 在行走范围内，但可能存在误判", position_range);

            // 输出最近5个位置点用于调试
            ESP_LOGI(TAG, "【缓存区调试】最近的位置点:");
            int points_to_show = (buffer->count < 5) ? buffer->count : 5;
            for (int i = 0; i < points_to_show; i++) {
                position_point_t point;
                if (position_ring_buffer_get_at_index(buffer, i, &point)) {
                    ESP_LOGI(TAG, "  位置[%d]: %.3f (时间戳: %lu)", i, point.position, point.timestamp);
                }
            }
        }
    } else {
        // 非调试打印时间，使用更低级别的日志
        ESP_LOGD(TAG, "【运动检测】波峰波谷差值: %.3f (数据点数: %d)",
                 position_range, buffer->count);
    }

    // 基于纯位置数据的运动模式判断
    if (position_range < STATIC_DETECTION_THRESHOLD) {
        // 静止模式：重置爬楼降档计数器
        state->climbing_downgrade_count = 0;
        ESP_LOGI(TAG, "【模式判定】位置变化 %.3f < %.3f，判定为静止模式",
                 position_range, STATIC_DETECTION_THRESHOLD);
        return MOTION_MODE_STATIC;
    } else if (position_range >= MODE_DETECTION_THRESHOLD) {
        // 爬楼模式：重置降档计数器
        state->climbing_downgrade_count = 0;
        ESP_LOGI(TAG, "【模式判定】位置变化 %.3f >= %.3f，判定为爬楼模式",
                 position_range, MODE_DETECTION_THRESHOLD);
        return MOTION_MODE_CLIMBING;
    } else {
        // 中等范围：需要根据当前模式和阈值进行精细判断
        if (state->current_mode == MOTION_MODE_CLIMBING) {
            // 当前是爬楼模式，检查是否满足降档条件（小于0.6）
            if (position_range < CLIMBING_DOWNGRADE_THRESHOLD) {
                state->climbing_downgrade_count++;
                ESP_LOGI(TAG, "【爬楼降档】位置变化 %.3f < %.3f，降档计数: %d/6",
                         position_range, CLIMBING_DOWNGRADE_THRESHOLD, state->climbing_downgrade_count);

                if (state->climbing_downgrade_count >= 6) {
                    // 连续6次低于降档阈值，执行降档
                    state->climbing_downgrade_count = 0; // 重置计数器
                    ESP_LOGI(TAG, "【模式判定】连续6次低于降档阈值，爬楼模式降档到行走模式");
                    return MOTION_MODE_WALKING;
                } else {
                    // 还未达到6次，保持爬楼模式
                    ESP_LOGI(TAG, "【爬楼保持】还需 %d 次确认才能降档，保持爬楼模式",
                             6 - state->climbing_downgrade_count);
                    return MOTION_MODE_CLIMBING;
                }
            } else {
                // 在0.6-0.7之间，重置计数器，保持爬楼模式
                state->climbing_downgrade_count = 0;
                ESP_LOGI(TAG, "【爬楼保持】位置变化 %.3f 在保持范围 %.3f - %.3f，保持爬楼模式",
                         position_range, CLIMBING_DOWNGRADE_THRESHOLD, MODE_DETECTION_THRESHOLD);
                return MOTION_MODE_CLIMBING;
            }
        } else {
            // 不是从爬楼模式来的，直接判定为行走模式
            state->climbing_downgrade_count = 0;
            ESP_LOGI(TAG, "【模式判定】位置变化 %.3f 在 %.3f - %.3f 之间，判定为行走模式",
                     position_range, STATIC_DETECTION_THRESHOLD, MODE_DETECTION_THRESHOLD);
            return MOTION_MODE_WALKING;
        }
    }
}

void get_motor_params(motion_mode_t mode, int motor_id, motor_params_t *params) {
    if (params == NULL) {
        ESP_LOGE(TAG, "电机参数指针为空");
        return;
    }

    // 检查电机ID有效性
    if (motor_id != 1 && motor_id != 2) {
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return;
    }

    switch (mode) {
        case MOTION_MODE_STATIC:
            // 静止模式：全部设为0
            params->velocity = 0;
            params->torque = 0.0f;
            params->kd = 0;
            ESP_LOGD(TAG, "电机%d静止模式参数: 速度=%d, 力矩=%.1f, kd=%d",
                     motor_id, params->velocity, params->torque, params->kd);
            break;

        case MOTION_MODE_WALKING:
            // 行走模式
            if (motor_id == 1) {
                params->velocity = -2;
                params->torque = -1.5f;
                params->kd = 1;
            } else { // motor_id == 2
                params->velocity = 2;
                params->torque = 1.5f;
                params->kd = 1;
            }
            ESP_LOGD(TAG, "电机%d行走模式参数: 速度=%d, 力矩=%.1f, kd=%d",
                     motor_id, params->velocity, params->torque, params->kd);
            break;

        case MOTION_MODE_CLIMBING:
            // 爬楼模式
            if (motor_id == 1) {
                params->velocity = -2;
                params->torque = -climbing_mode_torque;
                params->kd = 1;
            } else { // motor_id == 2
                params->velocity = 2;
                params->torque = climbing_mode_torque;
                params->kd = 1;
            }
            ESP_LOGD(TAG, "电机%d爬楼模式参数: 速度=%d, 力矩=%.1f, kd=%d",
                     motor_id, params->velocity, params->torque, params->kd);
            break;

        default:
            ESP_LOGW(TAG, "未知运动模式: %d，使用静止模式参数", mode);
            params->velocity = 0;
            params->torque = 0.0f;
            params->kd = 0;
            break;
    }
}

void torque_gradient_init(torque_gradient_t *gradient) {
    if (gradient == NULL) {
        ESP_LOGE(TAG, "力矩渐变指针为空");
        return;
    }

    gradient->current_torque = 0.0f;
    gradient->target_torque = 0.0f;
    gradient->last_update_time = 0;
    gradient->is_increasing = false;

    ESP_LOGD(TAG, "力矩渐变控制初始化完成");
}

float update_torque_gradient(torque_gradient_t *gradient,
                             float target_torque,
                             uint32_t current_timestamp) {
    if (gradient == NULL) {
        ESP_LOGE(TAG, "力矩渐变指针为空");
        return 0.0f;
    }

    // 如果目标力矩改变，重新开始渐变过程
    if (gradient->target_torque != target_torque) {
        gradient->target_torque = target_torque;
        gradient->last_update_time = current_timestamp;
        gradient->is_increasing = (target_torque > gradient->current_torque);
        ESP_LOGI(TAG, "力矩渐变目标更新: %.1f -> %.1f", gradient->current_torque, target_torque);
    }

    // 如果已经达到目标值，直接返回
    if (fabs(gradient->current_torque - gradient->target_torque) < 0.1f) {
        gradient->current_torque = gradient->target_torque;
        return gradient->current_torque;
    }

    // 检查是否需要更新力矩值
    uint32_t time_interval = gradient->is_increasing ? TORQUE_INCREASE_INTERVAL_MS : TORQUE_DECREASE_INTERVAL_MS;
    if (current_timestamp - gradient->last_update_time >= time_interval) {
        float step = gradient->is_increasing ? TORQUE_INCREASE_STEP : TORQUE_DECREASE_STEP;

        if (gradient->is_increasing) {
            gradient->current_torque += step;
            if (gradient->current_torque > gradient->target_torque) {
                gradient->current_torque = gradient->target_torque;
            }
        } else {
            gradient->current_torque -= step;
            if (gradient->current_torque < gradient->target_torque) {
                gradient->current_torque = gradient->target_torque;
            }
        }

        gradient->last_update_time = current_timestamp;
        ESP_LOGV(TAG, "力矩渐变更新: %.1f (目标: %.1f)", gradient->current_torque, gradient->target_torque);
    }

    return gradient->current_torque;
}

void motion_mode_state_init(motion_mode_state_t *state) {
    if (state == NULL) {
        ESP_LOGE(TAG, "状态管理指针为空");
        return;
    }

    state->current_mode = MOTION_MODE_STATIC;
    state->previous_mode = MOTION_MODE_STATIC;
    state->mode_start_timestamp = 0;
    state->is_continuous_mode = false;

    // 初始化力矩渐变控制
    torque_gradient_init(&state->motor1_torque);
    torque_gradient_init(&state->motor2_torque);

    // 初始化模式稳定性控制
    state->last_mode_change_time = 0;
    state->static_confirm_start_time = 0;
    state->in_static_confirmation = false;
    state->last_position_range = 0.0f;

    // 初始化爬楼模式降档控制
    state->climbing_downgrade_count = 0;

    ESP_LOGD(TAG, "运动模式状态管理初始化完成");
}

void update_motion_mode_and_get_params(motion_mode_state_t *state,
                                      motion_mode_t new_mode,
                                      uint32_t current_timestamp,
                                      int motor_id,
                                      motor_params_t *params) {
    if (state == NULL || params == NULL) {
        ESP_LOGE(TAG, "参数指针为空");
        return;
    }

    motion_mode_t actual_mode = state->current_mode;

    // 模式稳定性逻辑：防止惯性导致的频繁切换
    if (new_mode != state->current_mode) {
        // 检查是否在模式锁定期内（刚切换后的一段时间内不允许再切换）
        uint32_t time_since_last_change = current_timestamp - state->last_mode_change_time;

        if (time_since_last_change < MODE_LOCK_DURATION_MS) {
            ESP_LOGI(TAG, "【模式锁定】距离上次切换仅%dms（需要%dms），忽略切换请求 %d -> %d",
                     time_since_last_change, MODE_LOCK_DURATION_MS, state->current_mode, new_mode);
            actual_mode = state->current_mode; // 保持当前模式
        }
        // 特殊处理：从运动模式切换到静止模式需要确认
        else if (state->current_mode != MOTION_MODE_STATIC && new_mode == MOTION_MODE_STATIC) {
            if (!state->in_static_confirmation) {
                // 在开始静止确认前，同步当前力矩状态
                motor_params_t current_params;
                get_motor_params(state->current_mode, motor_id, &current_params);

                torque_gradient_t *gradient = (motor_id == 1) ? &state->motor1_torque : &state->motor2_torque;
                gradient->current_torque = current_params.torque;
                ESP_LOGI(TAG, "电机%d开始静止确认前同步力矩: %.1f", motor_id, gradient->current_torque);

                // 开始静止确认过程
                state->in_static_confirmation = true;
                state->static_confirm_start_time = current_timestamp;
                ESP_LOGD(TAG, "【静止确认】开始静止确认过程，需要持续%dms", STATIC_CONFIRM_DURATION_MS);
                actual_mode = MOTION_MODE_STATIC; // 立即切换到静止模式让电机停止
            } else {
                // 检查是否已经确认足够时间
                uint32_t confirm_duration = current_timestamp - state->static_confirm_start_time;
                if (confirm_duration >= STATIC_CONFIRM_DURATION_MS) {
                    // 确认切换到静止模式
                    ESP_LOGD(TAG, "静止确认完成，切换到静止模式");
                    actual_mode = MOTION_MODE_STATIC;
                    state->in_static_confirmation = false;

                    // 更新状态
                    state->previous_mode = state->current_mode;
                    state->current_mode = MOTION_MODE_STATIC;
                    state->mode_start_timestamp = current_timestamp;
                    state->last_mode_change_time = current_timestamp;
                    state->is_continuous_mode = false;
                } else {
                    ESP_LOGD(TAG, "【静止确认】已确认%dms，还需%dms",
                             confirm_duration, STATIC_CONFIRM_DURATION_MS - confirm_duration);
                    actual_mode = MOTION_MODE_STATIC; // 确认期间保持静止模式
                }
            }
        }
        // 其他模式切换（运动到运动，静止到运动）
        else {
            // 如果正在静止确认中，但检测到新的运动，取消确认
            if (state->in_static_confirmation) {
                ESP_LOGD(TAG, "检测到新运动，取消静止确认");
                state->in_static_confirmation = false;
            }

            // 在模式切换前，同步当前力矩渐变状态
            if (state->current_mode != MOTION_MODE_STATIC) {
                // 获取当前模式的基础力矩作为渐变起点
                motor_params_t current_params;
                get_motor_params(state->current_mode, motor_id, &current_params);

                torque_gradient_t *gradient = (motor_id == 1) ? &state->motor1_torque : &state->motor2_torque;
                gradient->current_torque = current_params.torque;
                ESP_LOGI(TAG, "电机%d模式切换前同步当前力矩: %.1f", motor_id, gradient->current_torque);
            }

            // 执行模式切换
            ESP_LOGI(TAG, "【模式切换】运动模式切换: %d -> %d", state->current_mode, new_mode);
            state->previous_mode = state->current_mode;
            state->current_mode = new_mode;
            state->mode_start_timestamp = current_timestamp;
            state->last_mode_change_time = current_timestamp;
            state->is_continuous_mode = false;
            actual_mode = new_mode;
        }
    } else {
        // 模式没有变化
        if (state->in_static_confirmation && new_mode != MOTION_MODE_STATIC) {
            // 如果正在静止确认，但又检测到运动，取消确认
            ESP_LOGD(TAG, "静止确认期间检测到运动，取消确认");
            state->in_static_confirmation = false;
        }

        // 检查是否进入持续模式
        if (!state->is_continuous_mode) {
            state->is_continuous_mode = true;
            ESP_LOGD(TAG, "模式%d进入持续状态", state->current_mode);
        }

        actual_mode = state->current_mode;
    }

    // 获取基础电机参数（目标值）
    get_motor_params(actual_mode, motor_id, params);

    // 应用力矩渐变控制
    torque_gradient_t *gradient = (motor_id == 1) ? &state->motor1_torque : &state->motor2_torque;
    float target_torque = params->torque;
    float actual_torque = update_torque_gradient(gradient, target_torque, current_timestamp);
    params->torque = actual_torque;

    // 如果是持续模式（行走或爬楼），并且不是静止模式，设置kd为0
    if (state->is_continuous_mode &&
        (actual_mode == MOTION_MODE_WALKING || actual_mode == MOTION_MODE_CLIMBING)) {
        params->kd = 0;
        ESP_LOGD(TAG, "持续%s模式，电机%d的kd参数设为0",
                 actual_mode == MOTION_MODE_WALKING ? "行走" : "爬楼",
                 motor_id);
    }

    ESP_LOGD(TAG, "【最终参数】电机%d当前模式=%d, 速度=%d, 力矩=%.1f (目标=%.1f), kd=%d",
             motor_id, actual_mode, params->velocity, params->torque, target_torque, params->kd);
}

