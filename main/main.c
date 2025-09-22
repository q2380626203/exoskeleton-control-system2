/**
 * @file main.c
 * @brief 外骨骼WiFi控制系统主程序入口
 * 
 * 这是一个ESP32外骨骼控制系统，集成了WiFi热点、电机控制、
 * 语音反馈和按键检测功能。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "system_init.h"
#include "wifi_softap_module.h"
#include "rs01_motor.h"
// #include "uart_motor_transmit.h"  // 已禁用，让出UART2给语音模块
#include "motor_web_control.h"
#include "alternating_speed.h"
#include "continuous_torque_velocity_mode.h"
#include "velocity_tracking_mode.h"

// 外部变量声明
extern MotorDataCallback data_callback;

// RS01电机库的外部变量
extern MI_Motor motors[2];

// 交替速度模块的外部变量
extern bool alternating_speed_enabled;

static const char *TAG = "exoskeleton_main";

// 全局速度控制变量
float motor_target_speed[2] = {0.0f, 0.0f};  // 目标速度值数组，索引对应电机ID-1 (rad/s)
bool motor_speed_control_enabled = false;

// 统一的电机控制参数结构体 - 避免重复定义
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

// 全局电机控制参数（数组索引对应电机ID-1）
motor_control_params_t motor_params[2] = {{0}};

// 参数变化检测缓存
motor_control_params_t sent_params[2] = {{0}};


// 串口数据解析任务相关变量
static TaskHandle_t uart_parse_task_handle = NULL;
bool uart_parse_enabled = true;

// 电机数据传输任务相关变量 - 已禁用
// static TaskHandle_t motor_transmit_task_handle = NULL;

// 运动模式检测任务相关变量
static TaskHandle_t motion_detection_task_handle = NULL;

// 参数变化检测阈值
#define PARAM_CHANGE_THRESHOLD 0.001f

// 位置记录缓存区（为每个电机独立记录）
position_ring_buffer_t position_recorder_motor1;  // 1号电机位置记录
position_ring_buffer_t position_recorder_motor2;  // 2号电机位置记录

// 运动模式状态管理
motion_mode_state_t motion_state;

// 运动模式检测功能开关
bool motion_mode_detection_enabled = false;

// 速度跟踪模式功能开关
bool velocity_tracking_mode_active = false;

// 导出统一的电机参数结构体供其他模块使用
extern motor_control_params_t motor_params[2];

// 函数声明
void set_motor_params(int motor_id, float torque, float position, float speed, float kp, float kd);
void unified_motor_control(int motor_id, const motor_control_params_t* params);

// 速度跟踪模式控制函数
void enable_velocity_tracking_mode(void);
void disable_velocity_tracking_mode(void);
bool is_velocity_tracking_mode_active(void);
void reset_velocity_tracking_mode(void);







/**
 * @brief 电机数据更新回调函数
 * @param motor 更新的电机数据指针
 */
void motor_data_update_callback(MI_Motor* motor) {
    ESP_LOGD(TAG, "电机%d数据更新: 位置=%.3f, 速度=%.3f, 电流=%.3f, 温度=%.1f°C",
             motor->id, motor->position, motor->velocity, motor->current, motor->temperature);

    // 检查电机错误状态
    if (motor->error != 0) {
        ESP_LOGW(TAG, "电机%d错误状态: 0x%02X", motor->id, motor->error);
    }


    // 这里可以添加其他数据处理逻辑，比如：
    // - 更新电机状态到全局变量
    // - 触发安全保护机制
    // - 记录数据到日志等
}


/**
 * @brief 串口数据解析任务
 * @param pvParameters 任务参数（未使用）
 */
void UART_Parse_Task(void* pvParameters) {
    ESP_LOGI(TAG, "串口数据解析任务启动");

    while (uart_parse_enabled) {
        // 调用RS01库的串口数据处理函数
        handle_uart_rx();

        // 短暂延时，避免占用过多CPU时间
        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms延时，200Hz解析频率
    }
    
    ESP_LOGI(TAG, "串口数据解析任务退出");
    uart_parse_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 运动模式检测任务
 * @param pvParameters 任务参数（未使用）
 */
void Motion_Detection_Task(void* pvParameters) {
    ESP_LOGI(TAG, "运动模式检测任务启动");

    TickType_t last_position_check = xTaskGetTickCount();
    TickType_t last_mode_check = xTaskGetTickCount();
    TickType_t last_torque_update = xTaskGetTickCount();
    const TickType_t position_check_interval = pdMS_TO_TICKS(20);  // 每20ms检查一次位置
    const TickType_t mode_check_interval = pdMS_TO_TICKS(1000);    // 每1秒进行一次模式检测
    const TickType_t torque_update_interval = pdMS_TO_TICKS(100);  // 每100ms更新一次力矩渐变

    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        uint32_t timestamp = current_time * portTICK_PERIOD_MS;

        // 检查是否需要位置记录
        // 1. 运动检测模式启用时需要记录
        // 2. velocity_tracking处于ENABLED状态时需要记录（用于升档判断）
        // 3. velocity_tracking处于ACTIVE状态时不需要记录（避免干扰交替工作）
        bool need_for_motion_detection = motion_mode_detection_enabled;
        bool need_for_velocity_tracking = velocity_tracking_mode_enabled &&
                                         (velocity_tracking_mode_get_state() == VELOCITY_TRACKING_ENABLED);
        bool need_position_recording = need_for_motion_detection || need_for_velocity_tracking;

        if (need_position_recording) {
            // 检查位置记录（同时记录两个电机的位置数据）
            if ((current_time - last_position_check) >= position_check_interval) {
                // 记录1号电机位置变化
                position_ring_buffer_add_if_changed(&position_recorder_motor1, motors[0].position, timestamp);
                // 记录2号电机位置变化
                position_ring_buffer_add_if_changed(&position_recorder_motor2, motors[1].position, timestamp);
                last_position_check = current_time;
            }

        }

        if (motion_mode_detection_enabled) {
            // 运动模式检测和参数更新
            if ((current_time - last_mode_check) >= mode_check_interval) {
                motion_mode_t detected_mode = detect_motion_mode(&position_recorder_motor1, timestamp, &motion_state);

                // 检查是否需要更新模式
                if (detected_mode != motion_state.current_mode) {
                    ESP_LOGI(TAG, "运动模式切换: %s",
                             detected_mode == MOTION_MODE_STATIC ? "静止" :
                             detected_mode == MOTION_MODE_WALKING ? "行走" : "爬楼");

                    // 如果检测到静止模式，清理位置缓存区避免旧数据影响
                    if (detected_mode == MOTION_MODE_STATIC) {
                        position_ring_buffer_clear(&position_recorder_motor1);
                        position_ring_buffer_clear(&position_recorder_motor2);
                        ESP_LOGI(TAG, "【静止确认】清理位置缓存区，避免旧数据影响");
                    }
                }

                // 更新两个电机的参数（包括力矩渐变）
                motor_params_t motor1_params, motor2_params;
                update_motion_mode_and_get_params(&motion_state, detected_mode, timestamp, 1, &motor1_params);
                update_motion_mode_and_get_params(&motion_state, detected_mode, timestamp, 2, &motor2_params);

                // 应用电机参数到统一结构体
                motor_params[0].speed = (float)motor1_params.velocity;
                motor_params[0].torque = motor1_params.torque;
                motor_params[0].kd = (float)motor1_params.kd;
                motor_params[0].position = 0.0f; // 运动检测模式位置参数为0
                motor_params[0].kp = 0.0f;       // 运动检测模式kp参数为0

                motor_params[1].speed = (float)motor2_params.velocity;
                motor_params[1].torque = motor2_params.torque;
                motor_params[1].kd = (float)motor2_params.kd;
                motor_params[1].position = 0.0f; // 运动检测模式位置参数为0
                motor_params[1].kp = 0.0f;       // 运动检测模式kp参数为0

                // 调试：显示即将发送的参数
                if (detected_mode == MOTION_MODE_STATIC) {
                    ESP_LOGI(TAG, "【静止参数】电机1: 速度=%.1f, 力矩=%.1f, kd=%.1f",
                             motor_params[0].speed, motor_params[0].torque, motor_params[0].kd);
                    ESP_LOGI(TAG, "【静止参数】电机2: 速度=%.1f, 力矩=%.1f, kd=%.1f",
                             motor_params[1].speed, motor_params[1].torque, motor_params[1].kd);
                }

                // 立即发送参数给电机
                unified_motor_control(0, &motor_params[0]);
                unified_motor_control(1, &motor_params[1]);

                last_mode_check = current_time;
            }

            // 力矩渐变更新（高频率更新以实现平滑渐变）
            if ((current_time - last_torque_update) >= torque_update_interval) {
                motor_params_t motor1_params, motor2_params;

                // 在静止确认期间，使用静止模式而不是当前模式
                motion_mode_t update_mode = motion_state.in_static_confirmation ? MOTION_MODE_STATIC : motion_state.current_mode;

                update_motion_mode_and_get_params(&motion_state, update_mode, timestamp, 1, &motor1_params);
                update_motion_mode_and_get_params(&motion_state, update_mode, timestamp, 2, &motor2_params);

                // 只更新力矩值（速度和kd保持不变）
                motor_params[0].torque = motor1_params.torque;
                motor_params[1].torque = motor2_params.torque;

                // 立即发送更新的力矩给电机
                unified_motor_control(0, &motor_params[0]);
                unified_motor_control(1, &motor_params[1]);

                last_torque_update = current_time;
            }
        }

        // 速度跟踪模式更新
        if (velocity_tracking_mode_active && velocity_tracking_mode_is_enabled()) {
            // 获取当前电机速度
            float motor1_velocity = motors[0].velocity;
            float motor2_velocity = motors[1].velocity;

            // 更新速度跟踪模式（传入双电机缓存区）
            velocity_tracking_mode_update(&position_recorder_motor1, &position_recorder_motor2, motor1_velocity, motor2_velocity, timestamp);
        }

        // 任务延时
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms延时
    }

    ESP_LOGI(TAG, "运动模式检测任务退出");
    motion_detection_task_handle = NULL;
    vTaskDelete(NULL);
}


/**
 * @brief 启动串口数据解析任务
 */
void Start_UART_Parse_Task(void) {
    if (uart_parse_task_handle == NULL) {
        uart_parse_enabled = true;
        
        BaseType_t result = xTaskCreate(
            UART_Parse_Task,
            "UARTParse",
            4096,  // 栈大小增加到4096字节
            NULL,
            5,     // 优先级
            &uart_parse_task_handle
        );
        
        if (result == pdPASS) {
            ESP_LOGI(TAG, "串口数据解析任务创建成功");
        } else {
            ESP_LOGE(TAG, "串口数据解析任务创建失败");
            uart_parse_enabled = false;
        }
    } else {
        ESP_LOGW(TAG, "串口数据解析任务已在运行");
    }
}

/**
 * @brief 停止串口数据解析任务
 */
void Stop_UART_Parse_Task(void) {
    if (uart_parse_task_handle != NULL) {
        uart_parse_enabled = false;
        ESP_LOGI(TAG, "请求停止串口数据解析任务");
        // 任务会自然退出并清理自己
    }
}

/**
 * @brief 启动运动模式检测任务
 */
void Start_Motion_Detection_Task(void) {
    if (motion_detection_task_handle == NULL) {
        BaseType_t result = xTaskCreate(
            Motion_Detection_Task,
            "MotionDetect",
            4096,  // 栈大小4096字节
            NULL,
            3,     // 中等优先级
            &motion_detection_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "运动模式检测任务创建成功");
        } else {
            ESP_LOGE(TAG, "运动模式检测任务创建失败");
        }
    } else {
        ESP_LOGW(TAG, "运动模式检测任务已在运行");
    }
}

/**
 * @brief 停止运动模式检测任务
 */
void Stop_Motion_Detection_Task(void) {
    if (motion_detection_task_handle != NULL) {
        vTaskDelete(motion_detection_task_handle);
        motion_detection_task_handle = NULL;
        ESP_LOGI(TAG, "运动模式检测任务已停止");
    }
}


/**
 * @brief 检查电机参数是否发生变化
 * @param motor_id 电机ID (0或1，对应电机1和电机2)
 * @param params 新的电机参数
 * @return true 参数发生变化，false 参数未变化
 */
bool motor_params_changed(int motor_id, const motor_control_params_t* params) {
    if (motor_id < 0 || motor_id >= 2 || params == NULL) return false;

    const float threshold = 0.001f; // 参数变化阈值

    return (fabsf(params->torque - sent_params[motor_id].torque) > threshold) ||
           (fabsf(params->position - sent_params[motor_id].position) > threshold) ||
           (fabsf(params->speed - sent_params[motor_id].speed) > threshold) ||
           (fabsf(params->kp - sent_params[motor_id].kp) > threshold) ||
           (fabsf(params->kd - sent_params[motor_id].kd) > threshold);
}

/**
 * @brief 更新已发送参数缓存
 * @param motor_id 电机ID (0或1，对应电机1和电机2)
 * @param params 已发送的参数
 */
void update_sent_params(int motor_id, const motor_control_params_t* params) {
    if (motor_id < 0 || motor_id >= 2 || params == NULL) return;

    sent_params[motor_id] = *params;
}

/**
 * @brief 统一的电机控制函数 - 集中调用Motor_ControlMode
 * @param motor_id 电机ID (0或1，对应电机1和电机2)
 * @param params 电机控制参数
 */
void unified_motor_control(int motor_id, const motor_control_params_t* params) {
    if (motor_id < 0 || motor_id >= 2 || params == NULL) {
        ESP_LOGE(TAG, "无效的电机ID或参数: motor_id=%d", motor_id);
        return;
    }

    // 检查参数是否发生变化，只有发生变化时才发送控制指令
    if (!motor_params_changed(motor_id, params)) {
        return; // 参数未变化，跳过发送
    }

    // 临界区保护，避免多任务并发访问
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    // 调用实际的电机控制函数
    Motor_ControlMode(&motors[motor_id], params->torque, params->position,
                      params->speed, params->kp, params->kd);

    portEXIT_CRITICAL(&mux);

    // 更新已发送参数缓存
    update_sent_params(motor_id, params);

    ESP_LOGI(TAG, "电机%d控制更新: 力矩=%.2f, 位置=%.2f, 速度=%.2f, Kp=%.2f, Kd=%.2f",
             motor_id+1, params->torque, params->position, params->speed,
             params->kp, params->kd);
}

/**
 * @brief 设置电机控制参数
 * @param motor_id 电机ID (0或1，对应电机1和电机2)
 * @param torque 力矩参数
 * @param position 位置参数
 * @param speed 速度参数
 * @param kp Kp参数
 * @param kd Kd参数
 */
void set_motor_params(int motor_id, float torque, float position, float speed, float kp, float kd) {
    if (motor_id < 0 || motor_id >= 2) {
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return;
    }

    motor_params[motor_id].torque = torque;
    motor_params[motor_id].position = position;
    motor_params[motor_id].speed = speed;
    motor_params[motor_id].kp = kp;
    motor_params[motor_id].kd = kd;

    // 立即执行控制
    unified_motor_control(motor_id, &motor_params[motor_id]);
}




/*
 * 电机数据传输任务 - 已禁用，让出UART2给语音模块
 * @brief 启动电机数据传输任务

void Start_Motor_Data_Transmit(void) {
    if (motor_transmit_task_handle == NULL) {
        BaseType_t result = xTaskCreate(
            motor_data_transmit_task,
            "MotorDataTx",
            2048,
            NULL,
            4,  // 中等优先级
            &motor_transmit_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "电机数据传输任务创建成功");
        } else {
            ESP_LOGE(TAG, "电机数据传输任务创建失败");
        }
    } else {
        ESP_LOGW(TAG, "电机数据传输任务已在运行");
    }
}
*/

/**
 * @brief 应用程序主入口函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "外骨骼WiFi控制系统启动中...");
    
    // 初始化整个系统
    ESP_ERROR_CHECK(system_init_all());
    
    // 初始化位置记录缓存区（每个电机独立）
    position_ring_buffer_init(&position_recorder_motor1);
    position_ring_buffer_init(&position_recorder_motor2);
    ESP_LOGI(TAG, "位置记录缓存区初始化完成");

    // 初始化运动模式状态管理
    motion_mode_state_init(&motion_state);
    ESP_LOGI(TAG, "运动模式状态管理初始化完成");

    // 更新电机数据回调函数为我们自定义的回调
    data_callback = motor_data_update_callback;
    ESP_LOGI(TAG, "已更新电机数据回调函数");

    // 启动串口数据解析任务 (UART已在system_init_all()中初始化)
    ESP_LOGI(TAG, "启动RS01电机数据解析任务...");
    Start_UART_Parse_Task();



    // 初始化并启动电机数据传输功能 - 已禁用，让出UART2给语音模块
    // ESP_LOGI(TAG, "初始化电机数据传输...");
    // uart_motor_transmit_init();
    // Start_Motor_Data_Transmit();

    // 启动运动模式检测任务
    ESP_LOGI(TAG, "启动运动模式检测任务...");
    Start_Motion_Detection_Task();

    // 初始化速度跟踪模式
    ESP_LOGI(TAG, "初始化速度跟踪模式...");
    velocity_tracking_mode_init();
    velocity_tracking_start_task();

    // 系统启动完成后默认启用速度跟踪模式
    ESP_LOGI(TAG, "启用速度跟踪模式...");
    velocity_tracking_mode_active = true;
    velocity_tracking_mode_set_enabled(true);
    ESP_LOGI(TAG, "速度跟踪模式已默认启用");

    // 初始化电机控制网页模块
    ESP_LOGI(TAG, "初始化电机控制网页...");
    esp_err_t web_result = motor_web_control_init();
    if (web_result == ESP_OK) {
        ESP_LOGI(TAG, "电机控制网页模块启动成功，可通过WiFi访问控制界面");
    } else {
        ESP_LOGE(TAG, "电机控制网页模块启动失败");
    }

    ESP_LOGI(TAG, "系统运行中，等待用户操作...");
    
    // 主循环 - 系统已在各个任务中运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒输出一次状态
        ESP_LOGI(TAG, "系统运行正常，WiFi连接数：%d", wifi_softap_get_connected_count());
    }
}

/**
 * @brief 启用速度跟踪模式
 */
void enable_velocity_tracking_mode(void) {
    velocity_tracking_mode_active = true;
    velocity_tracking_mode_set_enabled(true);
    ESP_LOGI(TAG, "速度跟踪模式已启用");
}

/**
 * @brief 禁用速度跟踪模式
 */
void disable_velocity_tracking_mode(void) {
    velocity_tracking_mode_active = false;
    velocity_tracking_mode_set_enabled(false);
    ESP_LOGI(TAG, "速度跟踪模式已禁用");
}

/**
 * @brief 获取速度跟踪模式状态
 * @return true启用，false禁用
 */
bool is_velocity_tracking_mode_active(void) {
    return velocity_tracking_mode_active;
}

/**
 * @brief 重置速度跟踪模式到启用状态
 *
 * 将速度跟踪模式重置到ENABLED状态，等待波峰波谷差值>=0.75重新激活。
 * 这样可以让模式重新检测激活条件，而不是一直保持激活状态。
 */
void reset_velocity_tracking_mode(void) {
    velocity_tracking_reset_to_enabled();
    ESP_LOGI(TAG, "速度跟踪模式已重置，等待重新激活");
}



