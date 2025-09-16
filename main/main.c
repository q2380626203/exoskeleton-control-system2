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
#include "uart_motor_transmit.h"
#include "motor_web_control.h"

// 外部变量声明
extern MotorDataCallback data_callback;

static const char *TAG = "exoskeleton_main";

// 全局速度控制变量
float motor_target_speed[2] = {0.0f, 0.0f};  // 目标速度值数组，索引对应电机ID-1 (rad/s)
bool motor_speed_control_enabled = false;
static TaskHandle_t speed_control_task_handle = NULL;

// 全局运控模式参数（数组索引对应电机ID-1）
float control_position[2] = {0.0f, 0.0f};     // 位置设定 (rad)
float control_speed[2] = {0.0f, 0.0f};        // 速度设定 (rad/s)
float control_torque[2] = {0.0f, 0.0f};       // 力矩设定 (Nm)
float control_kp[2] = {0.0f, 0.0f};           // Kp参数
float control_kd[2] = {0.0f, 0.0f};           // Kd参数

// 串口数据解析任务相关变量
static TaskHandle_t uart_parse_task_handle = NULL;
bool uart_parse_enabled = true;

// 电机数据传输任务相关变量
static TaskHandle_t motor_transmit_task_handle = NULL;

// 交替速度控制变量
bool alternating_speed_enabled = false;
static int speed_state = 0;  // 0: 正向，1: 反向
static TickType_t last_switch_time = 0;

// 交替速度配置变量（默认平地模式）
int alternating_interval_ms = 800;  // 默认800ms间隔（平地模式）
float alternating_speed_x = 2.25f;  // 交替速度值X (rad/s) - 抬腿速度提升至1.5倍
float alternating_speed_y = 1.5f;   // 交替速度值Y (rad/s)
float speed_current_limit = 0.0f;   // 速度模式下的电流限制 (平地模式:5A, 爬楼模式:10A)

// 模式参数常量定义
typedef enum {
    MODE_FLAT_GROUND = 0,  // 平地模式
    MODE_STAIRS = 1        // 爬楼模式
} walking_mode_t;

// 平地模式参数
#define FLAT_INTERVAL_MS 800
#define FLAT_SPEED_X 2.25f  
#define FLAT_SPEED_Y 1.5f
#define FLAT_CURRENT_LIMIT 2.0f

// 爬楼模式参数
#define STAIRS_INTERVAL_MS 1000
#define STAIRS_SPEED_X 3.0f  
#define STAIRS_SPEED_Y 2.25f
#define STAIRS_CURRENT_LIMIT 2.0f

// 当前模式
static walking_mode_t current_walking_mode = MODE_FLAT_GROUND;




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

// 上次发送的参数缓存，用于检测参数变化
static float last_sent_torque[2] = {0.0f, 0.0f};
static float last_sent_position[2] = {0.0f, 0.0f};
static float last_sent_speed[2] = {0.0f, 0.0f};
static float last_sent_kp[2] = {0.0f, 0.0f};
static float last_sent_kd[2] = {0.0f, 0.0f};

// 参数变化检测阈值
#define PARAM_CHANGE_THRESHOLD 0.001f

/**
 * @brief 检查电机参数是否发生变化
 * @param motor_id 电机ID (0或1)
 * @param torque 力矩参数
 * @param position 位置参数
 * @param speed 速度参数
 * @param kp Kp参数
 * @param kd Kd参数
 * @return true 参数发生变化，false 参数未变化
 */
bool Motor_Params_Changed(int motor_id, float torque, float position, float speed, float kp, float kd) {
    if (motor_id < 0 || motor_id >= 2) return false;

    return (fabsf(torque - last_sent_torque[motor_id]) > PARAM_CHANGE_THRESHOLD) ||
           (fabsf(position - last_sent_position[motor_id]) > PARAM_CHANGE_THRESHOLD) ||
           (fabsf(speed - last_sent_speed[motor_id]) > PARAM_CHANGE_THRESHOLD) ||
           (fabsf(kp - last_sent_kp[motor_id]) > PARAM_CHANGE_THRESHOLD) ||
           (fabsf(kd - last_sent_kd[motor_id]) > PARAM_CHANGE_THRESHOLD);
}

/**
 * @brief 更新已发送参数缓存
 * @param motor_id 电机ID (0或1)
 * @param torque 力矩参数
 * @param position 位置参数
 * @param speed 速度参数
 * @param kp Kp参数
 * @param kd Kd参数
 */
void Update_Sent_Params(int motor_id, float torque, float position, float speed, float kp, float kd) {
    if (motor_id < 0 || motor_id >= 2) return;

    last_sent_torque[motor_id] = torque;
    last_sent_position[motor_id] = position;
    last_sent_speed[motor_id] = speed;
    last_sent_kp[motor_id] = kp;
    last_sent_kd[motor_id] = kd;
}

/**
 * @brief 电机速度控制任务（条件发送版本）
 */
void Motor_Speed_Control_Task(void* pvParameters) {
    ESP_LOGI(TAG, "电机速度控制任务启动（条件发送模式）");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(20); //
    last_switch_time = last_wake_time;

    // 初始化参数变化标志
    bool params_changed[2] = {true, true}; // 首次启动时强制发送
    bool alternating_state_changed = false;

    while (motor_speed_control_enabled) {
        TickType_t current_time = xTaskGetTickCount();

        // 检查是否需要切换速度方向（使用配置的间隔时间）
        if (alternating_speed_enabled &&
            (current_time - last_switch_time) >= pdMS_TO_TICKS(alternating_interval_ms)) {
            speed_state = 1 - speed_state; // 0变1，1变0
            last_switch_time = current_time;
            alternating_state_changed = true;
            ESP_LOGI(TAG, "速度切换到状态: %d", speed_state);
        }

        // 计算目标速度
        float target_speeds[2];

        for (int i = 0; i < 2; i++) {
            if (alternating_speed_enabled) {
                // 交替速度模式：两电机交替使用对方的速度值，1号负向，2号正向
                if (speed_state == 0) {
                    // 周期1：1号-X rad/s，2号-Y rad/s
                    target_speeds[i] = (i == 0) ? -alternating_speed_x : -alternating_speed_y;
                } else {
                    // 周期2：1号+Y rad/s，2号+X rad/s
                    target_speeds[i] = (i == 0) ? alternating_speed_y : alternating_speed_x;
                }
            } else {
                // 普通模式：使用设定的目标速度
                target_speeds[i] = motor_target_speed[i];
            }

            // 限制速度范围到安全值 (-44 ~ 44 rad/s)
            if (target_speeds[i] > 44.0f) target_speeds[i] = 44.0f;
            else if (target_speeds[i] < -44.0f) target_speeds[i] = -44.0f;
        }

        // 检查每个电机的参数是否发生变化
        for (int i = 0; i < 2; i++) {
            params_changed[i] = Motor_Params_Changed(i, control_torque[i], control_position[i],
                                                   target_speeds[i], control_kp[i], control_kd[i]) ||
                              alternating_state_changed;
        }

        // 临界区保护，避免WiFi中断影响电机控制时序
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

        // 只有在参数发生变化时才发送控制指令
        for (int i = 0; i < 2; i++) {
            if (params_changed[i]) {
                portENTER_CRITICAL(&mux);

                // 发送电机控制指令
                Motor_ControlMode(&motors[i], control_torque[i], control_position[i], target_speeds[i], control_kp[i], control_kd[i]);

                portEXIT_CRITICAL(&mux);

                // 更新已发送参数缓存
                Update_Sent_Params(i, control_torque[i], control_position[i], target_speeds[i], control_kp[i], control_kd[i]);

                ESP_LOGI(TAG, "电机%d参数更新: 力矩=%.2f, 位置=%.2f, 速度=%.2f, Kp=%.2f, Kd=%.2f",
                         i+1, control_torque[i], control_position[i], target_speeds[i], control_kp[i], control_kd[i]);

                // 发送间隔
                if (i == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        // 重置状态变化标志
        alternating_state_changed = false;

        // 等待剩余的时间完成20ms周期
        vTaskDelayUntil(&last_wake_time, task_period);
    }

    ESP_LOGI(TAG, "电机速度控制任务退出");
    speed_control_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 启动电机速度控制任务
 */
void Start_Motor_Speed_Control(void) {
    if (speed_control_task_handle == NULL) {
        motor_speed_control_enabled = true;

        BaseType_t result = xTaskCreate(
            Motor_Speed_Control_Task,
            "MotorSpeedCtrl",
            4096,
            NULL,
            7,  // 提高电机控制优先级，确保实时性
            &speed_control_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "电机速度控制任务创建成功");
        } else {
            ESP_LOGE(TAG, "电机速度控制任务创建失败");
            motor_speed_control_enabled = false;
        }
    }
}

/**
 * @brief 启动电机数据传输任务
 */
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

/**
 * @brief 应用程序主入口函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "外骨骼WiFi控制系统启动中...");
    
    // 初始化整个系统
    ESP_ERROR_CHECK(system_init_all());
    
    // 更新电机数据回调函数为我们自定义的回调
    data_callback = motor_data_update_callback;
    ESP_LOGI(TAG, "已更新电机数据回调函数");

    // 启动串口数据解析任务 (UART已在system_init_all()中初始化)
    ESP_LOGI(TAG, "启动RS01电机数据解析任务...");
    Start_UART_Parse_Task();

    // 启动电机速度控制任务
    Start_Motor_Speed_Control();

    // 初始化并启动电机数据传输功能
    ESP_LOGI(TAG, "初始化电机数据传输...");
    uart_motor_transmit_init();
    Start_Motor_Data_Transmit();

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
 * @brief 启动交替速度模式
 */
void Start_Alternating_Speed(void) {
    alternating_speed_enabled = true;
    speed_state = 0;
    last_switch_time = xTaskGetTickCount();

    // 启动交替行走时设置Kd=2.0
    control_kd[0] = 2.0f;
    control_kd[1] = 2.0f;

    // 立即发送Motor_ControlMode指令，确保带有速度和Kd=2
    // 计算初始交替速度：周期1状态（1号-X rad/s，2号-Y rad/s）
    float initial_speed_1 = -alternating_speed_x;  // 1号电机负向速度
    float initial_speed_2 = -alternating_speed_y;  // 2号电机负向速度

    // 限制速度范围到安全值 (-44 ~ 44 rad/s)
    if (initial_speed_1 > 44.0f) initial_speed_1 = 44.0f;
    else if (initial_speed_1 < -44.0f) initial_speed_1 = -44.0f;

    if (initial_speed_2 > 44.0f) initial_speed_2 = 44.0f;
    else if (initial_speed_2 < -44.0f) initial_speed_2 = -44.0f;

    // 临界区保护，立即发送电机控制指令
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    portENTER_CRITICAL(&mux);
    Motor_ControlMode(&motors[0], control_torque[0], control_position[0], initial_speed_1, control_kp[0], control_kd[0]);
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms间隔

    portENTER_CRITICAL(&mux);
    Motor_ControlMode(&motors[1], control_torque[1], control_position[1], initial_speed_2, control_kp[1], control_kd[1]);
    portEXIT_CRITICAL(&mux);

    // 更新已发送参数缓存
    Update_Sent_Params(0, control_torque[0], control_position[0], initial_speed_1, control_kp[0], control_kd[0]);
    Update_Sent_Params(1, control_torque[1], control_position[1], initial_speed_2, control_kp[1], control_kd[1]);

    ESP_LOGI(TAG, "交替速度模式已启动 - 立即发送控制指令: 电机1速度=%.2f(Kd=%.1f), 电机2速度=%.2f(Kd=%.1f), 每%.1f秒切换",
             initial_speed_1, control_kd[0], initial_speed_2, control_kd[1], alternating_interval_ms / 1000.0f);
}

/**
 * @brief 停止交替速度模式
 */
void Stop_Alternating_Speed(void) {
    alternating_speed_enabled = false;

    // 停止交替行走时设置Kd=0.0
    //control_kd[0] = 0.0f;
    //control_kd[1] = 0.0f;

    ESP_LOGI(TAG, "交替速度模式已停止，Kd设置为0.0");
}

/**
 * @brief 切换行走模式
 * @param mode 目标模式：0=平地模式，1=爬楼模式
 */
void Switch_Walking_Mode(walking_mode_t mode) {
    if (mode != current_walking_mode) {
        current_walking_mode = mode;
        
        // 根据模式更新参数
        if (mode == MODE_FLAT_GROUND) {
            alternating_interval_ms = FLAT_INTERVAL_MS;
            alternating_speed_x = FLAT_SPEED_X;
            alternating_speed_y = FLAT_SPEED_Y;
            speed_current_limit = FLAT_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到平地模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        } else if (mode == MODE_STAIRS) {
            alternating_interval_ms = STAIRS_INTERVAL_MS;
            alternating_speed_x = STAIRS_SPEED_X;
            alternating_speed_y = STAIRS_SPEED_Y;
            speed_current_limit = STAIRS_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到爬楼模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        }

        // 如果交替速度模式正在运行，重置切换时间
        if (alternating_speed_enabled) {
            speed_state = 0;
            last_switch_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "重置交替速度状态");
        }
    } else {
        ESP_LOGW(TAG, "已经处于所选模式，无需切换");
    }
}

/**
 * @brief 获取当前行走模式
 * @return 当前模式：0=平地模式，1=爬楼模式
 */
walking_mode_t Get_Current_Walking_Mode(void) {
    return current_walking_mode;
}

/**
 * @brief 切换到平地模式
 */
void Switch_To_Flat_Mode(void) {
    Switch_Walking_Mode(MODE_FLAT_GROUND);
}

/**
 * @brief 切换到爬楼模式
 */
void Switch_To_Stairs_Mode(void) {
    Switch_Walking_Mode(MODE_STAIRS);
}


