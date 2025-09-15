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
#include "system_init.h"
#include "wifi_softap_module.h"
#include "rs01_motor.h"

// 外部变量声明
extern MotorDataCallback data_callback;

static const char *TAG = "exoskeleton_main";

// 全局速度控制变量
float motor_target_speed[2] = {0.0f, 0.0f};  // 目标速度值数组，索引对应电机ID-1 (rad/s)
bool motor_speed_control_enabled = false;
static TaskHandle_t speed_control_task_handle = NULL;

// 串口数据解析任务相关变量
static TaskHandle_t uart_parse_task_handle = NULL;
bool uart_parse_enabled = true;

// 交替速度控制变量
bool alternating_speed_enabled = false;
static int speed_state = 0;  // 0: 正向，1: 反向
static TickType_t last_switch_time = 0;

// 交替速度配置变量（默认平地模式）
int alternating_interval_ms = 800;  // 默认800ms间隔（平地模式）
float alternating_speed_x = 1.0f;   // 交替速度值X (rad/s)
float alternating_speed_y = 1.0f;   // 交替速度值Y (rad/s)  
float speed_current_limit = 5.0f;   // 速度模式下的电流限制 (A，平地模式)

// 模式参数常量定义
typedef enum {
    MODE_FLAT_GROUND = 0,  // 平地模式
    MODE_STAIRS = 1        // 爬楼模式
} walking_mode_t;

// 平地模式参数
#define FLAT_INTERVAL_MS 800
#define FLAT_SPEED_X 1.0f
#define FLAT_SPEED_Y 1.0f
#define FLAT_CURRENT_LIMIT 5.0f

// 爬楼模式参数
#define STAIRS_INTERVAL_MS 1000
#define STAIRS_SPEED_X 2.5f
#define STAIRS_SPEED_Y 2.5f
#define STAIRS_CURRENT_LIMIT 10.0f

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

/**
 * @brief 电机速度控制任务
 */
void Motor_Speed_Control_Task(void* pvParameters) {
    ESP_LOGI(TAG, "电机速度控制任务启动");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(100); // 100ms总周期
    last_switch_time = last_wake_time;
    
    while (motor_speed_control_enabled) {
        TickType_t current_time = xTaskGetTickCount();
        
        // 检查是否需要切换速度方向（使用配置的间隔时间）
        if (alternating_speed_enabled && 
            (current_time - last_switch_time) >= pdMS_TO_TICKS(alternating_interval_ms)) {
            speed_state = 1 - speed_state; // 0变1，1变0
            last_switch_time = current_time;
            ESP_LOGI(TAG, "速度切换到状态: %d", speed_state);
        }
        
        // 为每个电机计算目标速度
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
            if (target_speeds[i] > 44.0f) {
                target_speeds[i] = 44.0f;
            } else if (target_speeds[i] < -44.0f) {
                target_speeds[i] = -44.0f;
            }
        }
        
        // 先发送1号电机控制命令
        MI_Motor* motor1 = &motors[0];
        Set_SpeedMode(motor1, target_speeds[0], speed_current_limit);
        
        // 等待10ms
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 再发送2号电机控制命令
        MI_Motor* motor2 = &motors[1];
        Set_SpeedMode(motor2, target_speeds[1], speed_current_limit);
        
        // 等待剩余的时间完成100ms周期 (100ms - 10ms = 90ms)
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
            6,
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
    ESP_LOGI(TAG, "交替速度模式已启动 - 两电机速度交替切换，每%.1f秒切换", alternating_interval_ms / 1000.0f);
}

/**
 * @brief 停止交替速度模式
 */
void Stop_Alternating_Speed(void) {
    alternating_speed_enabled = false;
    ESP_LOGI(TAG, "交替速度模式已停止");
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
            ESP_LOGI(TAG, "切换到平地模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 电流限制=%.1fA", 
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        } else if (mode == MODE_STAIRS) {
            alternating_interval_ms = STAIRS_INTERVAL_MS;
            alternating_speed_x = STAIRS_SPEED_X;
            alternating_speed_y = STAIRS_SPEED_Y;
            speed_current_limit = STAIRS_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到爬楼模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 电流限制=%.1fA", 
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

