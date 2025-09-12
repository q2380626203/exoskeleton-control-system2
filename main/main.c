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

static const char *TAG = "exoskeleton_main";

// 全局电流控制变量
float motor_target_current[2] = {0.0f, 0.0f};  // 目标电流值数组，索引对应电机ID-1
bool motor_current_control_enabled = false;
static TaskHandle_t current_control_task_handle = NULL;

// 交替电流控制变量
bool alternating_current_enabled = false;
static int current_state = 0;  // 0: 正向，1: 反向
static TickType_t last_switch_time = 0;

// 交替电流配置变量
int alternating_interval_ms = 750;  // 默认750ms间隔
float alternating_current_x = 6.0f;  // 交替电流值X
float alternating_current_y = 3.0f;  // 交替电流值Y

/**
 * @brief 电机电流控制任务
 */
void Motor_Current_Control_Task(void* pvParameters) {
    ESP_LOGI(TAG, "电机电流控制任务启动");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(100); // 100ms总周期
    last_switch_time = last_wake_time;
    
    while (motor_current_control_enabled) {
        TickType_t current_time = xTaskGetTickCount();
        
        // 检查是否需要切换电流方向（使用配置的间隔时间）
        if (alternating_current_enabled && 
            (current_time - last_switch_time) >= pdMS_TO_TICKS(alternating_interval_ms)) {
            current_state = 1 - current_state; // 0变1，1变0
            last_switch_time = current_time;
            ESP_LOGI(TAG, "电流切换到状态: %d", current_state);
        }
        
        // 为每个电机计算目标电流
        float target_currents[2];
        for (int i = 0; i < 2; i++) {
            if (alternating_current_enabled) {
                // 交替电流模式：两电机交替使用对方的电流值，1号负向，2号正向
                if (current_state == 0) {
                    // 周期1：1号-X A，2号-Y A
                    target_currents[i] = (i == 0) ? -alternating_current_x : -alternating_current_y;
                } else {
                    // 周期2：1号+Y A，2号+X A
                    target_currents[i] = (i == 0) ? alternating_current_y : alternating_current_x;
                }
            } else {
                // 普通模式：使用设定的目标电流
                target_currents[i] = motor_target_current[i];
            }
            
            // 限制电流范围到安全值 (-17A到17A)
            if (target_currents[i] > 17.0f) {
                target_currents[i] = 17.0f;
            } else if (target_currents[i] < -17.0f) {
                target_currents[i] = -17.0f;
            }
        }
        
        // 先发送1号电机控制命令
        MI_Motor* motor1 = &motors[0];
        Set_CurMode(motor1, target_currents[0]);
        
        // 等待10ms
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 再发送2号电机控制命令
        MI_Motor* motor2 = &motors[1];
        Set_CurMode(motor2, target_currents[1]);
        
        // 等待剩余的时间完成100ms周期 (100ms - 10ms = 90ms)
        vTaskDelayUntil(&last_wake_time, task_period);
    }
    
    ESP_LOGI(TAG, "电机电流控制任务退出");
    current_control_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 启动电机电流控制任务
 */
void Start_Motor_Current_Control(void) {
    if (current_control_task_handle == NULL) {
        motor_current_control_enabled = true;
        
        BaseType_t result = xTaskCreate(
            Motor_Current_Control_Task,
            "MotorCurrentCtrl",
            4096,
            NULL,
            6,
            &current_control_task_handle
        );
        
        if (result == pdPASS) {
            ESP_LOGI(TAG, "电机电流控制任务创建成功");
        } else {
            ESP_LOGE(TAG, "电机电流控制任务创建失败");
            motor_current_control_enabled = false;
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
    
    // 启动电机电流控制任务
    Start_Motor_Current_Control();
    
    ESP_LOGI(TAG, "系统运行中，等待用户操作...");
    
    // 主循环 - 系统已在各个任务中运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒输出一次状态
        ESP_LOGI(TAG, "系统运行正常，WiFi连接数：%d", wifi_softap_get_connected_count());
    }
}

/**
 * @brief 启动交替电流模式
 */
void Start_Alternating_Current(void) {
    alternating_current_enabled = true;
    current_state = 0;
    last_switch_time = xTaskGetTickCount();
    ESP_LOGI(TAG, "交替电流模式已启动 - 两电机同向7A/-7A，每1秒切换");
}

/**
 * @brief 停止交替电流模式
 */
void Stop_Alternating_Current(void) {
    alternating_current_enabled = false;
    ESP_LOGI(TAG, "交替电流模式已停止");
}