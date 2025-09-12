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

/**
 * @brief 电机电流控制任务
 */
void Motor_Current_Control_Task(void* pvParameters) {
    ESP_LOGI(TAG, "电机电流控制任务启动");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(100); // 50ms周期
    
    while (motor_current_control_enabled) {
        // 为每个电机设置目标电流
        for (int i = 0; i < 2; i++) {
            float target_current = motor_target_current[i];
            
            // 限制电流范围到安全值 (-17A到17A)
            if (target_current > 17.0f) {
                target_current = 17.0f;
            } else if (target_current < -17.0f) {
                target_current = -17.0f;
            }
            
            // 这里暂时注释掉，由Web界面直接调用Set_CurMode
            MI_Motor* motor = &motors[i];
            Set_CurMode(motor, target_current);
        }
        
        // 等待下一个周期
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