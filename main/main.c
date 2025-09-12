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

static const char *TAG = "exoskeleton_main";

/**
 * @brief 应用程序主入口函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "外骨骼WiFi控制系统启动中...");
    
    // 初始化整个系统
    ESP_ERROR_CHECK(system_init_all());
    
    ESP_LOGI(TAG, "系统运行中，等待用户操作...");
    
    // 主循环 - 系统已在各个任务中运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒输出一次状态
        ESP_LOGI(TAG, "系统运行正常，WiFi连接数：%d", wifi_softap_get_connected_count());
    }
}