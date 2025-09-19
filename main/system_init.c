/**
 * @file system_init.c
 * @brief 系统初始化模块实现文件
 */

#include "system_init.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_softap_module.h"
#include "rs01_motor.h"
#include "button_detector.h"
#include "voice_module.h"

static const char *TAG = "system_init";

// WiFi配置定义
#define DEFAULT_WIFI_SSID      "ESP32_Exoskeleton"
#define DEFAULT_WIFI_PASS      "12345678"
#define DEFAULT_WIFI_CHANNEL   1
#define DEFAULT_MAX_STA_CONN   4

// 全局变量

// 外部电机数组
extern MI_Motor motors[];

// 语音模块实例（全局可访问）
VoiceModule voiceModule;

// WiFi热点事件回调函数
static void wifi_event_callback(int32_t event_id, void* event_data)
{
    // 可以在这里添加WiFi事件的语音提示等
}

// 电机数据回调函数
static void motor_data_callback(MI_Motor* motor) {
    ESP_LOGI(TAG, "电机%d数据更新: 位置=%.3f, 速度=%.3f, 电流=%.3f, 温度=%.1f, 模式=%d", 
             motor->id, motor->position, motor->velocity, motor->current, motor->temperature, motor->mode);
}

esp_err_t system_init_nvs(void)
{
    ESP_LOGI(TAG, "初始化NVS存储...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "NVS存储初始化完成");
    return ESP_OK;
}

esp_err_t system_init_wifi(void)
{
    ESP_LOGI(TAG, "初始化WiFi热点...");
    
    wifi_softap_config_t config = {
        .ssid = DEFAULT_WIFI_SSID,
        .password = DEFAULT_WIFI_PASS,
        .channel = DEFAULT_WIFI_CHANNEL,
        .max_connection = DEFAULT_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
        .authmode = WIFI_AUTH_WPA3_PSK,
#else
        .authmode = WIFI_AUTH_WPA2_PSK,
#endif
    };
    
    ESP_ERROR_CHECK(wifi_softap_init(&config, wifi_event_callback));
    return ESP_OK;
}

esp_err_t system_init_exoskeleton(void)
{
    ESP_LOGI(TAG, "初始化外骨骼控制模块...");
    
    // 启用语音模块
    ESP_LOGI(TAG, "初始化语音模块...");
    voice_module_init(&voiceModule);
    //voice_speak(&voiceModule, "系统启动中");
    
    // 初始化电机通信
    UART_Rx_Init(motor_data_callback);
    
    // 等待UART稳定
    vTaskDelay(pdMS_TO_TICKS(500));

    // 语音播报系统启动成功
    ESP_LOGI(TAG, "语音播报: 启动成功");
    voice_speak(&voiceModule, "启动成功");
    vTaskDelay(pdMS_TO_TICKS(1500)); // 死等待1.5秒播放完成
    ESP_LOGI(TAG, "语音播报完成，开始设置电机");

    // 设置电机为运控模式
    ESP_LOGI(TAG, "设置电机为运控模式...");
    for(int i = 0; i < 2; i++) {
        MI_Motor* motor = &motors[i];

        // 设置上报时间参数
        ESP_LOGI(TAG, "设置电机 %d 上报时间参数...", motor->id);
        Set_SingleParameter(motor, REPORT_TIME, 1.0f);
        vTaskDelay(pdMS_TO_TICKS(50));


        Change_Mode(motor, CTRL_MODE);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 使能电机
        Motor_Enable(motor);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 开启电机上报
        ESP_LOGI(TAG, "开启电机 %d 上报功能...", motor->id);
        Motor_SetReporting(motor, true);
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_LOGI(TAG, "电机 %d 已设置为运控模式并使能", motor->id);
    }
    
    ESP_LOGI(TAG, "电机模式设置完成");
    
    // 初始化按键检测
    ESP_LOGI(TAG, "初始化按键检测模块...");
    button_init();

    // 创建按键处理任务
    ESP_LOGI(TAG, "启动按键处理任务...");
    xTaskCreate(buttonProcessTask, "ButtonProcess", 4096, NULL, 5, NULL);
    

    ESP_LOGI(TAG, "外骨骼控制模块初始化完成");
    return ESP_OK;
}

esp_err_t system_init_all(void)
{
    ESP_LOGI(TAG, "开始系统初始化...");
    
    // 等待5秒让系统稳定
    ESP_LOGI(TAG, "系统启动延时5秒...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "延时完成，开始初始化各模块");
    
    // 按顺序初始化各模块
    ESP_ERROR_CHECK(system_init_nvs());
    ESP_ERROR_CHECK(system_init_wifi());
    ESP_ERROR_CHECK(system_init_exoskeleton());
    
    // 系统初始化完成，但不在这里播报，等待main中速度跟踪启用后再播报
    // voice_speak(&voiceModule, "启动成功");
    ESP_LOGI(TAG, "系统初始化完成！");
    
    return ESP_OK;
}