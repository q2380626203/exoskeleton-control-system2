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
#include "alternating_speed.h"

// 外部变量声明
extern MotorDataCallback data_callback;

static const char *TAG = "exoskeleton_main";

// 全局速度控制变量
float motor_target_speed[2] = {0.0f, 0.0f};  // 目标速度值数组，索引对应电机ID-1 (rad/s)
bool motor_speed_control_enabled = false;

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



