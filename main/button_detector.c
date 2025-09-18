#include "button_detector.h"
#include "esp_log.h"
#include "system_init.h"
#include "velocity_tracking_mode.h"

static const char *TAG = "BUTTON_DETECTOR";

// 全局变量定义
ButtonData g_buttonData = {0, 0, false, false};


// GPIO1中断服务函数
void IRAM_ATTR button_gpio1_isr(void* args) {
    uint64_t currentTime = esp_timer_get_time() / 1000; // Convert to ms
    
    // 去抖动处理 - 1秒内只判定一次按下
    if (currentTime - g_buttonData.gpio1_lastInterruptTime > BUTTON_DEBOUNCE_MS) {
        g_buttonData.gpio1_pressed_flag = true;
        g_buttonData.gpio1_lastInterruptTime = currentTime;
    }
}

// GPIO2中断服务函数
void IRAM_ATTR button_gpio2_isr(void* args) {
    uint64_t currentTime = esp_timer_get_time() / 1000; // Convert to ms
    
    // 去抖动处理 - 1秒内只判定一次按下
    if (currentTime - g_buttonData.gpio2_lastInterruptTime > BUTTON_DEBOUNCE_MS) {
        g_buttonData.gpio2_pressed_flag = true;
        g_buttonData.gpio2_lastInterruptTime = currentTime;
    }
}

void button_init() {
    // 配置GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // 下降沿触发
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO1_PIN) | (1ULL << BUTTON_GPIO2_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;  // 启用内部上拉
    gpio_config(&io_conf);
    
    // 初始化按键数据结构
    g_buttonData.gpio1_lastInterruptTime = 0;
    g_buttonData.gpio2_lastInterruptTime = 0;
    g_buttonData.gpio1_pressed_flag = false;
    g_buttonData.gpio2_pressed_flag = false;
    
    // 安装GPIO中断服务
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO1_PIN, button_gpio1_isr, NULL);
    gpio_isr_handler_add(BUTTON_GPIO2_PIN, button_gpio2_isr, NULL);
    
    ESP_LOGI(TAG, "按键检测模块初始化完成（仅按下检测，1秒去抖）");
    ESP_LOGI(TAG, "GPIO1引脚: %d, GPIO2引脚: %d", BUTTON_GPIO1_PIN, BUTTON_GPIO2_PIN);
}

// 按键处理任务
void buttonProcessTask(void* pvParameters) {
    while (true) {
        // 检查按键1状态位（增加velocity_tracking_lift_leg_torque）
        if (g_buttonData.gpio1_pressed_flag) {
            g_buttonData.gpio1_pressed_flag = false;  // 清除状态位

            // 增加抬腿力矩，最大限制在10Nm
            if (velocity_tracking_lift_leg_torque < 10.0f) {
                velocity_tracking_lift_leg_torque += 1.0f;
                if (velocity_tracking_lift_leg_torque > 10.0f) {
                    velocity_tracking_lift_leg_torque = 10.0f;
                }
            } else {
                // 即使已经在最大值，也保持在10Nm
                ESP_LOGW(TAG, "按键1被按下 - 抬腿力矩已在最大值10Nm");
            }

            ESP_LOGI(TAG, "按键1被按下 - 增加抬腿力矩至: %.1f Nm", velocity_tracking_lift_leg_torque);

            // 统一语音播报新值
            int torque_int = (int)velocity_tracking_lift_leg_torque;
            voice_speak(&voiceModule, "增加");
            vTaskDelay(pdMS_TO_TICKS(1500)); 

            // 组合"当前"和数字一起播报
            char current_msg[32];
            const char* number_str = number_to_chinese(torque_int);
            snprintf(current_msg, sizeof(current_msg), "当前%s", number_str);
            voice_speak(&voiceModule, current_msg);
        }
        
        // 检查按键2状态位（减少velocity_tracking_lift_leg_torque）
        if (g_buttonData.gpio2_pressed_flag) {
            g_buttonData.gpio2_pressed_flag = false;  // 清除状态位

            // 减少抬腿力矩，最小限制在1Nm
            if (velocity_tracking_lift_leg_torque > 1.0f) {
                velocity_tracking_lift_leg_torque -= 1.0f;
                if (velocity_tracking_lift_leg_torque < 1.0f) {
                    velocity_tracking_lift_leg_torque = 1.0f;
                }
            } else {
                // 即使已经在最小值，也保持在1Nm
                ESP_LOGW(TAG, "按键2被按下 - 抬腿力矩已在最小值1Nm");
            }

            ESP_LOGI(TAG, "按键2被按下 - 减少抬腿力矩至: %.1f Nm", velocity_tracking_lift_leg_torque);

            // 统一语音播报新值
            int torque_int = (int)velocity_tracking_lift_leg_torque;
            voice_speak(&voiceModule, "减少");
            vTaskDelay(pdMS_TO_TICKS(1500)); 

            // 组合"当前"和数字一起播报
            char current_msg[32];
            const char* number_str = number_to_chinese(torque_int);
            snprintf(current_msg, sizeof(current_msg), "当前%s", number_str);
            voice_speak(&voiceModule, current_msg);
        }
        
        // 任务延时，避免过度占用CPU
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时
    }
}