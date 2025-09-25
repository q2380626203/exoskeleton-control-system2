#ifndef BUTTON_DETECTOR_H
#define BUTTON_DETECTOR_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// GPIO引脚配置
#define BUTTON_GPIO1_PIN    GPIO_NUM_13
#define BUTTON_GPIO2_PIN    GPIO_NUM_12

// 按键状态定义
#define BUTTON_RELEASED     1
#define BUTTON_PRESSED      0

// 按键去抖动配置
#define BUTTON_DEBOUNCE_MS  1000

// 按键数据结构 - 去抖动和状态位
typedef struct {
    volatile uint64_t gpio1_lastInterruptTime;
    volatile uint64_t gpio2_lastInterruptTime;
    volatile bool gpio1_pressed_flag;
    volatile bool gpio2_pressed_flag;
} ButtonData;

// 全局变量声明
extern ButtonData g_buttonData;

// 中断服务函数声明
void IRAM_ATTR button_gpio1_isr(void* args);
void IRAM_ATTR button_gpio2_isr(void* args);

// 函数声明
void button_init();
void buttonProcessTask(void* pvParameters);

#endif // BUTTON_DETECTOR_H