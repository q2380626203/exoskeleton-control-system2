#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

// UART配置
#define VOICE_UART_NUM       UART_NUM_2
#define VOICE_TX_PIN         4
#define VOICE_RX_PIN         3
#define VOICE_BAUDRATE       9600

typedef struct {
    uart_config_t uart_config;
    bool initialized;
} VoiceModule;

// 函数声明
void voice_module_init(VoiceModule* module);
void voice_speak(VoiceModule* module, const char* text);
void voice_speak_number(VoiceModule* module, int number);
void voice_speak_float(VoiceModule* module, float number);
void voice_speak_status(VoiceModule* module, const char* status);
void voice_speak_chinese_number(VoiceModule* module, int number);

// 内部辅助函数
void voice_send_gb2312(const char* text);
const char* number_to_chinese(int number);

#endif