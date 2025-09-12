/**
 * @file system_init.h
 * @brief 系统初始化模块头文件
 * 
 * 负责整个外骨骼系统的初始化，包括WiFi、电机、语音、按键等模块
 */

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化NVS存储
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t system_init_nvs(void);

/**
 * @brief 初始化WiFi热点
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t system_init_wifi(void);

/**
 * @brief 初始化外骨骼控制模块
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t system_init_exoskeleton(void);

/**
 * @brief 初始化整个系统
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t system_init_all(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INIT_H */