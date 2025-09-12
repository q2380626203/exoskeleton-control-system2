/**
 * @file wifi_softap_module.h
 * @brief WiFi SoftAP模块头文件
 * 
 * 提供WiFi热点功能的封装，包括初始化、配置和事件处理
 */

#ifndef WIFI_SOFTAP_MODULE_H
#define WIFI_SOFTAP_MODULE_H

#include "esp_wifi.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi热点配置结构体
 */
typedef struct {
    char ssid[32];          /**< WiFi热点名称 */
    char password[64];      /**< WiFi密码 */
    uint8_t channel;        /**< WiFi信道 */
    uint8_t max_connection; /**< 最大连接数 */
    wifi_auth_mode_t authmode; /**< 认证模式 */
} wifi_softap_config_t;

/**
 * @brief WiFi热点事件回调函数类型
 * 
 * @param event_id 事件ID
 * @param event_data 事件数据
 */
typedef void (*wifi_softap_event_cb_t)(int32_t event_id, void* event_data);

/**
 * @brief 初始化WiFi热点模块
 * 
 * @param config WiFi热点配置
 * @param event_callback 事件回调函数（可选）
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t wifi_softap_init(const wifi_softap_config_t* config, wifi_softap_event_cb_t event_callback);


/**
 * @brief 获取当前连接的设备数量
 * 
 * @return uint8_t 连接的设备数量
 */
uint8_t wifi_softap_get_connected_count(void);

/**
 * @brief 获取默认WiFi热点配置
 * 
 * @return wifi_softap_config_t 默认配置
 */
wifi_softap_config_t wifi_softap_get_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_SOFTAP_MODULE_H */