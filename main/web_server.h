/**
 * @file web_server.h
 * @brief HTTP Web服务器模块头文件
 * 
 * 提供基于ESP32的Web服务器功能，用于控制外骨骼电机系统
 * 默认地址：192.168.4.1
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "esp_err.h"
#include "rs01_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WEB_SERVER_PORT 80
#define WEB_SERVER_MAX_OPEN_SOCKETS 7

/**
 * @brief Web服务器配置结构体
 */
typedef struct {
    uint16_t port;              /**< 服务器端口，默认80 */
    uint8_t max_open_sockets;   /**< 最大并发连接数 */
    uint32_t lru_purge_enable;  /**< 启用LRU清理 */
} web_server_config_t;

/**
 * @brief Web服务器句柄
 */
extern httpd_handle_t web_server_handle;

/**
 * @brief 外部电机数组引用
 */
extern MI_Motor* web_motors;

/**
 * @brief 初始化Web服务器
 * 
 * @param config Web服务器配置，传入NULL使用默认配置
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t web_server_init(web_server_config_t* config);

/**
 * @brief 启动Web服务器
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t web_server_start(void);

/**
 * @brief 停止Web服务器
 * 
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t web_server_stop(void);

/**
 * @brief 获取默认Web服务器配置
 * 
 * @return web_server_config_t 默认配置
 */
web_server_config_t web_server_get_default_config(void);

/**
 * @brief 设置电机数组指针
 * 
 * @param motors 电机数组指针
 */
void web_server_set_motors(MI_Motor* motors);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */