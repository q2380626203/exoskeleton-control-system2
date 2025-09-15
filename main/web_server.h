/**
 * @file web_server.h  
 * @brief 简化的Web服务器头文件
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "esp_err.h"
#include "rs01_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动Web服务器
 * @return esp_err_t 
 */
esp_err_t web_server_start(void);

/**
 * @brief 停止Web服务器
 * @return esp_err_t 
 */
esp_err_t web_server_stop(void);

/**
 * @brief 设置电机数组指针
 * @param motors 电机数组
 */
void web_server_set_motors(MI_Motor* motors);

/**
 * @brief 获取电机状态JSON字符串
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return 返回JSON字符串长度，失败返回-1
 */
int get_motor_status_json(char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */