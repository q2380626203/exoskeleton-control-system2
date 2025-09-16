/**
 * @file motor_web_control.h
 * @brief 电机控制网页模块头文件
 *
 * 提供基于HTTP服务器的电机运控参数调整和指令发送功能
 */

#ifndef MOTOR_WEB_CONTROL_H
#define MOTOR_WEB_CONTROL_H

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机控制网页模块初始化
 *
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t motor_web_control_init(void);

/**
 * @brief 启动HTTP服务器
 *
 * @return httpd_handle_t HTTP服务器句柄，失败返回NULL
 */
httpd_handle_t motor_web_control_start_server(void);

/**
 * @brief 停止HTTP服务器
 *
 * @param server HTTP服务器句柄
 * @return esp_err_t 返回ESP_OK表示成功
 */
esp_err_t motor_web_control_stop_server(httpd_handle_t server);

/**
 * @brief 网页首页处理函数
 */
esp_err_t motor_web_index_handler(httpd_req_t *req);

/**
 * @brief 电机控制API处理函数
 */
esp_err_t motor_web_control_api_handler(httpd_req_t *req);

/**
 * @brief 电机状态获取API处理函数
 */
esp_err_t motor_web_status_api_handler(httpd_req_t *req);

/**
 * @brief 设置电机运控参数API处理函数
 */
esp_err_t motor_web_params_api_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_WEB_CONTROL_H */