/**
 * @file web_server.c
 * @brief HTTP Web服务器模块实现文件
 */

#include "web_server.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string.h>

static const char *TAG = "web_server";

httpd_handle_t web_server_handle = NULL;
MI_Motor* web_motors = NULL;

// HTML页面内容
static const char* html_page = 
"<!DOCTYPE html>"
"<html lang='zh-CN'>"
"<head>"
"    <meta charset='UTF-8'>"
"    <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"    <title>外骨骼控制系统</title>"
"    <style>"
"        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f0f0; }"
"        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
"        h1 { color: #333; text-align: center; margin-bottom: 30px; }"
"        .motor-section { margin-bottom: 30px; padding: 20px; border: 2px solid #ddd; border-radius: 8px; }"
"        .motor-title { font-size: 20px; font-weight: bold; color: #2c3e50; margin-bottom: 15px; }"
"        .motor-status { margin: 10px 0; padding: 10px; background: #f8f9fa; border-radius: 5px; }"
"        .button-group { margin: 15px 0; }"
"        .btn { padding: 10px 20px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; }"
"        .btn-enable { background: #28a745; color: white; }"
"        .btn-disable { background: #dc3545; color: white; }"
"        .btn-mode { background: #17a2b8; color: white; }"
"        .btn:hover { opacity: 0.8; }"
"        .status-online { color: #28a745; }"
"        .status-offline { color: #dc3545; }"
"        .refresh-btn { background: #007bff; color: white; float: right; }"
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🦾 外骨骼控制系统</h1>"
"        <button class='btn refresh-btn' onclick='location.reload()'>刷新状态</button>"
"        <div style='clear: both;'></div>"
"        "
"        <div class='motor-section'>"
"            <div class='motor-title'>电机 1</div>"
"            <div class='motor-status'>"
"                <div>状态: <span id='motor1-status' class='status-offline'>离线</span></div>"
"                <div>位置: <span id='motor1-pos'>--</span> rad</div>"
"                <div>速度: <span id='motor1-vel'>--</span> rad/s</div>"
"                <div>电流: <span id='motor1-cur'>--</span> A</div>"
"                <div>温度: <span id='motor1-temp'>--</span> °C</div>"
"            </div>"
"            <div class='button-group'>"
"                <button class='btn btn-enable' onclick='controlMotor(1, \"enable\")'>使能电机</button>"
"                <button class='btn btn-disable' onclick='controlMotor(1, \"disable\")'>禁用电机</button>"
"                <button class='btn btn-mode' onclick='controlMotor(1, \"current_mode\")'>电流模式</button>"
"            </div>"
"        </div>"
"        "
"        <div class='motor-section'>"
"            <div class='motor-title'>电机 2</div>"
"            <div class='motor-status'>"
"                <div>状态: <span id='motor2-status' class='status-offline'>离线</span></div>"
"                <div>位置: <span id='motor2-pos'>--</span> rad</div>"
"                <div>速度: <span id='motor2-vel'>--</span> rad/s</div>"
"                <div>电流: <span id='motor2-cur'>--</span> A</div>"
"                <div>温度: <span id='motor2-temp'>--</span> °C</div>"
"            </div>"
"            <div class='button-group'>"
"                <button class='btn btn-enable' onclick='controlMotor(2, \"enable\")'>使能电机</button>"
"                <button class='btn btn-disable' onclick='controlMotor(2, \"disable\")'>禁用电机</button>"
"                <button class='btn btn-mode' onclick='controlMotor(2, \"current_mode\")'>电流模式</button>"
"            </div>"
"        </div>"
"        "
"        <div style='text-align: center; margin-top: 30px; color: #666;'>"
"            <p>外骨骼WiFi控制系统 v1.0</p>"
"            <p>访问地址: <strong>http://192.168.4.1</strong></p>"
"        </div>"
"    </div>"
"    "
"    <script>"
"        function controlMotor(motorId, action) {"
"            fetch(`/motor/${motorId}/${action}`, { method: 'POST' })"
"            .then(response => response.json())"
"            .then(data => {"
"                alert(`电机${motorId} ${action} 操作${data.success ? '成功' : '失败'}: ${data.message}`);"
"                if(data.success) setTimeout(() => location.reload(), 1000);"
"            })"
"            .catch(error => alert('操作失败: ' + error));"
"        }"
"        "
"        function updateStatus() {"
"            fetch('/status')"
"            .then(response => response.json())"
"            .then(data => {"
"                for(let i = 1; i <= 2; i++) {"
"                    const motor = data.motors[i-1];"
"                    document.getElementById(`motor${i}-status`).textContent = motor.online ? '在线' : '离线';"
"                    document.getElementById(`motor${i}-status`).className = motor.online ? 'status-online' : 'status-offline';"
"                    document.getElementById(`motor${i}-pos`).textContent = motor.position.toFixed(3);"
"                    document.getElementById(`motor${i}-vel`).textContent = motor.velocity.toFixed(3);"
"                    document.getElementById(`motor${i}-cur`).textContent = motor.current.toFixed(3);"
"                    document.getElementById(`motor${i}-temp`).textContent = motor.temperature.toFixed(1);"
"                }"
"            })"
"            .catch(error => console.error('获取状态失败:', error));"
"        }"
"        "
"        // 页面加载时更新状态"
"        updateStatus();"
"        // 每5秒自动更新状态"
"        setInterval(updateStatus, 5000);"
"    </script>"
"</body>"
"</html>";

// HTTP处理函数

/* 主页处理 */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Favicon处理 - 避免404错误 */
static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* 状态接口处理 */
static esp_err_t status_handler(httpd_req_t *req)
{
    char json_response[512];
    
    if (web_motors == NULL) {
        snprintf(json_response, sizeof(json_response),
                "{\"motors\":["
                "{\"online\":false,\"position\":0,\"velocity\":0,\"current\":0,\"temperature\":0},"
                "{\"online\":false,\"position\":0,\"velocity\":0,\"current\":0,\"temperature\":0}"
                "]}");
    } else {
        snprintf(json_response, sizeof(json_response),
                "{\"motors\":["
                "{\"online\":%s,\"position\":%.3f,\"velocity\":%.3f,\"current\":%.3f,\"temperature\":%.1f},"
                "{\"online\":%s,\"position\":%.3f,\"velocity\":%.3f,\"current\":%.3f,\"temperature\":%.1f}"
                "]}",
                (web_motors[0].mode > 0) ? "true" : "false",
                web_motors[0].position, web_motors[0].velocity, 
                web_motors[0].current, web_motors[0].temperature,
                (web_motors[1].mode > 0) ? "true" : "false",
                web_motors[1].position, web_motors[1].velocity, 
                web_motors[1].current, web_motors[1].temperature);
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* 通用电机控制处理函数 */
static esp_err_t handle_motor_control(httpd_req_t *req, int motor_id, const char* action)
{
    char json_response[128];
    bool success = false;
    char message[64] = "未知错误";
    
    ESP_LOGI(TAG, "电机控制请求: 电机%d, 动作%s", motor_id, action);
    
    if (web_motors == NULL) {
        strcpy(message, "电机系统未初始化");
    } else if (motor_id < 1 || motor_id > 2) {
        strcpy(message, "无效的电机ID");
    } else {
        MI_Motor* motor = &web_motors[motor_id - 1];
        
        if (strcmp(action, "enable") == 0) {
            Motor_Enable(motor);
            success = true;
            strcpy(message, "电机使能成功");
        } else if (strcmp(action, "disable") == 0) {
            Motor_Reset(motor, 0);
            success = true;
            strcpy(message, "电机禁用成功");
        } else if (strcmp(action, "current_mode") == 0) {
            Change_Mode(motor, CUR_MODE);
            success = true;
            strcpy(message, "切换到电流控制模式");
        } else {
            strcpy(message, "未知的控制动作");
        }
    }
    
    snprintf(json_response, sizeof(json_response),
             "{\"success\":%s,\"message\":\"%s\",\"motor_id\":%d}",
             success ? "true" : "false", message, motor_id);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    
    ESP_LOGI(TAG, "电机控制响应: %s", json_response);
    return ESP_OK;
}

/* 电机1使能 */
static esp_err_t motor1_enable_handler(httpd_req_t *req) {
    return handle_motor_control(req, 1, "enable");
}

/* 电机1禁用 */
static esp_err_t motor1_disable_handler(httpd_req_t *req) {
    return handle_motor_control(req, 1, "disable");
}


/* 电机2使能 */
static esp_err_t motor2_enable_handler(httpd_req_t *req) {
    return handle_motor_control(req, 2, "enable");
}

/* 电机2禁用 */
static esp_err_t motor2_disable_handler(httpd_req_t *req) {
    return handle_motor_control(req, 2, "disable");
}


/* 电机1电流模式 */
static esp_err_t motor1_current_mode_handler(httpd_req_t *req) {
    return handle_motor_control(req, 1, "current_mode");
}

/* 电机2电流模式 */
static esp_err_t motor2_current_mode_handler(httpd_req_t *req) {
    return handle_motor_control(req, 2, "current_mode");
}

// URI处理器定义
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t favicon_uri = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t status_uri = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
};

// 电机控制URI处理器
static const httpd_uri_t motor1_enable_uri = {
    .uri       = "/motor/1/enable",
    .method    = HTTP_POST,
    .handler   = motor1_enable_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t motor1_disable_uri = {
    .uri       = "/motor/1/disable",
    .method    = HTTP_POST,
    .handler   = motor1_disable_handler,
    .user_ctx  = NULL
};


static const httpd_uri_t motor2_enable_uri = {
    .uri       = "/motor/2/enable",
    .method    = HTTP_POST,
    .handler   = motor2_enable_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t motor2_disable_uri = {
    .uri       = "/motor/2/disable",
    .method    = HTTP_POST,
    .handler   = motor2_disable_handler,
    .user_ctx  = NULL
};


static const httpd_uri_t motor1_current_mode_uri = {
    .uri       = "/motor/1/current_mode",
    .method    = HTTP_POST,
    .handler   = motor1_current_mode_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t motor2_current_mode_uri = {
    .uri       = "/motor/2/current_mode",
    .method    = HTTP_POST,
    .handler   = motor2_current_mode_handler,
    .user_ctx  = NULL
};

esp_err_t web_server_init(web_server_config_t* config)
{
    if (web_server_handle != NULL) {
        ESP_LOGW(TAG, "Web服务器已初始化");
        return ESP_OK;
    }
    
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    
    if (config != NULL) {
        server_config.server_port = config->port;
        server_config.max_open_sockets = config->max_open_sockets;
        server_config.lru_purge_enable = config->lru_purge_enable;
    } else {
        server_config.server_port = WEB_SERVER_PORT;
        server_config.max_open_sockets = WEB_SERVER_MAX_OPEN_SOCKETS;
    }
    
    ESP_LOGI(TAG, "初始化Web服务器，端口: %d", server_config.server_port);
    return ESP_OK;
}

esp_err_t web_server_start(void)
{
    if (web_server_handle != NULL) {
        ESP_LOGW(TAG, "Web服务器已在运行");
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_open_sockets = WEB_SERVER_MAX_OPEN_SOCKETS;
    
    ESP_LOGI(TAG, "启动Web服务器...");
    
    if (httpd_start(&web_server_handle, &config) == ESP_OK) {
        ESP_LOGI(TAG, "注册URI处理器");
        
        httpd_register_uri_handler(web_server_handle, &root_uri);
        httpd_register_uri_handler(web_server_handle, &favicon_uri);
        httpd_register_uri_handler(web_server_handle, &status_uri);
        
        // 注册电机控制URI处理器
        httpd_register_uri_handler(web_server_handle, &motor1_enable_uri);
        httpd_register_uri_handler(web_server_handle, &motor1_disable_uri);
        httpd_register_uri_handler(web_server_handle, &motor1_current_mode_uri);
        httpd_register_uri_handler(web_server_handle, &motor2_enable_uri);
        httpd_register_uri_handler(web_server_handle, &motor2_disable_uri);
        httpd_register_uri_handler(web_server_handle, &motor2_current_mode_uri);
        
        ESP_LOGI(TAG, "Web服务器启动成功，访问地址: http://192.168.4.1");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Web服务器启动失败");
        return ESP_FAIL;
    }
}

esp_err_t web_server_stop(void)
{
    if (web_server_handle == NULL) {
        ESP_LOGW(TAG, "Web服务器未运行");
        return ESP_OK;
    }
    
    esp_err_t ret = httpd_stop(web_server_handle);
    web_server_handle = NULL;
    
    ESP_LOGI(TAG, "Web服务器已停止");
    return ret;
}

web_server_config_t web_server_get_default_config(void)
{
    web_server_config_t default_config = {
        .port = WEB_SERVER_PORT,
        .max_open_sockets = WEB_SERVER_MAX_OPEN_SOCKETS,
        .lru_purge_enable = true
    };
    return default_config;
}

void web_server_set_motors(MI_Motor* motors)
{
    web_motors = motors;
    ESP_LOGI(TAG, "设置电机数组指针: %p", motors);
}