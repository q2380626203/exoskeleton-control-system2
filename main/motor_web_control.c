/**
 * @file motor_web_control.c
 * @brief 电机控制网页模块实现文件
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/param.h>
#include "motor_web_control.h"
#include "motor_web_html.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "alternating_speed.h"

// 引入main.c中的全局变量和函数
extern float motor_target_speed[2];
extern bool motor_speed_control_enabled;

// 统一的电机控制参数结构体
typedef struct {
    float torque;      // 力矩设定 (Nm)
    float position;    // 位置设定 (rad)
    float speed;       // 速度设定 (rad/s)
    float kp;          // Kp参数
    float kd;          // Kd参数
} motor_control_params_t;

extern motor_control_params_t motor_params[2];

// 运动模式检测功能开关
extern bool motion_mode_detection_enabled;


// alternating_speed模块中的外部变量
extern bool alternating_speed_enabled;
extern int alternating_interval_ms;
extern float alternating_speed_x;
extern float alternating_speed_y;
extern float speed_current_limit;
extern float feedforward_torque_value;

// continuous_torque_velocity_mode模块中的外部变量
extern float climbing_mode_torque;

// RS01电机库的外部变量和函数
#include "rs01_motor.h"
extern MI_Motor motors[2];

static const char *TAG = "motor_web_control";
static httpd_handle_t server = NULL;

// 网页首页处理函数
esp_err_t motor_web_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, motor_control_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 电机状态API处理函数
esp_err_t motor_web_status_api_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    cJSON *motor1 = cJSON_CreateObject();
    cJSON *motor2 = cJSON_CreateObject();

    // 添加电机1数据
    cJSON_AddNumberToObject(motor1, "position", motors[0].position);
    cJSON_AddNumberToObject(motor1, "velocity", motors[0].velocity);
    cJSON_AddNumberToObject(motor1, "current", motors[0].current);
    cJSON_AddNumberToObject(motor1, "temperature", motors[0].temperature);
    cJSON_AddItemToObject(json, "motor1", motor1);

    // 添加电机2数据
    cJSON_AddNumberToObject(motor2, "position", motors[1].position);
    cJSON_AddNumberToObject(motor2, "velocity", motors[1].velocity);
    cJSON_AddNumberToObject(motor2, "current", motors[1].current);
    cJSON_AddNumberToObject(motor2, "temperature", motors[1].temperature);
    cJSON_AddItemToObject(json, "motor2", motor2);

    // 添加系统状态
    cJSON_AddBoolToObject(json, "motor_control_enabled", motor_speed_control_enabled);
    cJSON_AddBoolToObject(json, "alternating_enabled", alternating_speed_enabled);
    cJSON_AddBoolToObject(json, "motion_detection_enabled", motion_mode_detection_enabled);
    cJSON_AddNumberToObject(json, "walking_mode", Get_Current_Walking_Mode());

    // 添加可配置参数状态
    cJSON_AddNumberToObject(json, "feedforward_torque", feedforward_torque_value);
    cJSON_AddNumberToObject(json, "climbing_torque", climbing_mode_torque);

    // 添加冲突状态信息
    cJSON_AddBoolToObject(json, "alternating_conflict", motion_mode_detection_enabled);
    cJSON_AddBoolToObject(json, "motion_detection_conflict", alternating_speed_enabled);

    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// 电机控制API处理函数
esp_err_t motor_web_control_api_handler(httpd_req_t *req)
{
    char buf[1000];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *action = cJSON_GetObjectItem(json, "action");
    if (!cJSON_IsString(action)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing action");
        return ESP_FAIL;
    }

    const char *response_msg = "Unknown action";

    if (strcmp(action->valuestring, "start") == 0) {
        motor_speed_control_enabled = true;
        response_msg = "电机控制已启动";
        ESP_LOGI(TAG, "网页请求启动电机控制");
    }
    else if (strcmp(action->valuestring, "stop") == 0) {
        motor_speed_control_enabled = false;
        Stop_Alternating_Speed();
        motion_mode_detection_enabled = false;
        // 停止所有电机
        motor_target_speed[0] = 0;
        motor_target_speed[1] = 0;
        response_msg = "电机控制已停止，所有模式已关闭";
        ESP_LOGI(TAG, "网页请求停止电机控制，关闭所有模式");
    }
    else if (strcmp(action->valuestring, "emergency") == 0) {
        motor_speed_control_enabled = false;
        Stop_Alternating_Speed();
        motion_mode_detection_enabled = false;
        motor_target_speed[0] = 0;
        motor_target_speed[1] = 0;
        // TODO: 添加紧急停止逻辑
        response_msg = "紧急停止已执行，所有模式已关闭";
        ESP_LOGW(TAG, "网页请求紧急停止，关闭所有模式");
    }
    else if (strcmp(action->valuestring, "mode") == 0) {
        cJSON *mode = cJSON_GetObjectItem(json, "mode");
        if (cJSON_IsNumber(mode)) {
            if (mode->valueint == 0) {
                Switch_To_Flat_Mode();
                response_msg = "已切换到平地模式";
            } else if (mode->valueint == 1) {
                Switch_To_Stairs_Mode();
                response_msg = "已切换到爬楼模式";
            }
            ESP_LOGI(TAG, "网页请求切换行走模式: %d", mode->valueint);
        }
    }
    else if (strcmp(action->valuestring, "start_alternating") == 0) {
        if (motion_mode_detection_enabled) {
            response_msg = "启动失败：智能运动控制正在运行，请先关闭智能运动控制";
            ESP_LOGW(TAG, "交替行走启动失败：智能运动控制冲突");
        } else {
            Start_Alternating_Speed();
            response_msg = "交替行走模式已启动";
            ESP_LOGI(TAG, "网页请求启动交替行走");
        }
    }
    else if (strcmp(action->valuestring, "stop_alternating") == 0) {
        Stop_Alternating_Speed();
        response_msg = "交替行走模式已停止";
        ESP_LOGI(TAG, "网页请求停止交替行走");
    }
    else if (strcmp(action->valuestring, "enable_motion_detection") == 0) {
        if (alternating_speed_enabled) {
            response_msg = "启动失败：交替行走模式正在运行，请先停止交替行走";
            ESP_LOGW(TAG, "智能运动控制启动失败：交替行走冲突");
        } else {
            motion_mode_detection_enabled = true;
            response_msg = "智能运动模式检测已启用";
            ESP_LOGI(TAG, "网页请求启用运动模式检测");
        }
    }
    else if (strcmp(action->valuestring, "disable_motion_detection") == 0) {
        motion_mode_detection_enabled = false;
        response_msg = "智能运动模式检测已关闭";
        ESP_LOGI(TAG, "网页请求关闭运动模式检测");
    }

    httpd_resp_send(req, response_msg, strlen(response_msg));
    cJSON_Delete(json);
    return ESP_OK;
}

// 运控参数设置API处理函数
esp_err_t motor_web_params_api_handler(httpd_req_t *req)
{
    char buf[1000];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *action = cJSON_GetObjectItem(json, "action");
    if (!cJSON_IsString(action)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing action");
        return ESP_FAIL;
    }

    const char *response_msg = "未知操作";

    if (strcmp(action->valuestring, "update_params") == 0) {
        // 更新电机运控参数
        cJSON *motor1 = cJSON_GetObjectItem(json, "motor1");
        cJSON *motor2 = cJSON_GetObjectItem(json, "motor2");

        if (motor1) {
            cJSON *pos = cJSON_GetObjectItem(motor1, "position");
            cJSON *speed = cJSON_GetObjectItem(motor1, "speed");
            cJSON *torque = cJSON_GetObjectItem(motor1, "torque");
            cJSON *kp = cJSON_GetObjectItem(motor1, "kp");
            cJSON *kd = cJSON_GetObjectItem(motor1, "kd");

            if (pos) motor_params[0].position = pos->valuedouble;
            if (speed) motor_params[0].speed = speed->valuedouble;
            if (torque) motor_params[0].torque = torque->valuedouble;
            if (kp) motor_params[0].kp = kp->valuedouble;
            if (kd) motor_params[0].kd = kd->valuedouble;
        }

        if (motor2) {
            cJSON *pos = cJSON_GetObjectItem(motor2, "position");
            cJSON *speed = cJSON_GetObjectItem(motor2, "speed");
            cJSON *torque = cJSON_GetObjectItem(motor2, "torque");
            cJSON *kp = cJSON_GetObjectItem(motor2, "kp");
            cJSON *kd = cJSON_GetObjectItem(motor2, "kd");

            if (pos) motor_params[1].position = pos->valuedouble;
            if (speed) motor_params[1].speed = speed->valuedouble;
            if (torque) motor_params[1].torque = torque->valuedouble;
            if (kp) motor_params[1].kp = kp->valuedouble;
            if (kd) motor_params[1].kd = kd->valuedouble;
        }

        // 立即发送参数给电机
        extern void unified_motor_control(int motor_id, const motor_control_params_t* params);
        unified_motor_control(0, &motor_params[0]);
        unified_motor_control(1, &motor_params[1]);

        response_msg = "运控参数已更新";
        ESP_LOGI(TAG, "网页更新运控参数");
    }
    else if (strcmp(action->valuestring, "update_alternating") == 0) {
        // 更新交替行走参数
        cJSON *interval = cJSON_GetObjectItem(json, "interval");
        cJSON *speedX = cJSON_GetObjectItem(json, "speedX");
        cJSON *speedY = cJSON_GetObjectItem(json, "speedY");
        cJSON *currentLimit = cJSON_GetObjectItem(json, "currentLimit");
        cJSON *feedforwardTorque = cJSON_GetObjectItem(json, "feedforwardTorque");

        if (interval) alternating_interval_ms = interval->valueint;
        if (speedX) alternating_speed_x = speedX->valuedouble;
        if (speedY) alternating_speed_y = speedY->valuedouble;
        if (currentLimit) speed_current_limit = currentLimit->valuedouble;
        if (feedforwardTorque) feedforward_torque_value = feedforwardTorque->valuedouble;

        response_msg = "交替行走参数已更新";
        ESP_LOGI(TAG, "网页更新交替行走参数，前馈力矩: %.2f", feedforward_torque_value);
    }
    else if (strcmp(action->valuestring, "update_motion_detection") == 0) {
        // 更新智能运动检测参数
        cJSON *climbingTorque = cJSON_GetObjectItem(json, "climbingTorque");

        if (climbingTorque) {
            climbing_mode_torque = climbingTorque->valuedouble;
        }

        response_msg = "智能运动检测参数已更新";
        ESP_LOGI(TAG, "网页更新智能运动检测参数，爬楼力矩: %.2f", climbing_mode_torque);
    }

    httpd_resp_send(req, response_msg, strlen(response_msg));
    cJSON_Delete(json);
    return ESP_OK;
}

// 启动HTTP服务器
httpd_handle_t motor_web_control_start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "启动HTTP服务器，端口: %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        // 注册URI处理函数
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = motor_web_index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t status_uri = {
            .uri       = "/api/status",
            .method    = HTTP_GET,
            .handler   = motor_web_status_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t control_uri = {
            .uri       = "/api/control",
            .method    = HTTP_POST,
            .handler   = motor_web_control_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &control_uri);

        httpd_uri_t params_uri = {
            .uri       = "/api/params",
            .method    = HTTP_POST,
            .handler   = motor_web_params_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &params_uri);

        ESP_LOGI(TAG, "HTTP服务器启动成功");
        return server;
    }

    ESP_LOGE(TAG, "HTTP服务器启动失败");
    return NULL;
}

// 停止HTTP服务器
esp_err_t motor_web_control_stop_server(httpd_handle_t server)
{
    if (server != NULL) {
        ESP_LOGI(TAG, "停止HTTP服务器");
        return httpd_stop(server);
    }
    return ESP_OK;
}

// 电机控制网页模块初始化
esp_err_t motor_web_control_init(void)
{
    ESP_LOGI(TAG, "初始化电机控制网页模块");

    // 启动HTTP服务器
    server = motor_web_control_start_server();

    if (server == NULL) {
        ESP_LOGE(TAG, "电机控制网页模块初始化失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "电机控制网页模块初始化成功");
    return ESP_OK;
}