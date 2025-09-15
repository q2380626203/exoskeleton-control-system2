/**
 * @file web_server.c
 * @brief 简化的Web服务器实现
 */

#include "web_server.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string.h>

static const char *TAG = "web_server";

// 全局变量
static httpd_handle_t server = NULL;
static MI_Motor* web_motors = NULL;

// 外部函数声明
extern void Start_Alternating_Speed(void);
extern void Stop_Alternating_Speed(void);
extern void Switch_To_Flat_Mode(void);
extern void Switch_To_Stairs_Mode(void);
extern int alternating_interval_ms;
extern float alternating_speed_x;
extern float alternating_speed_y;
extern float speed_current_limit;

// HTML页面
static const char* html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <meta charset='UTF-8'>"
"    <title>外骨骼控制</title>"
"    <style>"
"        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
"        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }"
"        h1 { text-align: center; color: #333; }"
"        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }"
"        .btn { padding: 10px 15px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; }"
"        .btn-enable { background: #28a745; color: white; }"
"        .btn-disable { background: #dc3545; color: white; }"
"        .btn-alt { background: #17a2b8; color: white; }"
"        .btn-config { background: #fd7e14; color: white; }"
"        .btn:hover { opacity: 0.8; }"
"        input { width: 80px; padding: 5px; margin: 3px; }"
"        label { display: inline-block; width: 120px; }"
"    </style>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🦾 外骨骼控制系统</h1>"
"        "
"        <div class='section'>"
"            <h3>电机状态监控</h3>"
"            <div style='display: flex; gap: 20px;'>"
"                <div style='flex: 1; border: 1px solid #ccc; padding: 10px; border-radius: 5px;'>"
"                    <h4>电机1</h4>"
"                    <div>位置: <span id='m1_pos'>--</span> rad</div>"
"                    <div>速度: <span id='m1_vel'>--</span> rad/s</div>"
"                    <div>电流: <span id='m1_cur'>--</span> A</div>"
"                    <div>温度: <span id='m1_temp'>--</span> °C</div>"
"                </div>"
"                <div style='flex: 1; border: 1px solid #ccc; padding: 10px; border-radius: 5px;'>"
"                    <h4>电机2</h4>"
"                    <div>位置: <span id='m2_pos'>--</span> rad</div>"
"                    <div>速度: <span id='m2_vel'>--</span> rad/s</div>"
"                    <div>电流: <span id='m2_cur'>--</span> A</div>"
"                    <div>温度: <span id='m2_temp'>--</span> °C</div>"
"                </div>"
"            </div>"
"        </div>"
"        "
"        <div class='section'>"
"            <h3>电机1控制</h3>"
"            <button class='btn btn-enable' onclick=\"motorControl(1,'enable')\">使能</button>"
"            <button class='btn btn-disable' onclick=\"motorControl(1,'disable')\">失能</button>"
"        </div>"
"        "
"        <div class='section'>"
"            <h3>电机2控制</h3>"
"            <button class='btn btn-enable' onclick=\"motorControl(2,'enable')\">使能</button>"
"            <button class='btn btn-disable' onclick=\"motorControl(2,'disable')\">失能</button>"
"        </div>"
"        "
"        "
"        <div class='section'>"
"            <h3>交替速度控制</h3>"
"            <button class='btn btn-alt' onclick=\"alternatingControl('start')\">开启交替</button>"
"            <button class='btn btn-disable' onclick=\"alternatingControl('stop')\">停止交替</button>"
"        </div>"
"        "
"        <div class='section'>"
"            <h3>行走模式切换</h3>"
"            <button class='btn btn-alt' onclick=\"modeControl('flat')\">平地模式</button>"
"            <button class='btn btn-config' onclick=\"modeControl('stairs')\">爬楼模式</button>"
"            <p><small>平地: 800ms/1.0rad/5A | 爬楼: 1000ms/2.5rad/10A</small></p>"
"        </div>"
"        "
"        <div class='section'>"
"            <h3>参数设置</h3>"
"            <div><label>间隔时间(ms):</label><input type='number' id='interval' value='800'></div>"
"            <div><label>速度X(rad/s):</label><input type='number' id='speed_x' value='1.0' step='0.1'></div>"
"            <div><label>速度Y(rad/s):</label><input type='number' id='speed_y' value='1.0' step='0.1'></div>"
"            <div><label>电流限制(A):</label><input type='number' id='current_limit' value='5.0' step='0.1'></div>"
"            <button class='btn btn-config' onclick=\"setConfig()\">应用参数</button>"
"            <div style='font-size:12px; color:#666; margin-top:8px;'>"
"                提示: 平地(800,1,1,5) | 爬楼(1000,5,5,10)"
"            </div>"
"        </div>"
"        "
"        <div style='text-align: center; margin-top: 20px;'>"
"            <button class='btn' onclick='location.reload()'>刷新</button>"
"        </div>"
"    </div>"
"    "
"    <script>"
"        function motorControl(id, action) {"
"            console.log('电机控制:', id, action);"
"            fetch('/motor/' + id + '/' + action, {method: 'POST'})"
"            .then(r => r.text())"
"            .then(data => alert(data))"
"            .catch(e => alert('错误: ' + e));"
"        }"
"        "
"        function alternatingControl(action) {"
"            console.log('交替控制:', action);"
"            fetch('/alt/' + action, {method: 'POST'})"
"            .then(r => r.text())"
"            .then(data => alert(data))"
"            .catch(e => alert('错误: ' + e));"
"        }"
"        "
"        function modeControl(mode) {"
"            console.log('模式控制:', mode);"
"            fetch('/mode/' + mode, {method: 'POST'})"
"            .then(r => r.text())"
"            .then(data => alert(data))"
"            .catch(e => alert('错误: ' + e));"
"        }"
"        "
"        function setConfig() {"
"            console.log('设置参数');"
"            const data = {"
"                interval: parseInt(document.getElementById('interval').value),"
"                speed_x: parseFloat(document.getElementById('speed_x').value),"
"                speed_y: parseFloat(document.getElementById('speed_y').value),"
"                current_limit: parseFloat(document.getElementById('current_limit').value)"
"            };"
"            fetch('/config', {"
"                method: 'POST',"
"                headers: {'Content-Type': 'application/json'},"
"                body: JSON.stringify(data)"
"            })"
"            .then(r => r.text())"
"            .then(data => alert(data))"
"            .catch(e => alert('错误: ' + e));"
"        }"
"        "
"        function updateStatus() {"
"            fetch('/status')"
"            .then(r => r.json())"
"            .then(data => {"
"                if (data.motor1) {"
"                    document.getElementById('m1_pos').textContent = data.motor1.position.toFixed(3);"
"                    document.getElementById('m1_vel').textContent = data.motor1.velocity.toFixed(3);"
"                    document.getElementById('m1_cur').textContent = data.motor1.current.toFixed(3);"
"                    document.getElementById('m1_temp').textContent = data.motor1.temperature.toFixed(1);"
"                }"
"                if (data.motor2) {"
"                    document.getElementById('m2_pos').textContent = data.motor2.position.toFixed(3);"
"                    document.getElementById('m2_vel').textContent = data.motor2.velocity.toFixed(3);"
"                    document.getElementById('m2_cur').textContent = data.motor2.current.toFixed(3);"
"                    document.getElementById('m2_temp').textContent = data.motor2.temperature.toFixed(1);"
"                }"
"            })"
"            .catch(e => console.log('状态更新失败:', e));"
"        }"
"        "
"        setInterval(updateStatus, 500);"
"        updateStatus();"
"    </script>"
"</body>"
"</html>";

// HTTP处理器函数
static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "主页请求");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t motor_handler(httpd_req_t *req)
{
    char url[64];
    strncpy(url, req->uri, sizeof(url) - 1);
    url[sizeof(url) - 1] = '\0';
    
    ESP_LOGI(TAG, "电机控制请求: %s", url);
    
    int motor_id = 0;
    char action[20] = {0};
    
    if (sscanf(url, "/motor/%d/%19s", &motor_id, action) != 2) {
        httpd_resp_send(req, "URL格式错误", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    if (motor_id < 1 || motor_id > 2 || web_motors == NULL) {
        httpd_resp_send(req, "电机ID无效或未初始化", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    MI_Motor* motor = &web_motors[motor_id - 1];
    
    if (strcmp(action, "enable") == 0) {
        Motor_Enable(motor);
        httpd_resp_send(req, "电机使能成功", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "电机%d使能", motor_id);
    }
    else if (strcmp(action, "disable") == 0) {
        Motor_Reset(motor, 0);
        httpd_resp_send(req, "电机失能成功", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "电机%d失能", motor_id);
    }
    else {
        httpd_resp_send(req, "未知动作", HTTPD_RESP_USE_STRLEN);
    }
    
    return ESP_OK;
}

static esp_err_t alt_handler(httpd_req_t *req)
{
    char url[64];
    strncpy(url, req->uri, sizeof(url) - 1);
    url[sizeof(url) - 1] = '\0';
    
    ESP_LOGI(TAG, "交替控制请求: %s", url);
    
    if (strstr(url, "/alt/start")) {
        Start_Alternating_Speed();
        httpd_resp_send(req, "交替速度已启动", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "交替速度启动");
    }
    else if (strstr(url, "/alt/stop")) {
        Stop_Alternating_Speed();
        httpd_resp_send(req, "交替速度已停止", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "交替速度停止");
    }
    else {
        httpd_resp_send(req, "未知交替动作", HTTPD_RESP_USE_STRLEN);
    }
    
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req)
{
    char url[64];
    strncpy(url, req->uri, sizeof(url) - 1);
    url[sizeof(url) - 1] = '\0';
    
    ESP_LOGI(TAG, "模式切换请求: %s", url);
    
    if (strstr(url, "/mode/flat")) {
        Switch_To_Flat_Mode();
        httpd_resp_send(req, "已切换到平地模式", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "切换到平地模式");
    }
    else if (strstr(url, "/mode/stairs")) {
        Switch_To_Stairs_Mode();
        httpd_resp_send(req, "已切换到爬楼模式", HTTPD_RESP_USE_STRLEN);
        ESP_LOGI(TAG, "切换到爬楼模式");
    }
    else {
        httpd_resp_send(req, "未知模式", HTTPD_RESP_USE_STRLEN);
    }
    
    return ESP_OK;
}

static esp_err_t config_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "参数配置请求");

    char buf[200];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send(req, "读取数据失败", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "收到参数: %s", buf);

    // 简单解析JSON
    int interval = 800;
    float speed_x = 1.0f;
    float speed_y = 1.0f;
    float current_limit = 5.0f;

    char *p;
    if ((p = strstr(buf, "\"interval\":")) != NULL) {
        interval = atoi(p + 11);
    }
    if ((p = strstr(buf, "\"speed_x\":")) != NULL) {
        speed_x = atof(p + 10);
    }
    if ((p = strstr(buf, "\"speed_y\":")) != NULL) {
        speed_y = atof(p + 10);
    }
    if ((p = strstr(buf, "\"current_limit\":")) != NULL) {
        current_limit = atof(p + 16);
    }

    // 更新全局变量
    alternating_interval_ms = interval;
    alternating_speed_x = speed_x;
    alternating_speed_y = speed_y;
    speed_current_limit = current_limit;

    ESP_LOGI(TAG, "参数已更新: 间隔%dms, X=%.1f, Y=%.1f, 电流=%.1f",
             interval, speed_x, speed_y, current_limit);

    char response[100];
    snprintf(response, sizeof(response), "参数更新成功: %dms, %.1f, %.1f, %.1fA",
             interval, speed_x, speed_y, current_limit);
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "电机状态请求");

    char status_json[512];
    int json_len = get_motor_status_json(status_json, sizeof(status_json));

    if (json_len > 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, status_json, json_len);
    } else {
        httpd_resp_send(req, "{\"error\":\"状态获取失败\"}", HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}


// URI处理器定义
static const httpd_uri_t uris[] = {
    { .uri = "/",             .method = HTTP_GET,  .handler = root_handler },
    { .uri = "/motor/1/enable",  .method = HTTP_POST, .handler = motor_handler },
    { .uri = "/motor/1/disable", .method = HTTP_POST, .handler = motor_handler },
    { .uri = "/motor/2/enable",  .method = HTTP_POST, .handler = motor_handler },
    { .uri = "/motor/2/disable", .method = HTTP_POST, .handler = motor_handler },
    { .uri = "/alt/start",    .method = HTTP_POST, .handler = alt_handler },
    { .uri = "/alt/stop",     .method = HTTP_POST, .handler = alt_handler },
    { .uri = "/mode/flat",    .method = HTTP_POST, .handler = mode_handler },
    { .uri = "/mode/stairs",  .method = HTTP_POST, .handler = mode_handler },
    { .uri = "/config",       .method = HTTP_POST, .handler = config_handler },
    { .uri = "/status",       .method = HTTP_GET,  .handler = status_handler },
};

// 公共函数实现
esp_err_t web_server_start(void)
{
    if (server != NULL) {
        ESP_LOGW(TAG, "服务器已运行");
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 16;
    
    ESP_LOGI(TAG, "启动Web服务器...");
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 注册所有URI
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
        ESP_LOGI(TAG, "注册URI: %s", uris[i].uri);
    }
    
    ESP_LOGI(TAG, "Web服务器启动成功! 访问: http://192.168.4.1");
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (server == NULL) {
        ESP_LOGW(TAG, "服务器未运行");
        return ESP_OK;
    }
    
    esp_err_t ret = httpd_stop(server);
    server = NULL;
    ESP_LOGI(TAG, "Web服务器已停止");
    return ret;
}

void web_server_set_motors(MI_Motor* motor_array)
{
    web_motors = motor_array;
    ESP_LOGI(TAG, "设置电机数组: %p", (void*)motor_array);
}

int get_motor_status_json(char* buffer, size_t buffer_size)
{
    if (buffer == NULL || web_motors == NULL) {
        return -1;
    }

    int len = snprintf(buffer, buffer_size,
        "{"
        "\"motor1\":{"
            "\"position\":%.3f,"
            "\"velocity\":%.3f,"
            "\"current\":%.3f,"
            "\"temperature\":%.1f"
        "},"
        "\"motor2\":{"
            "\"position\":%.3f,"
            "\"velocity\":%.3f,"
            "\"current\":%.3f,"
            "\"temperature\":%.1f"
        "}"
        "}",
        web_motors[0].position,
        web_motors[0].velocity,
        web_motors[0].current,
        web_motors[0].temperature,
        web_motors[1].position,
        web_motors[1].velocity,
        web_motors[1].current,
        web_motors[1].temperature
    );

    return (len > 0 && len < buffer_size) ? len : -1;
}