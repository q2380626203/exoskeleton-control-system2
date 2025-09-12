/**
 * @file wifi_softap_module.c
 * @brief WiFi SoftAP模块实现文件
 */

#include <string.h>
#include "wifi_softap_module.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "wifi_softap_module";

// 模块内部状态
static bool is_initialized = false;
static bool is_started = false;
static uint8_t connected_count = 0;
static wifi_softap_event_cb_t user_event_callback = NULL;

/**
 * @brief WiFi事件处理函数
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        connected_count++;
        ESP_LOGI(TAG, "设备 "MACSTR" 连接，AID=%d，当前连接数=%d",
                 MAC2STR(event->mac), event->aid, connected_count);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        connected_count--;
        ESP_LOGI(TAG, "设备 "MACSTR" 断开，AID=%d，原因=%d，当前连接数=%d",
                 MAC2STR(event->mac), event->aid, event->reason, connected_count);
    }
    
    // 调用用户回调函数
    if (user_event_callback != NULL) {
        user_event_callback(event_id, event_data);
    }
}

esp_err_t wifi_softap_init(const wifi_softap_config_t* config, wifi_softap_event_cb_t event_callback)
{
    if (is_initialized) {
        ESP_LOGW(TAG, "WiFi热点模块已初始化");
        return ESP_OK;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "配置参数不能为空");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 保存用户回调函数
    user_event_callback = event_callback;
    
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册WiFi事件处理
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // 配置WiFi热点
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(config->ssid),
            .channel = config->channel,
            .max_connection = config->max_connection,
            .authmode = config->authmode,
            .pmf_cfg = {
                .required = true,
            },
        },
    };
    
    // 复制SSID和密码
    strncpy((char*)wifi_config.ap.ssid, config->ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char*)wifi_config.ap.password, config->password, sizeof(wifi_config.ap.password) - 1);
    
    // 如果密码为空，设置为开放模式
    if (strlen(config->password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    is_initialized = true;
    is_started = true;
    connected_count = 0;
    
    ESP_LOGI(TAG, "WiFi热点启动完成 - SSID:%s, 信道:%d, 最大连接数:%d",
             config->ssid, config->channel, config->max_connection);
    
    return ESP_OK;
}


uint8_t wifi_softap_get_connected_count(void)
{
    return connected_count;
}

wifi_softap_config_t wifi_softap_get_default_config(void)
{
    wifi_softap_config_t default_config = {
        .ssid = "ESP32-SoftAP",
        .password = "12345678",
        .channel = 1,
        .max_connection = 4,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
        .authmode = WIFI_AUTH_WPA3_PSK,
#else
        .authmode = WIFI_AUTH_WPA2_PSK,
#endif
    };
    
    return default_config;
}