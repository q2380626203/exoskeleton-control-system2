idf_component_register(SRCS "main.c" 
                           "system_init.c"
                           "wifi_softap_module.c"
                           "web_server.c"
                           "rs01_motor.c" 
                           "button_detector.c" 
                           "voice_module.c"
                    PRIV_REQUIRES esp_wifi esp_http_server nvs_flash driver esp_timer
                    INCLUDE_DIRS ".")
