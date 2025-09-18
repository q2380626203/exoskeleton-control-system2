idf_component_register(SRCS "uart_motor_transmit.c" "main.c"
                           "system_init.c"
                           "wifi_softap_module.c"
                           "motor_web_control.c"
                           "rs01_motor.c"
                           "button_detector.c"
                           "voice_module.c"
                           "alternating_speed.c"
                           "continuous_torque_velocity_mode.c"
                           "velocity_tracking_mode.c"
                    PRIV_REQUIRES esp_wifi nvs_flash driver esp_timer esp_http_server json
                    INCLUDE_DIRS ".")
