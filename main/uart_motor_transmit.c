#include "uart_motor_transmit.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART_TX";

/**
 * @brief 初始化UART1用于电机数据传输
 */
void uart_motor_transmit_init(void) {
    uart_config_t uart_config = {
        .baud_rate = TX_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(TX_UART_NUM, &uart_config));

    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(TX_UART_NUM, TX_UART_TX_PIN, TX_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(TX_UART_NUM, 256, 256, 0, NULL, 0));

    ESP_LOGI(TAG, "UART电机数据传输初始化完成 - TX引脚:%d, RX引脚:%d, 波特率:%d",
             TX_UART_TX_PIN, TX_UART_RX_PIN, TX_UART_BAUDRATE);
}

/**
 * @brief 计算校验和
 * @param data 数据指针
 * @param length 数据长度
 * @return 校验和
 */
uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 发送电机数据
 * @param motors 电机数组指针
 * @param motor_count 电机数量
 */
void send_motor_data(const MI_Motor* motors, uint8_t motor_count) {
    motor_data_packet_t packet;

    // 填充数据包头部
    packet.header = PACKET_HEADER;
    packet.motor_count = motor_count;

    // 填充电机1数据
    if (motor_count >= 1) {
        packet.motor1_id = motors[0].id;
        packet.motor1_position = motors[0].position;
        packet.motor1_velocity = motors[0].velocity;
        packet.motor1_current = motors[0].current;
        packet.motor1_temp = motors[0].temperature;
    } else {
        packet.motor1_id = 0;
        packet.motor1_position = 0.0f;
        packet.motor1_velocity = 0.0f;
        packet.motor1_current = 0.0f;
        packet.motor1_temp = 0.0f;
    }

    // 填充电机2数据
    if (motor_count >= 2) {
        packet.motor2_id = motors[1].id;
        packet.motor2_position = motors[1].position;
        packet.motor2_velocity = motors[1].velocity;
        packet.motor2_current = motors[1].current;
        packet.motor2_temp = motors[1].temperature;
    } else {
        packet.motor2_id = 0;
        packet.motor2_position = 0.0f;
        packet.motor2_velocity = 0.0f;
        packet.motor2_current = 0.0f;
        packet.motor2_temp = 0.0f;
    }

    // 计算校验和（除校验和和尾部字段外的所有数据）
    packet.checksum = calculate_checksum((uint8_t*)&packet,
                                        sizeof(motor_data_packet_t) - 2);
    packet.footer = PACKET_FOOTER;

    // 通过UART发送数据包
    int written = uart_write_bytes(TX_UART_NUM, (const char*)&packet,
                                   sizeof(motor_data_packet_t));

    if (written != sizeof(motor_data_packet_t)) {
        ESP_LOGE(TAG, "UART发送失败，期望:%d字节，实际:%d字节",
                 sizeof(motor_data_packet_t), written);
    } else {
        ESP_LOGD(TAG, "电机数据发送成功，电机数量:%d", motor_count);
    }
}

/**
 * @brief 电机数据传输任务
 * @param pvParameters 任务参数
 */
void motor_data_transmit_task(void* pvParameters) {
    ESP_LOGI(TAG, "电机数据传输任务启动");

    while (1) {
        // 发送电机数据（外部电机数组）
        send_motor_data(motors, 2);

        // 每25ms发送一次，匹配电机数据更新频率
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}