#ifndef _UART_MOTOR_TRANSMIT_H__
#define _UART_MOTOR_TRANSMIT_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "rs01_motor.h"

// UART配置 - 使用UART2，TX=1, RX=2
#define TX_UART_NUM       UART_NUM_2
#define TX_UART_TX_PIN    1
#define TX_UART_RX_PIN    2
#define TX_UART_BAUDRATE  115200

// 数据包格式定义
#define MOTOR_DATA_PACKET_SIZE 32
#define PACKET_HEADER          0xAA
#define PACKET_FOOTER          0x55

// 电机数据传输结构体
typedef struct {
    uint8_t header;         // 数据包头 0xAA
    uint8_t motor_count;    // 电机数量

    // 电机1数据
    uint8_t motor1_id;      // 电机1 ID
    float motor1_position;  // 位置
    float motor1_velocity;  // 速度
    float motor1_current;   // 电流
    float motor1_temp;      // 温度

    // 电机2数据
    uint8_t motor2_id;      // 电机2 ID
    float motor2_position;  // 位置
    float motor2_velocity;  // 速度
    float motor2_current;   // 电流
    float motor2_temp;      // 温度

    uint8_t checksum;       // 校验和
    uint8_t footer;         // 数据包尾 0x55
} __attribute__((packed)) motor_data_packet_t;

// 函数声明
void uart_motor_transmit_init(void);
void send_motor_data(const MI_Motor* motors, uint8_t motor_count);
uint8_t calculate_checksum(const uint8_t* data, size_t length);
void motor_data_transmit_task(void* pvParameters);

#endif // _UART_MOTOR_TRANSMIT_H__