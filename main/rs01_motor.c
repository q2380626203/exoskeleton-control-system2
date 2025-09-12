#include "rs01_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RS01_MOTOR";

// Global variables
uart_config_t motor_uart_config;
MI_Motor motors[2];
MotorDataCallback data_callback = NULL;


// Helper function to convert float to uint for CAN transmission
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Helper function to convert uint to float from CAN reception
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}

/**
 * @brief Sends a raw 12-byte CAN frame (4-byte ID + 8-byte data) over UART.
 * @param frame Pointer to the can_frame_t structure.
 */
void UART_Send_Frame(const can_frame_t* frame) {
    // Construct the 29-bit extended CAN ID directly from the frame structure
    uint32_t extended_id = ((uint32_t)frame->type << 24) |
                           ((uint32_t)frame->data << 8) |
                           frame->target_id;

    uint8_t packet[CAN_RAW_FRAME_LENGTH];

    // Pack the 4-byte extended CAN ID (big-endian)
    packet[0] = (extended_id >> 24) & 0xFF;
    packet[1] = (extended_id >> 16) & 0xFF;
    packet[2] = (extended_id >> 8) & 0xFF;
    packet[3] = extended_id & 0xFF;

    // Copy the 8-byte data payload
    memcpy(&packet[4], frame->payload, 8);

    // 打印原始发送数据
    // ESP_LOGI(TAG, "发送原始数据[%d字节]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
    //          CAN_RAW_FRAME_LENGTH,
    //          packet[0], packet[1], packet[2], packet[3],
    //          packet[4], packet[5], packet[6], packet[7],
    //          packet[8], packet[9], packet[10], packet[11]);
    
    // ESP_LOGI(TAG, "帧信息: type=0x%02X, data=0x%04X, target_id=0x%02X", 
    //          frame->type, frame->data, frame->target_id);

    // Send via UART
    int written = uart_write_bytes(MOTOR_UART_NUM, packet, CAN_RAW_FRAME_LENGTH);
    if (written != CAN_RAW_FRAME_LENGTH) {
        ESP_LOGE(TAG, "UART write failed, expected %d, wrote %d", CAN_RAW_FRAME_LENGTH, written);
    }
}

/**
 * @brief Initialize UART for motor communication
 * @param callback Callback function for motor data updates
 */
void UART_Rx_Init(MotorDataCallback callback) {
    data_callback = callback;

    // Configure UART parameters
    motor_uart_config.baud_rate = UART_BAUDRATE;
    motor_uart_config.data_bits = UART_DATA_8_BITS;
    motor_uart_config.parity = UART_PARITY_DISABLE;
    motor_uart_config.stop_bits = UART_STOP_BITS_1;
    motor_uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    motor_uart_config.source_clk = UART_SCLK_DEFAULT;

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(MOTOR_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MOTOR_UART_NUM, &motor_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MOTOR_UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Initialize motor structures
    for (int i = 0; i < 2; i++) {
        motors[i].id = (i == 0) ? MOTER_1_ID : MOTER_2_ID;
        motors[i].position = 0.0f;
        motors[i].velocity = 0.0f;
        motors[i].current = 0.0f;
        motors[i].temperature = 0.0f;
        motors[i].mode = 0;
        motors[i].error = 0;
        motors[i].error_uncalibrated = false;
        motors[i].error_overload = false;
        motors[i].error_magnetic_encoder = false;
        motors[i].error_over_temperature = false;
        motors[i].error_driver_fault = false;
        motors[i].error_undervoltage = false;
    }

    ESP_LOGI(TAG, "UART initialized for motor communication");
}

/**
 * @brief Handle incoming UART data and parse motor feedback
 */
void handle_uart_rx() {
    uint8_t buffer[CAN_RAW_FRAME_LENGTH];
    int length = uart_read_bytes(MOTOR_UART_NUM, buffer, CAN_RAW_FRAME_LENGTH, pdMS_TO_TICKS(10));
    
    if (length == CAN_RAW_FRAME_LENGTH) {
        // Parse the received CAN frame
        uint32_t extended_id = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
        uint8_t motor_id = extended_id & 0xFF;
        uint16_t data_field = (extended_id >> 8) & 0xFFFF;
        uint8_t frame_type = (extended_id >> 24) & 0x1F;

        // Find the motor by ID
        MI_Motor* motor = NULL;
        for (int i = 0; i < 2; i++) {
            if (motors[i].id == motor_id) {
                motor = &motors[i];
                break;
            }
        }

        if (motor != NULL && frame_type == 2) { // Motor feedback frame
            // Parse feedback data from payload
            uint16_t pos_raw = (buffer[4] << 8) | buffer[5];
            uint16_t vel_raw = (buffer[6] << 8) | buffer[7];
            uint16_t cur_raw = (buffer[8] << 8) | buffer[9];
            uint16_t temp_raw = (buffer[10] << 8) | buffer[11];

            // Convert to float values
            motor->position = uint_to_float(pos_raw, P_MIN, P_MAX, 16);
            motor->velocity = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
            motor->current = uint_to_float(cur_raw, T_MIN, T_MAX, 16);
            motor->temperature = (float)temp_raw / 10.0f;

            // Parse error flags from data_field
            motor->error = (data_field >> 10) & 0x3F;
            motor->mode = data_field & 0x3;

            motor->error_uncalibrated = (motor->error & 0x01) != 0;
            motor->error_overload = (motor->error & 0x02) != 0;
            motor->error_magnetic_encoder = (motor->error & 0x04) != 0;
            motor->error_over_temperature = (motor->error & 0x08) != 0;
            motor->error_driver_fault = (motor->error & 0x10) != 0;
            motor->error_undervoltage = (motor->error & 0x20) != 0;

            // Call callback if registered
            if (data_callback) {
                data_callback(motor);
            }
        }
    }
}

// Motor control functions
void Motor_Enable(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x03;
    frame.target_id = motor->id;
    frame.data = MASTER_ID; // 使用MASTER_ID
    memset(frame.payload, 0, sizeof(frame.payload)); // Data payload is all zeros for enable command
    
    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d enabled", motor->id);
}

void Motor_Reset(MI_Motor* motor, uint8_t clear_error) {
    can_frame_t frame;
    frame.type = 0x04;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    if (clear_error) {
        frame.payload[0] = 1;
    }
    
    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d reset", motor->id);
}

void Motor_Set_Zero(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x06;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    frame.payload[0] = 1;
    
    UART_Send_Frame(&frame);
    ESP_LOGI(TAG, "Motor %d set zero position", motor->id);
}

void Set_CurMode(MI_Motor* motor, float current) {
    Set_SingleParameter(motor, IQ_REF, current);
}

void Change_Mode(MI_Motor* motor, uint8_t mode) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18
    frame.target_id = motor->id;
    frame.data = MASTER_ID;

    memset(frame.payload, 0, sizeof(frame.payload));
    
    uint16_t index = RUN_MODE;
    // Parameter index (little-endian in payload)
    frame.payload[0] = index & 0xFF;
    frame.payload[1] = (index >> 8) & 0xFF;

    // Mode value
    frame.payload[4] = mode;

    UART_Send_Frame(&frame);
}

void Set_SingleParameter(MI_Motor* motor, uint16_t parameter, float value) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18 - 正确的参数设置类型
    frame.data = MASTER_ID; // 使用MASTER_ID
    frame.target_id = motor->id;
    
    memset(frame.payload, 0, sizeof(frame.payload));
    
    // Parameter index (little-endian in payload)
    frame.payload[0] = parameter & 0xFF;
    frame.payload[1] = (parameter >> 8) & 0xFF;
    
    // Parameter value (float, little-endian)
    memcpy(&frame.payload[4], &value, sizeof(float));
    
    UART_Send_Frame(&frame);
}

void Motor_ControlMode(MI_Motor* motor, float torque, float position, float speed, float kp, float kd) {
    can_frame_t frame;
    frame.type = 1;
    frame.data = 0x7005; // Control mode parameter
    frame.target_id = motor->id;
    
    // Pack control parameters into payload
    uint16_t pos_cmd = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t vel_cmd = float_to_uint(speed, V_MIN, V_MAX, 16);
    uint16_t kp_cmd = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_cmd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t torque_cmd = float_to_uint(torque, T_MIN, T_MAX, 16);
    
    frame.payload[0] = pos_cmd >> 8;
    frame.payload[1] = pos_cmd & 0xFF;
    frame.payload[2] = vel_cmd >> 8;
    frame.payload[3] = vel_cmd & 0xFF;
    frame.payload[4] = (kp_cmd >> 4) & 0xFF;
    frame.payload[5] = ((kp_cmd & 0xF) << 4) | ((kd_cmd >> 8) & 0xF);
    frame.payload[6] = kd_cmd & 0xFF;
    frame.payload[7] = torque_cmd & 0xFF;
    
    UART_Send_Frame(&frame);
}