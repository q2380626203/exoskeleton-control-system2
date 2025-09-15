#include "rs01_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RS01_MOTOR";

// Global variables
uart_config_t motor_uart_config;
MI_Motor motors[2];
MotorDataCallback data_callback = NULL;

// 调试打印时间控制（每2秒打印一次）
static TickType_t lastPrintTime[2] = {0, 0};
static const TickType_t PRINT_INTERVAL = pdMS_TO_TICKS(2000);

// Static function declarations
static void parse_can_frame(const uint8_t* can_payload);


// Helper function for linear mapping (for Motor_ControlMode)
uint16_t float_to_uint16_linear(float value, float min_val, float max_val) {
    // 对称映射：0值映射到0x8000(32768)
    if (value < min_val) value = min_val;
    if (value > max_val) value = max_val;

    // 计算相对于中点的偏移
    float center = (min_val + max_val) / 2.0f;
    float half_range = (max_val - min_val) / 2.0f;

    // 将[-half_range, half_range]映射到[0, 65535]，中心点0x8000
    float normalized = (value - center) / half_range;  // [-1, 1]
    return (uint16_t)(32768 + normalized * 32768);     // 0x8000 ± 32768
}

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
    static uint8_t packet_buffer[CAN_RAW_FRAME_LENGTH];
    static uint8_t packet_index = 0;
    
    // Read available bytes one by one to build complete packets
    size_t available_bytes = 0;
    uart_get_buffered_data_len(MOTOR_UART_NUM, &available_bytes);
    
    while (available_bytes > 0) {
        uint8_t byte;
        int length = uart_read_bytes(MOTOR_UART_NUM, &byte, 1, pdMS_TO_TICKS(1));
        
        if (length == 1) {
            if (packet_index < CAN_RAW_FRAME_LENGTH) {
                packet_buffer[packet_index++] = byte;
            } else {
                // Buffer overflow, reset
                packet_index = 0;
                packet_buffer[packet_index++] = byte;
            }
            
            // When we have a complete packet, process it
            if (packet_index >= CAN_RAW_FRAME_LENGTH) {
                // 减少调试输出频率，只在需要时打印
                // ESP_LOGI(TAG, "RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                //          packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3],
                //          packet_buffer[4], packet_buffer[5], packet_buffer[6], packet_buffer[7],
                //          packet_buffer[8], packet_buffer[9], packet_buffer[10], packet_buffer[11]);
                
                parse_can_frame(packet_buffer);
                packet_index = 0;
            }
        }
        
        // Update available bytes count
        uart_get_buffered_data_len(MOTOR_UART_NUM, &available_bytes);
    }
}

/**
 * @brief Parses a received 12-byte CAN frame
 * @param can_payload Pointer to the 12-byte array
 */
static void parse_can_frame(const uint8_t* can_payload) {
    uint32_t extended_id = ((uint32_t)can_payload[0] << 24) |
                           ((uint32_t)can_payload[1] << 16) |
                           ((uint32_t)can_payload[2] << 8) |
                           can_payload[3];

    uint8_t type = (extended_id >> 24) & 0x1F;

    // We are interested in feedback frames (Type 2) and auto-report frames (Type 24)
    if (type == 0x02 || type == 0x18) {
        uint8_t motor_id = (extended_id >> 8) & 0xFF;
        
        // Check if the motor ID is one we are controlling
        if (motor_id >= MOTER_1_ID && motor_id <= MOTER_2_ID) {
            MI_Motor* motor = &motors[motor_id - 1]; // Use motor_id - 1 for array index
            motor->id = motor_id; // Explicitly assign the parsed motor ID
            const uint8_t* data = &can_payload[4];

            // Parse data payload according to Type 2 format
            uint16_t raw_angle = (data[0] << 8) | data[1];
            uint16_t raw_speed = (data[2] << 8) | data[3];
            uint16_t raw_torque = (data[4] << 8) | data[5];
            uint16_t raw_temp = (data[6] << 8) | data[7];

            // Convert raw data to physical units
            motor->position = uint_to_float(raw_angle, P_MIN, P_MAX, 16);
            motor->velocity = uint_to_float(raw_speed, V_MIN, V_MAX, 16);
            motor->current = uint_to_float(raw_torque, T_MIN, T_MAX, 16);
            motor->temperature = raw_temp / 10.0f;
            
            // --- Detailed Status Parsing ---
            uint32_t status_part = extended_id >> 16;
            
            // Parse combined error code
            motor->error = status_part & 0x3F; // Bits 16-21

            // Parse individual error flags
            motor->error_undervoltage      = (status_part >> 0) & 0x01; // Bit 16
            motor->error_driver_fault      = (status_part >> 1) & 0x01; // Bit 17
            motor->error_over_temperature  = (status_part >> 2) & 0x01; // Bit 18
            motor->error_magnetic_encoder  = (status_part >> 3) & 0x01; // Bit 19
            motor->error_overload          = (status_part >> 4) & 0x01; // Bit 20
            motor->error_uncalibrated      = (status_part >> 5) & 0x01; // Bit 21

            // Parse mode status
            motor->mode = (status_part >> 6) & 0x03; // Bits 22-23

            // --- 调试打印解析数据（每2秒一次） ---
            TickType_t currentTime = xTaskGetTickCount();
            int motorIndex = (motor->id == 1) ? 0 : 1; // 电机1对应索引0，电机2对应索引1
            
            if (currentTime - lastPrintTime[motorIndex] >= PRINT_INTERVAL) {
                ESP_LOGI(TAG, "[Parsed] ID: %d, Pos: %.2f, Vel: %.2f, Cur: %.2f, Temp: %.1f, Mode: %d, Err: 0x%X",
                         motor->id, motor->position, motor->velocity, motor->current, motor->temperature, motor->mode, motor->error);
                lastPrintTime[motorIndex] = currentTime;
            }

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

void Set_SpeedMode(MI_Motor* motor, float speed, float current_limit) {
    // 设置速度指令
    Set_SingleParameter(motor, SPD_REF, speed);
    
    // 设置电流限制
    Set_SingleParameter(motor, LIMIT_CUR, current_limit);
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

    // 数据转换：使用线性映射
    uint16_t torque_cmd = float_to_uint16_linear(torque, T_MIN, T_MAX);       // 力矩 -17~17Nm
    uint16_t pos_cmd = float_to_uint16_linear(position, P_MIN, P_MAX);        // 角度 -12.57~12.57f
    uint16_t vel_cmd = float_to_uint16_linear(speed, V_MIN, V_MAX);           // 角速度 -44~44rad/s
    uint16_t kp_cmd = float_to_uint16_linear(kp, KP_MIN, KP_MAX);             // Kp 0.0~500.0
    uint16_t kd_cmd = float_to_uint16_linear(kd, KD_MIN, KD_MAX);             // Kd 0.0~5.0

    // 运控模式帧头构造
    frame.type = 0x01;                           // 运控模式类型
    frame.data = torque_cmd;                     // 力矩值直接放在data字段(16位)
    frame.target_id = motor->id;                 // 目标电机CAN_ID

    // 8字节数据区：目标角度 + 目标角速度 + Kp + Kd
    frame.payload[0] = (pos_cmd >> 8) & 0xFF;    // 目标角度高字节
    frame.payload[1] = pos_cmd & 0xFF;           // 目标角度低字节
    frame.payload[2] = (vel_cmd >> 8) & 0xFF;    // 目标角速度高字节
    frame.payload[3] = vel_cmd & 0xFF;           // 目标角速度低字节
    frame.payload[4] = (kp_cmd >> 8) & 0xFF;     // Kp高字节
    frame.payload[5] = kp_cmd & 0xFF;            // Kp低字节
    frame.payload[6] = (kd_cmd >> 8) & 0xFF;     // Kd高字节
    frame.payload[7] = kd_cmd & 0xFF;            // Kd低字节

    UART_Send_Frame(&frame);
}