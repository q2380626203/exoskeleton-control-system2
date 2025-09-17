/**
 * @file alternating_speed.c
 * @brief 速度交替模式控制实现
 *
 * 提供外骨骼速度交替控制功能实现
 */

#include "alternating_speed.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "rs01_motor.h"

static const char *TAG = "alternating_speed";

// 交替速度控制变量
bool alternating_speed_enabled = false;
static int speed_state = 0;  // 0: 正向，1: 反向
static TickType_t last_switch_time = 0;

// 交替速度配置变量（默认平地模式）
int alternating_interval_ms = 800;  // 默认800ms间隔（平地模式）
float alternating_speed_x = 2.25f;  // 交替速度值X (rad/s) - 抬腿速度提升至1.5倍
float alternating_speed_y = 1.5f;   // 交替速度值Y (rad/s)
float speed_current_limit = 0.0f;   // 速度模式下的电流限制 (平地模式:5A, 爬楼模式:10A)

// 模式参数常量定义
// 静止模式参数
#define IDLE_INTERVAL_MS 1000
#define IDLE_SPEED_X 0.0f
#define IDLE_SPEED_Y 0.0f
#define IDLE_CURRENT_LIMIT 0.0f

// 过渡模式参数
#define TRANSITION_INTERVAL_MS 800
#define TRANSITION_SPEED_X 2.25f
#define TRANSITION_SPEED_Y 1.0f
#define TRANSITION_CURRENT_LIMIT 1.0f

// 平地模式参数（行走模式）
#define FLAT_INTERVAL_MS 800
#define FLAT_SPEED_X 2.25f
#define FLAT_SPEED_Y 1.5f
#define FLAT_CURRENT_LIMIT 2.0f

// 爬楼模式参数
#define STAIRS_INTERVAL_MS 1000
#define STAIRS_SPEED_X 3.0f
#define STAIRS_SPEED_Y 2.25f
#define STAIRS_CURRENT_LIMIT 2.0f

// 前馈力矩参数
float feedforward_torque_value = 0.0f;  // 前馈力矩大小 (Nm)

// 当前模式
static walking_mode_t current_walking_mode = MODE_IDLE;

// 位置监测任务相关变量
static TaskHandle_t position_monitor_task_handle = NULL;
bool position_monitor_enabled = false;

// 位置数据缓冲区（2秒内的数据点，100Hz采样）
#define POSITION_BUFFER_SIZE 200
#define MONITOR_SAMPLING_PERIOD_MS 10  // 10ms采样间隔，100Hz
#define ANALYSIS_WINDOW_MS 2000        // 2秒分析窗口

typedef struct {
    float position[2];  // 两个电机的位置
    TickType_t timestamp;
} position_sample_t;

static position_sample_t position_buffer[POSITION_BUFFER_SIZE];
static int buffer_write_index = 0;
static bool buffer_full = false;

// 模式切换阈值
#define THRESHOLD_START_TRANSITION 0.75f // 从静止到过渡模式
#define THRESHOLD_START_WALKING 1.0f     // 从过渡到行走，原来从静止到行走
#define THRESHOLD_STAIRS_UP 1.25f        // 从行走到爬楼
#define THRESHOLD_STAIRS_DOWN 1.3f       // 从爬楼到行走
#define THRESHOLD_WALKING_DOWN 0.7f      // 从行走降到过渡模式
#define THRESHOLD_TRANSITION_DOWN 0.40f  // 从过渡降到静止模式

// 当前状态
static motion_state_t current_motion_state = MOTION_STATE_IDLE;

// 模式切换冷却机制
static TickType_t last_mode_switch_time = 0;
#define MODE_SWITCH_COOLDOWN_MS 2000  // 2秒冷却时间

// 外部变量声明
// 统一的电机控制参数结构体
typedef struct {
    float torque;      // 力矩设定 (Nm)
    float position;    // 位置设定 (rad)
    float speed;       // 速度设定 (rad/s)
    float kp;          // Kp参数
    float kd;          // Kd参数
} motor_control_params_t;

extern motor_control_params_t motor_params[2];
extern MI_Motor motors[2];            // 电机数组

// 外部函数声明
extern void unified_motor_control(int motor_id, const motor_control_params_t* params);



// 内部变量
static TaskHandle_t alternating_speed_task_handle = NULL;

/**
 * @brief 将新的位置数据添加到缓冲区
 * @param positions 两个电机的位置数组
 */
void Add_Position_Sample(float positions[2]) {
    TickType_t current_time = xTaskGetTickCount();

    position_buffer[buffer_write_index].position[0] = positions[0];
    position_buffer[buffer_write_index].position[1] = positions[1];
    position_buffer[buffer_write_index].timestamp = current_time;

    buffer_write_index = (buffer_write_index + 1) % POSITION_BUFFER_SIZE;

    if (buffer_write_index == 0) {
        buffer_full = true;
    }
}

/**
 * @brief 分析2秒内的位置数据，计算波峰波谷差值
 * @param motor_id 电机ID (0或1)
 * @param peak_valley_diff 输出的波峰波谷差值
 * @return true 如果有足够数据进行分析，false 否则
 */
bool Analyze_Position_Variation(int motor_id, float* peak_valley_diff) {
    if (motor_id < 0 || motor_id >= 2) return false;

    TickType_t current_time = xTaskGetTickCount();
    TickType_t analysis_start_time = current_time - pdMS_TO_TICKS(ANALYSIS_WINDOW_MS);

    float max_position = -999.0f;
    float min_position = 999.0f;
    int valid_samples = 0;

    // 遍历缓冲区中的有效数据
    int samples_to_check = buffer_full ? POSITION_BUFFER_SIZE : buffer_write_index;

    for (int i = 0; i < samples_to_check; i++) {
        if (position_buffer[i].timestamp >= analysis_start_time) {
            float pos = position_buffer[i].position[motor_id];

            if (pos > max_position) max_position = pos;
            if (pos < min_position) min_position = pos;
            valid_samples++;
        }
    }

    // 需要至少50个样本（0.5秒数据）才进行分析
    if (valid_samples < 50) {
        return false;
    }

    *peak_valley_diff = max_position - min_position;
    return true;
}

/**
 * @brief 基于位置变化分析的模式自动切换逻辑（带冷却时间）
 */
void Auto_Switch_Mode_Based_On_Position(void) {
    if (!alternating_speed_enabled) {
        return; // 只在交替模式开启时才进行自动切换
    }

    // 检查冷却时间（只对高强度模式间的切换进行冷却）
    TickType_t current_time = xTaskGetTickCount();
    bool need_cooldown = (current_motion_state == MOTION_STATE_WALKING ||
                         current_motion_state == MOTION_STATE_STAIRS);

    if (need_cooldown &&
        current_time - last_mode_switch_time < pdMS_TO_TICKS(MODE_SWITCH_COOLDOWN_MS)) {
        return; // 仍在冷却期内，不进行切换
    }

    float motor1_diff, motor2_diff;
    bool motor1_valid = Analyze_Position_Variation(0, &motor1_diff);
    bool motor2_valid = Analyze_Position_Variation(1, &motor2_diff);

    if (!motor1_valid || !motor2_valid) {
        return; // 数据不足，无法分析
    }

    // 取两个电机位置变化的最大值作为判断依据
    float max_diff = fmaxf(motor1_diff, motor2_diff);

    motion_state_t new_state = current_motion_state;

    // 状态机切换逻辑
    switch (current_motion_state) {
        case MOTION_STATE_IDLE:
            if (max_diff > THRESHOLD_START_WALKING) {
                new_state = MOTION_STATE_WALKING;
                ESP_LOGI(TAG, "模式切换: 静止 -> 行走 (差值=%.3f)", max_diff);
            } else if (max_diff > THRESHOLD_START_TRANSITION) {
                new_state = MOTION_STATE_TRANSITION;
                ESP_LOGI(TAG, "模式切换: 静止 -> 过渡 (差值=%.3f)", max_diff);
            }
            break;

        case MOTION_STATE_TRANSITION:
            if (max_diff > THRESHOLD_START_WALKING) {
                new_state = MOTION_STATE_WALKING;
                ESP_LOGI(TAG, "模式切换: 过渡 -> 行走 (差值=%.3f)", max_diff);
            } else if (max_diff < THRESHOLD_TRANSITION_DOWN) {
                new_state = MOTION_STATE_IDLE;
                ESP_LOGI(TAG, "模式切换: 过渡 -> 静止 (差值=%.3f)", max_diff);
            }
            break;

        case MOTION_STATE_WALKING:
            if (max_diff > THRESHOLD_STAIRS_UP) {
                new_state = MOTION_STATE_STAIRS;
                ESP_LOGI(TAG, "模式切换: 行走 -> 爬楼 (差值=%.3f)", max_diff);
            } else if (max_diff < THRESHOLD_WALKING_DOWN) {
                new_state = MOTION_STATE_TRANSITION;
                ESP_LOGI(TAG, "模式切换: 行走 -> 过渡 (差值=%.3f)", max_diff);
            }
            break;

        case MOTION_STATE_STAIRS:
            if (max_diff < THRESHOLD_STAIRS_DOWN) {
                new_state = MOTION_STATE_WALKING;
                ESP_LOGI(TAG, "模式切换: 爬楼 -> 行走 (差值=%.3f)", max_diff);
            }
            break;
    }

    // 如果状态发生变化，执行模式切换（保持当前步态周期）
    if (new_state != current_motion_state) {
        motion_state_t old_state = current_motion_state;
        current_motion_state = new_state;

        // 只在高强度模式间切换时记录冷却时间
        bool is_high_intensity_switch = (old_state == MOTION_STATE_WALKING || old_state == MOTION_STATE_STAIRS) &&
                                       (new_state == MOTION_STATE_WALKING || new_state == MOTION_STATE_STAIRS);

        if (is_high_intensity_switch) {
            last_mode_switch_time = current_time; // 记录切换时间用于冷却
        }

        // 根据新状态切换行走模式（不重置步态周期）
        switch (new_state) {
            case MOTION_STATE_IDLE:
                Switch_To_Idle_Mode_Keep_Phase();
                break;
            case MOTION_STATE_TRANSITION:
                Switch_To_Transition_Mode_Keep_Phase();
                break;
            case MOTION_STATE_WALKING:
                Switch_To_Flat_Mode_Keep_Phase();
                break;
            case MOTION_STATE_STAIRS:
                Switch_To_Stairs_Mode_Keep_Phase();
                break;
        }

        const char* state_names[] = {"静止", "过渡", "行走", "爬楼"};
        const char* cooldown_info = is_high_intensity_switch ? "（应用2秒冷却）" : "（无冷却限制）";

        ESP_LOGI(TAG, "当前运动状态: %s%s, 位置变化差值: 电机1=%.3f, 电机2=%.3f, 保持步态周期: %d",
                 state_names[new_state], cooldown_info, motor1_diff, motor2_diff, speed_state);
    }
}

/**
 * @brief 位置监测任务 - 监测电机位置变化并自动切换模式
 * @param pvParameters 任务参数（未使用）
 */
void Position_Monitor_Task(void* pvParameters) {
    ESP_LOGI(TAG, "位置监测任务启动");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(MONITOR_SAMPLING_PERIOD_MS); // 10ms采样周期

    TickType_t last_analysis_time = 0;
    const TickType_t analysis_interval = pdMS_TO_TICKS(500); // 每500ms分析一次

    while (position_monitor_enabled) {
        TickType_t current_time = xTaskGetTickCount();

        // 采集当前电机位置数据
        float current_positions[2] = {motors[0].position, motors[1].position};
        Add_Position_Sample(current_positions);

        // 定期进行模式分析（每500ms一次）
        if (current_time - last_analysis_time >= analysis_interval) {
            Auto_Switch_Mode_Based_On_Position();
            last_analysis_time = current_time;
        }

        // 等待下个采样周期
        vTaskDelayUntil(&last_wake_time, task_period);
    }

    ESP_LOGI(TAG, "位置监测任务退出");
    position_monitor_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 启动位置监测任务
 */
void Start_Position_Monitor_Task(void) {
    if (position_monitor_task_handle == NULL) {
        position_monitor_enabled = true;

        // 初始化缓冲区
        buffer_write_index = 0;
        buffer_full = false;
        current_motion_state = MOTION_STATE_IDLE;

        BaseType_t result = xTaskCreate(
            Position_Monitor_Task,
            "PositionMonitor",
            3072,  // 栈大小
            NULL,
            6,     // 优先级（高于一般任务，低于电机控制）
            &position_monitor_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "位置监测任务创建成功");
        } else {
            ESP_LOGE(TAG, "位置监测任务创建失败");
            position_monitor_enabled = false;
        }
    } else {
        ESP_LOGW(TAG, "位置监测任务已在运行");
    }
}

/**
 * @brief 停止位置监测任务
 */
void Stop_Position_Monitor_Task(void) {
    if (position_monitor_task_handle != NULL) {
        position_monitor_enabled = false;
        ESP_LOGI(TAG, "请求停止位置监测任务");
        // 任务会自然退出并清理自己
    }
}


/**
 * @brief 启动交替速度模式
 */
void Start_Alternating_Speed(void) {
    alternating_speed_enabled = true;
    speed_state = 0;
    last_switch_time = xTaskGetTickCount();

    // 启动位置监测任务（如果尚未启动）
    if (position_monitor_task_handle == NULL) {
        Start_Position_Monitor_Task();
        ESP_LOGI(TAG, "位置监测任务已启动，将监控电机位置变化进行自动模式切换");
    }

    // 启动交替速度控制任务
    if (alternating_speed_task_handle == NULL) {
        BaseType_t result = xTaskCreate(
            Alternating_Speed_Control_Task,
            "AltSpeedCtrl",
            4096,
            NULL,
            7,  // 提高电机控制优先级，确保实时性
            &alternating_speed_task_handle
        );

        if (result == pdPASS) {
            ESP_LOGI(TAG, "交替速度控制任务创建成功");
        } else {
            ESP_LOGE(TAG, "交替速度控制任务创建失败");
            alternating_speed_enabled = false;
        }
    }

    ESP_LOGI(TAG, "交替速度模式已启动，每%.1f秒切换", alternating_interval_ms / 1000.0f);
}

/**
 * @brief 停止交替速度模式
 */
void Stop_Alternating_Speed(void) {
    alternating_speed_enabled = false;

    // 停止交替速度控制任务
    if (alternating_speed_task_handle != NULL) {
        ESP_LOGI(TAG, "请求停止交替速度控制任务");
        // 任务会自然退出并清理自己
    }

    ESP_LOGI(TAG, "交替速度模式已停止");
    ESP_LOGI(TAG, "注意: 位置监测任务仍在运行，如需停止请调用Stop_Position_Monitor_Task()");
}

/**
 * @brief 切换行走模式
 * @param mode 目标模式：0=静止模式，1=过渡模式，2=平地模式，3=爬楼模式
 */
void Switch_Walking_Mode(walking_mode_t mode) {
    if (mode != current_walking_mode) {
        current_walking_mode = mode;

        // 根据模式更新参数
        if (mode == MODE_IDLE) {
            alternating_interval_ms = IDLE_INTERVAL_MS;
            alternating_speed_x = IDLE_SPEED_X;
            alternating_speed_y = IDLE_SPEED_Y;
            speed_current_limit = IDLE_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到静止模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        } else if (mode == MODE_TRANSITION) {
            alternating_interval_ms = TRANSITION_INTERVAL_MS;
            alternating_speed_x = TRANSITION_SPEED_X;
            alternating_speed_y = TRANSITION_SPEED_Y;
            speed_current_limit = TRANSITION_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到过渡模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        } else if (mode == MODE_FLAT_GROUND) {
            alternating_interval_ms = FLAT_INTERVAL_MS;
            alternating_speed_x = FLAT_SPEED_X;
            alternating_speed_y = FLAT_SPEED_Y;
            speed_current_limit = FLAT_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到平地模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        } else if (mode == MODE_STAIRS) {
            alternating_interval_ms = STAIRS_INTERVAL_MS;
            alternating_speed_x = STAIRS_SPEED_X;
            alternating_speed_y = STAIRS_SPEED_Y;
            speed_current_limit = STAIRS_CURRENT_LIMIT;
            ESP_LOGI(TAG, "切换到爬楼模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, Kd=%.1f",
                     alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_current_limit);
        }

        // 如果交替速度模式正在运行，重置切换时间
        if (alternating_speed_enabled) {
            speed_state = 0;
            last_switch_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "重置交替速度状态");
        }
    } else {
        ESP_LOGW(TAG, "已经处于所选模式，无需切换");
    }
}

/**
 * @brief 获取当前行走模式
 * @return 当前模式：0=静止模式，1=过渡模式，2=平地模式，3=爬楼模式
 */
walking_mode_t Get_Current_Walking_Mode(void) {
    return current_walking_mode;
}

/**
 * @brief 切换到静止模式
 */
void Switch_To_Idle_Mode(void) {
    Switch_Walking_Mode(MODE_IDLE);
}

/**
 * @brief 切换到过渡模式
 */
void Switch_To_Transition_Mode(void) {
    Switch_Walking_Mode(MODE_TRANSITION);
}

/**
 * @brief 切换到平地模式
 */
void Switch_To_Flat_Mode(void) {
    Switch_Walking_Mode(MODE_FLAT_GROUND);
}

/**
 * @brief 切换到爬楼模式
 */
void Switch_To_Stairs_Mode(void) {
    Switch_Walking_Mode(MODE_STAIRS);
}

/**
 * @brief 获取当前运动状态
 * @return 当前运动状态：0=静止，1=过渡，2=行走，3=爬楼
 */
motion_state_t Get_Current_Motion_State(void) {
    return current_motion_state;
}

/**
 * @brief 获取当前位置变化信息（用于调试）
 * @param motor1_diff 输出电机1的位置变化差值
 * @param motor2_diff 输出电机2的位置变化差值
 * @return true 如果有有效数据，false 否则
 */
bool Get_Position_Variation_Info(float* motor1_diff, float* motor2_diff) {
    bool motor1_valid = Analyze_Position_Variation(0, motor1_diff);
    bool motor2_valid = Analyze_Position_Variation(1, motor2_diff);
    return motor1_valid && motor2_valid;
}

/**
 * @brief 切换到静止模式（保持当前步态周期）
 */
void Switch_To_Idle_Mode_Keep_Phase(void) {
    if (current_walking_mode != MODE_IDLE) {
        // 只更新模式参数，不重置步态状态
        current_walking_mode = MODE_IDLE;
        alternating_interval_ms = IDLE_INTERVAL_MS;
        alternating_speed_x = IDLE_SPEED_X;
        alternating_speed_y = IDLE_SPEED_Y;
        speed_current_limit = IDLE_CURRENT_LIMIT;

        ESP_LOGI(TAG, "平滑切换到静止模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 保持当前步态周期=%d",
                 alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_state);

        // 不重置last_switch_time，保持当前的步态节奏
        // 不重置speed_state，保持当前的周期状态
    } else {
        ESP_LOGD(TAG, "已经处于静止模式，无需切换");
    }
}

/**
 * @brief 切换到过渡模式（保持当前步态周期）
 */
void Switch_To_Transition_Mode_Keep_Phase(void) {
    if (current_walking_mode != MODE_TRANSITION) {
        // 只更新模式参数，不重置步态状态
        current_walking_mode = MODE_TRANSITION;
        alternating_interval_ms = TRANSITION_INTERVAL_MS;
        alternating_speed_x = TRANSITION_SPEED_X;
        alternating_speed_y = TRANSITION_SPEED_Y;
        speed_current_limit = TRANSITION_CURRENT_LIMIT;

        ESP_LOGI(TAG, "平滑切换到过渡模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 保持当前步态周期=%d",
                 alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_state);

        // 不重置last_switch_time，保持当前的步态节奏
        // 不重置speed_state，保持当前的周期状态
    } else {
        ESP_LOGD(TAG, "已经处于过渡模式，无需切换");
    }
}

/**
 * @brief 切换到平地模式（保持当前步态周期）
 */
void Switch_To_Flat_Mode_Keep_Phase(void) {
    if (current_walking_mode != MODE_FLAT_GROUND) {
        // 只更新模式参数，不重置步态状态
        current_walking_mode = MODE_FLAT_GROUND;
        alternating_interval_ms = FLAT_INTERVAL_MS;
        alternating_speed_x = FLAT_SPEED_X;
        alternating_speed_y = FLAT_SPEED_Y;
        speed_current_limit = FLAT_CURRENT_LIMIT;

        ESP_LOGI(TAG, "平滑切换到平地模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 保持当前步态周期=%d",
                 alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_state);

        // 不重置last_switch_time，保持当前的步态节奏
        // 不重置speed_state，保持当前的周期状态
    } else {
        ESP_LOGD(TAG, "已经处于平地模式，无需切换");
    }
}

/**
 * @brief 切换到爬楼模式（保持当前步态周期）
 */
void Switch_To_Stairs_Mode_Keep_Phase(void) {
    if (current_walking_mode != MODE_STAIRS) {
        // 只更新模式参数，不重置步态状态
        current_walking_mode = MODE_STAIRS;
        alternating_interval_ms = STAIRS_INTERVAL_MS;
        alternating_speed_x = STAIRS_SPEED_X;
        alternating_speed_y = STAIRS_SPEED_Y;
        speed_current_limit = STAIRS_CURRENT_LIMIT;

        ESP_LOGI(TAG, "平滑切换到爬楼模式: 间隔=%dms, 速度X=%.1f, 速度Y=%.1f, 保持当前步态周期=%d",
                 alternating_interval_ms, alternating_speed_x, alternating_speed_y, speed_state);

        // 不重置last_switch_time，保持当前的步态节奏
        // 不重置speed_state，保持当前的周期状态
    } else {
        ESP_LOGD(TAG, "已经处于爬楼模式，无需切换");
    }
}

/**
 * @brief 交替速度控制任务
 * @param pvParameters 任务参数（未使用）
 */
void Alternating_Speed_Control_Task(void* pvParameters) {
    ESP_LOGI(TAG, "交替速度控制任务启动");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(20);
    last_switch_time = last_wake_time;


    while (alternating_speed_enabled) {
        TickType_t current_time = xTaskGetTickCount();

        // 检查是否需要切换速度方向（使用配置的间隔时间）
        if (alternating_speed_enabled &&
            (current_time - last_switch_time) >= pdMS_TO_TICKS(alternating_interval_ms)) {
            speed_state = 1 - speed_state; // 0变1，1变0
            last_switch_time = current_time;
            ESP_LOGI(TAG, "速度切换到状态: %d", speed_state);
        }

        // 计算目标速度
        float target_speeds[2];

        for (int i = 0; i < 2; i++) {
            // 交替速度模式：两电机交替使用对方的速度值，1号负向，2号正向
            if (speed_state == 0) {
                // 周期1：1号-X rad/s，2号-Y rad/s
                target_speeds[i] = (i == 0) ? -alternating_speed_x : -alternating_speed_y;
            } else {
                // 周期2：1号+Y rad/s，2号+X rad/s
                target_speeds[i] = (i == 0) ? alternating_speed_y : alternating_speed_x;
            }

            // 限制速度范围到安全值 (-44 ~ 44 rad/s)
            if (target_speeds[i] > 44.0f) target_speeds[i] = 44.0f;
            else if (target_speeds[i] < -44.0f) target_speeds[i] = -44.0f;
        }

        // 计算前馈力矩和动态Kd值（基于当前模式和步态周期）
        float feedforward_torque[2] = {0.0f, 0.0f};
        float dynamic_kd[2];

        // 根据当前模式设置Kd值（使用对应模式的current_limit作为Kd值）
        dynamic_kd[0] = speed_current_limit;
        dynamic_kd[1] = speed_current_limit;

        if (alternating_speed_enabled &&
            (current_walking_mode == MODE_FLAT_GROUND || current_walking_mode == MODE_STAIRS)) {
            // 只在行走和爬楼模式下提供前馈力矩
            for (int i = 0; i < 2; i++) {
                if (speed_state == 0) {
                    // 周期1：电机1提供负前馈力矩，电机2提供负前馈力矩
                    feedforward_torque[i] = -feedforward_torque_value;
                } else {
                    // 周期2：电机1提供正前馈力矩，电机2提供正前馈力矩
                    feedforward_torque[i] = feedforward_torque_value;
                }
            }
        }
        // 过渡和静止模式下前馈力矩保持为0，但Kd值仍为对应模式的current_limit

        // 直接更新统一参数结构体，让unified_motor_control来处理变化检测
        for (int i = 0; i < 2; i++) {
            // 交替速度模式：力矩只使用前馈力矩（基础力矩为0）
            float base_torque = 0.0f;  // 交替速度模式基础力矩为0
            float total_torque = base_torque + feedforward_torque[i];

            // 更新统一结构体参数
            motor_params[i].torque = total_torque;
            motor_params[i].position = 0.0f; // 交替速度模式位置为0
            motor_params[i].speed = target_speeds[i];
            motor_params[i].kp = 0.0f; // 交替速度模式kp为0
            motor_params[i].kd = dynamic_kd[i];

            // 通过统一控制函数执行（内部会自动检查参数变化）
            unified_motor_control(i, &motor_params[i]);

            ESP_LOGI(TAG, "电机%d参数更新: 总力矩=%.2f(控制力矩+前馈=%.2f), 位置=%.2f, 速度=%.2f, Kp=%.2f, Kd=%.2f",
                     i+1, total_torque, feedforward_torque[i], 0.0f, target_speeds[i], 0.0f, dynamic_kd[i]);

            // 发送间隔
            if (i == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }


        // 等待剩余的时间完成20ms周期
        vTaskDelayUntil(&last_wake_time, task_period);
    }

    ESP_LOGI(TAG, "交替速度控制任务退出");
    alternating_speed_task_handle = NULL;
    vTaskDelete(NULL);
}

