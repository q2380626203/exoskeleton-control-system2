#ifndef CONTINUOUS_TORQUE_VELOCITY_MODE_H
#define CONTINUOUS_TORQUE_VELOCITY_MODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 环形缓存区配置
#define POSITION_BUFFER_SIZE 25
#define POSITION_CHANGE_THRESHOLD 0.05f

// 运动模式检测配置
#define MODE_DETECTION_THRESHOLD 0.7f      // 爬楼模式升档阈值
#define CLIMBING_DOWNGRADE_THRESHOLD 0.5f  // 爬楼模式降档阈值
#define STATIC_DETECTION_THRESHOLD 0.2f    // 静止检测阈值 (调高避免力矩控制的震动)
#define STATIC_TIMEOUT_MS 3000              // 静止超时时间(3秒)

// 模式稳定性配置
#define MODE_LOCK_DURATION_MS 2000          // 模式锁定时间(2秒) - 切换后2秒内不允许再切换
#define STATIC_CONFIRM_DURATION_MS 1500     // 静止确认时间(1.5秒) - 需要持续1.5秒静止才确认

// 力矩渐变配置
#define TORQUE_INCREASE_INTERVAL_MS 500     // 力矩增加间隔(0.5秒)
#define TORQUE_DECREASE_INTERVAL_MS 200     // 力矩减少间隔(0.2秒) - 更快减少
#define TORQUE_INCREASE_STEP 0.5f           // 每次增加0.5N
#define TORQUE_DECREASE_STEP 1.5f           // 每次减少1.5N - 更大步长

// 运动模式枚举
typedef enum {
    MOTION_MODE_STATIC = 0,     // 静止模式
    MOTION_MODE_WALKING,        // 行走模式
    MOTION_MODE_CLIMBING        // 爬楼模式
} motion_mode_t;

// 电机参数结构体
typedef struct {
    int velocity;               // 速度
    float torque;               // 力矩（改为float支持小数）
    int kd;                     // kd参数
} motor_params_t;

// 力矩渐变控制结构体
typedef struct {
    float current_torque;       // 当前力矩值
    float target_torque;        // 目标力矩值
    uint32_t last_update_time;  // 上次更新时间戳
    bool is_increasing;         // 是否正在增加
} torque_gradient_t;

// 运动模式状态管理结构体
typedef struct {
    motion_mode_t current_mode;         // 当前运动模式
    motion_mode_t previous_mode;        // 上一次运动模式
    uint32_t mode_start_timestamp;      // 模式开始时间戳
    bool is_continuous_mode;            // 是否为持续模式
    torque_gradient_t motor1_torque;    // 电机1力矩渐变控制
    torque_gradient_t motor2_torque;    // 电机2力矩渐变控制

    // 基于位置的模式稳定性控制
    uint32_t last_mode_change_time;     // 上次模式切换时间
    uint32_t static_confirm_start_time; // 开始静止确认的时间
    bool in_static_confirmation;        // 是否正在进行静止确认
    float last_position_range;          // 上次的位置变化范围

    // 爬楼模式降档控制
    int climbing_downgrade_count;       // 连续低于阈值的次数
} motion_mode_state_t;

// 位置点结构体
typedef struct {
    float position;          // 位置值
    uint32_t timestamp;      // 时间戳 (ms)
    bool valid;              // 数据有效标志
} position_point_t;

// 环形缓存区结构体
typedef struct {
    position_point_t buffer[POSITION_BUFFER_SIZE];  // 缓存数组
    uint16_t head;                                  // 写入指针
    uint16_t tail;                                  // 读取指针
    uint16_t count;                                 // 当前元素数量
    float last_recorded_position;                   // 上次记录的位置
    bool initialized;                               // 初始化标志
} position_ring_buffer_t;

// 函数声明

/**
 * @brief 初始化环形缓存区
 * @param buffer 缓存区指针
 */
void position_ring_buffer_init(position_ring_buffer_t *buffer);

/**
 * @brief 检查位置变化并添加到缓存区
 * @param buffer 缓存区指针
 * @param new_position 新位置值
 * @param timestamp 时间戳
 * @return true 如果位置被记录，false 如果变化小于阈值
 */
bool position_ring_buffer_add_if_changed(position_ring_buffer_t *buffer,
                                        float new_position,
                                        uint32_t timestamp);

/**
 * @brief 强制添加位置点到缓存区
 * @param buffer 缓存区指针
 * @param position 位置值
 * @param timestamp 时间戳
 */
void position_ring_buffer_add(position_ring_buffer_t *buffer,
                             float position,
                             uint32_t timestamp);

/**
 * @brief 从缓存区读取最新的位置点
 * @param buffer 缓存区指针
 * @param point 输出的位置点
 * @return true 如果成功读取，false 如果缓存区为空
 */
bool position_ring_buffer_get_latest(position_ring_buffer_t *buffer,
                                    position_point_t *point);

/**
 * @brief 从缓存区读取指定索引的位置点
 * @param buffer 缓存区指针
 * @param index 索引 (0为最新，负数表示从最新往前数)
 * @param point 输出的位置点
 * @return true 如果成功读取，false 如果索引无效
 */
bool position_ring_buffer_get_at_index(position_ring_buffer_t *buffer,
                                      int16_t index,
                                      position_point_t *point);

/**
 * @brief 获取缓存区中的元素数量
 * @param buffer 缓存区指针
 * @return 元素数量
 */
uint16_t position_ring_buffer_get_count(position_ring_buffer_t *buffer);

/**
 * @brief 检查缓存区是否为空
 * @param buffer 缓存区指针
 * @return true 如果为空，false 如果有数据
 */
bool position_ring_buffer_is_empty(position_ring_buffer_t *buffer);

/**
 * @brief 检查缓存区是否已满
 * @param buffer 缓存区指针
 * @return true 如果已满，false 如果未满
 */
bool position_ring_buffer_is_full(position_ring_buffer_t *buffer);

/**
 * @brief 清空缓存区
 * @param buffer 缓存区指针
 */
void position_ring_buffer_clear(position_ring_buffer_t *buffer);

/**
 * @brief 获取环形缓冲区中的最大值和最小值
 * @param buffer 缓存区指针
 * @param min_pos 输出最小位置值
 * @param max_pos 输出最大位置值
 * @return true 如果成功获取，false 如果缓存区为空
 */
bool position_ring_buffer_get_min_max(position_ring_buffer_t *buffer,
                                      float *min_pos, float *max_pos);

/**
 * @brief 检测运动模式
 * @param buffer 缓存区指针
 * @param current_timestamp 当前时间戳
 * @param state 运动模式状态管理结构体指针
 * @return 检测到的运动模式
 */
motion_mode_t detect_motion_mode(position_ring_buffer_t *buffer,
                                uint32_t current_timestamp,
                                motion_mode_state_t *state);

/**
 * @brief 检测波峰波谷差值
 * @param buffer 缓存区指针
 * @return 波峰波谷差值，失败返回-1
 */
float detect_peak_valley_difference(position_ring_buffer_t *buffer);

/**
 * @brief 获取指定运动模式下的电机参数
 * @param mode 运动模式
 * @param motor_id 电机ID (1或2)
 * @param params 输出电机参数
 */
void get_motor_params(motion_mode_t mode, int motor_id, motor_params_t *params);

/**
 * @brief 初始化运动模式状态管理
 * @param state 状态管理结构体指针
 */
void motion_mode_state_init(motion_mode_state_t *state);

/**
 * @brief 更新运动模式状态并获取电机参数（考虑持续模式）
 * @param state 状态管理结构体指针
 * @param new_mode 新检测到的运动模式
 * @param current_timestamp 当前时间戳
 * @param motor_id 电机ID (1或2)
 * @param params 输出电机参数
 */
void update_motion_mode_and_get_params(motion_mode_state_t *state,
                                      motion_mode_t new_mode,
                                      uint32_t current_timestamp,
                                      int motor_id,
                                      motor_params_t *params);

/**
 * @brief 初始化力矩渐变控制
 * @param gradient 力矩渐变控制结构体指针
 */
void torque_gradient_init(torque_gradient_t *gradient);

/**
 * @brief 更新力矩渐变值
 * @param gradient 力矩渐变控制结构体指针
 * @param target_torque 目标力矩值
 * @param current_timestamp 当前时间戳
 * @return 当前实际力矩值
 */
float update_torque_gradient(torque_gradient_t *gradient,
                             float target_torque,
                             uint32_t current_timestamp);

#ifdef __cplusplus
}
#endif

#endif // CONTINUOUS_TORQUE_VELOCITY_MODE_H