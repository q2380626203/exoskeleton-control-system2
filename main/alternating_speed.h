/**
 * @file alternating_speed.h
 * @brief 速度交替模式控制头文件
 *
 * 提供外骨骼速度交替控制功能，包括：
 * - 交替速度控制
 * - 行走模式切换（静止/过渡/平地/爬楼）
 * - 位置监测和自动模式切换
 */

#ifndef ALTERNATING_SPEED_H
#define ALTERNATING_SPEED_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 行走模式枚举
typedef enum {
    MODE_IDLE = 0,         // 静止模式
    MODE_TRANSITION = 1,   // 过渡模式
    MODE_FLAT_GROUND = 2,  // 平地模式（行走模式）
    MODE_STAIRS = 3        // 爬楼模式
} walking_mode_t;

// 运动状态枚举
typedef enum {
    MOTION_STATE_IDLE = 0,       // 静止状态
    MOTION_STATE_TRANSITION = 1, // 过渡状态
    MOTION_STATE_WALKING = 2,    // 行走状态
    MOTION_STATE_STAIRS = 3      // 爬楼状态
} motion_state_t;

// 交替速度控制变量
extern bool alternating_speed_enabled;
extern int alternating_interval_ms;
extern float alternating_speed_x;
extern float alternating_speed_y;
extern float speed_current_limit;

// 速度控制函数
void Start_Alternating_Speed(void);
void Stop_Alternating_Speed(void);

// 行走模式切换函数
void Switch_Walking_Mode(walking_mode_t mode);
walking_mode_t Get_Current_Walking_Mode(void);

// 快捷模式切换函数
void Switch_To_Idle_Mode(void);
void Switch_To_Transition_Mode(void);
void Switch_To_Flat_Mode(void);
void Switch_To_Stairs_Mode(void);

// 保持步态周期的模式切换函数
void Switch_To_Idle_Mode_Keep_Phase(void);
void Switch_To_Transition_Mode_Keep_Phase(void);
void Switch_To_Flat_Mode_Keep_Phase(void);
void Switch_To_Stairs_Mode_Keep_Phase(void);

// 运动状态查询函数
motion_state_t Get_Current_Motion_State(void);

// 位置监测函数
void Start_Position_Monitor_Task(void);
void Stop_Position_Monitor_Task(void);
bool Get_Position_Variation_Info(float* motor1_diff, float* motor2_diff);

// 位置数据添加函数（供外部调用）
void Add_Position_Sample(float positions[2]);

// 外部函数声明 - 来自main.c的统一电机控制函数
extern void set_motor_params(int motor_id, float torque, float position, float speed, float kp, float kd);

// 交替速度控制任务函数
void Alternating_Speed_Control_Task(void* pvParameters);

// 普通电机控制任务函数
void Motor_Speed_Control_Task(void* pvParameters);
void Start_Motor_Speed_Control(void);
void Stop_Motor_Speed_Control(void);

#ifdef __cplusplus
}
#endif

#endif // ALTERNATING_SPEED_H