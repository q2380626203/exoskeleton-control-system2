# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于ESP32-S3的外骨骼WiFi控制系统，集成了电机控制、WiFi热点、语音反馈和运动检测功能。该系统使用ESP-IDF框架，通过WiFi网页接口控制RS01电机。

## 构建和开发命令

### 基本ESP-IDF命令
- `idf.py build` - 构建项目
- `idf.py flash` - 烧录到设备
- `idf.py flash monitor` - 烧录并启动串口监控
- `idf.py monitor` - 启动串口监控查看日志
- `idf.py clean` - 清理构建文件
- `idf.py menuconfig` - 配置项目参数

### 设备配置
- 目标芯片：ESP32-S3
- 串口端口：/dev/ttyACM0（在.vscode/settings.json中配置）
- 波特率：115200

## 核心架构

### 主要模块结构

1. **main.c** - 系统主入口和任务管理
   - 统一的电机控制参数管理 (`motor_control_params_t`)
   - 串口数据解析任务 (`UART_Parse_Task`)
   - 运动模式检测任务 (`Motion_Detection_Task`)
   - 电机数据传输任务管理

2. **系统初始化模块** (`system_init.c/.h`)
   - NVS存储初始化
   - WiFi热点初始化
   - 外骨骼控制模块初始化

3. **电机控制模块**
   - `rs01_motor.c/.h` - RS01电机通信协议和UART接口
   - `motor_web_control.c/.h` - HTTP服务器和网页控制界面
   - `uart_motor_transmit.c/.h` - 电机数据传输任务

4. **运动控制模块**
   - `alternating_speed.c/.h` - 速度交替模式和行走模式切换
   - `continuous_torque_velocity_mode.c/.h` - 智能运动检测和参数化控制
   - `velocity_tracking_mode.c/.h` - 速度跟踪模式和交替工作机制
   - 环形缓存区位置监测和运动模式自动切换

5. **外设模块**
   - `wifi_softap_module.c/.h` - WiFi热点功能
   - `button_detector.c/.h` - 按键检测
   - `voice_module.c/.h` - 语音反馈

### 关键特性

- **智能运动检测**：基于位置数据自动识别静止/行走/爬楼模式
- **力矩渐变控制**：平滑的力矩变化避免突变
- **参数化电机控制**：统一的5参数控制接口(力矩/位置/速度/Kp/Kd)
- **条件发送机制**：只在参数变化时发送控制指令
- **WiFi网页控制**：通过浏览器实时控制和监控电机状态
- **速度跟踪模式**：基于速度方向变化的交替工作机制，支持抬腿MIT超时保护

### RS01电机通信

- 通信协议：CAN转串口模块
- UART端口：UART1 (TX: GPIO13, RX: GPIO12)
- 波特率：115200
- 支持两个电机 (ID: 1, 2)
- 参数范围：
  - 位置：-12.57 ~ 12.57 rad
  - 速度：-44 ~ 44 rad/s
  - 力矩：-17 ~ 17 Nm
  - Kp：0 ~ 500
  - Kd：0 ~ 5

### 任务管理

- `UART_Parse_Task` - 200Hz频率解析串口数据
- `Motion_Detection_Task` - 运动模式检测和力矩渐变控制
- `motor_data_transmit_task` - 定期发送电机数据
- `Alternating_Speed_Control_Task` - 速度交替控制（可选）
- `velocity_tracking_task` - 速度跟踪模式任务（可选）

## 关键设计模式

### 运动模式互斥机制
- alternating_speed模式与motion_detection模式互斥
- velocity_tracking模式独立运行，可与其他模式并存
- 网页界面提供冲突检测和状态显示

### 电机控制统一接口
- 所有模块通过 `unified_motor_control()` 函数控制电机
- 参数变化检测避免重复发送相同指令
- 支持实时参数调整（网页接口）

### 速度跟踪模式架构
- **交替工作机制**：1号工作2号休息 ↔ 2号工作1号休息
- **超时保护**：抬腿MIT超时1200ms后强制切换到600ms放腿MIT
- **自适应时间**：放腿MIT持续时间=检测间隔+200ms
- **网页可调参数**：抬腿/放腿的力矩和速度参数

## 开发注意事项

- 电机参数使用 `motor_control_params_t` 结构体统一管理
- 通过 `unified_motor_control()` 函数发送控制指令
- 位置监测使用环形缓存区 (`position_ring_buffer_t`)
- 运动模式状态通过 `motion_mode_state_t` 管理
- velocity_tracking模式参数可通过全局变量实时调整
- WiFi热点默认SSID可在 `wifi_softap_module.c` 中配置
- 所有日志使用ESP_LOG系列宏，TAG统一命名

## 调试和监控

- 使用 `idf.py monitor` 查看系统日志
- 通过WiFi连接访问网页控制界面
- 电机状态通过串口数据实时反馈
- 支持VS Code ESP-IDF插件开发环境