#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外骨骼WiFi控制系统框图生成器（最终版本）
使用英文标签避免字体问题，但保持中文注释和说明
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np

# 使用默认字体避免中文字体问题
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'DejaVu Sans'

def create_system_diagram():
    """创建系统框图"""

    # 创建图形和轴
    fig, ax = plt.subplots(1, 1, figsize=(18, 14))
    ax.set_xlim(0, 22)
    ax.set_ylim(0, 16)
    ax.set_aspect('equal')

    # 隐藏坐标轴
    ax.axis('off')

    # 定义颜色
    colors = {
        'core': '#4CAF50',      # 核心模块 - 绿色
        'control': '#2196F3',   # 控制模块 - 蓝色
        'comm': '#FF9800',      # 通信模块 - 橙色
        'sensor': '#9C27B0',    # 传感器 - 紫色
        'actuator': '#F44336',  # 执行器 - 红色
        'interface': '#607D8B'  # 接口 - 灰蓝色
    }

    # 绘制框和文本的函数
    def draw_box(x, y, width, height, text, color, text_color='white', fontsize=9):
        # 绘制圆角矩形
        box = FancyBboxPatch(
            (x, y), width, height,
            boxstyle="round,pad=0.05",
            facecolor=color,
            edgecolor='black',
            linewidth=1.5
        )
        ax.add_patch(box)

        # 处理换行符：将\\n替换为实际换行
        processed_text = text.replace('\\n', '\n')

        # 添加文本
        ax.text(x + width/2, y + height/2, processed_text,
                ha='center', va='center',
                fontsize=fontsize, color=text_color, weight='bold')

    # 绘制连接线的函数
    def draw_arrow(x1, y1, x2, y2, color='black', style='->', width=1.5):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                   arrowprops=dict(arrowstyle=style, color=color, lw=width))

    # 标题 - 使用英文避免字体问题
    ax.text(11, 15.2, 'Exoskeleton WiFi Control System Architecture', ha='center', va='center',
            fontsize=18, weight='bold')
    ax.text(11, 14.7, 'ESP32-S3 Based Exoskeleton Control System', ha='center', va='center',
            fontsize=14, style='italic')

    # ============ 核心处理单元 ============
    # ESP32-S3主控
    draw_box(9, 12.5, 4, 1.2, 'ESP32-S3\\nMain Controller', colors['core'], fontsize=11)

    # ============ 系统初始化层 ============
    draw_box(1, 10.8, 2.8, 1, 'System Init\\nModule', colors['core'], fontsize=9)
    draw_box(4.2, 10.8, 2.8, 1, 'NVS Storage\\nConfiguration', colors['core'], fontsize=9)

    # ============ 通信模块层 ============
    draw_box(14.5, 10.8, 2.8, 1, 'WiFi SoftAP\\nHotspot', colors['comm'], fontsize=9)
    draw_box(17.8, 10.8, 2.8, 1, 'HTTP Server\\nWeb Control', colors['comm'], fontsize=9)

    # ============ 运动控制层 ============
    draw_box(0.5, 8.8, 2.6, 1, 'Velocity\\nTracking', colors['control'], fontsize=9)
    draw_box(3.4, 8.8, 2.6, 1, 'Motion\\nDetection', colors['control'], fontsize=9)
    draw_box(6.3, 8.8, 2.6, 1, 'Alternating\\nSpeed', colors['control'], fontsize=9)
    draw_box(9.2, 8.8, 2.6, 1, 'Continuous\\nTorque', colors['control'], fontsize=9)
    draw_box(12.1, 8.8, 2.6, 1, 'Unified Motor\\nControl API', colors['control'], fontsize=9)

    # ============ UART通信层 ============
    draw_box(2.5, 6.8, 3.5, 1, 'UART1 (115200)\\nRS01 Motors', colors['interface'], fontsize=9)
    draw_box(15.5, 6.8, 3.5, 1, 'UART2 (9600)\\nVoice Module', colors['interface'], fontsize=9)

    # ============ 执行器层 ============
    draw_box(0.5, 4.8, 2.8, 1, 'RS01 Motor #1\\n(ID: 1)', colors['actuator'], fontsize=9)
    draw_box(3.8, 4.8, 2.8, 1, 'RS01 Motor #2\\n(ID: 2)', colors['actuator'], fontsize=9)

    # 语音模块
    draw_box(15, 4.8, 3.5, 1, 'Voice Module\\nChinese TTS', colors['sensor'], fontsize=9)

    # ============ 传感器/输入层 ============
    draw_box(7.5, 4.8, 2.8, 1, 'Button Input\\nGPIO 10/11', colors['sensor'], fontsize=9)
    draw_box(18.5, 4.8, 2.8, 1, 'Web Interface\\nBrowser', colors['interface'], fontsize=9)

    # ============ 系统参数显示区域 ============
    # 系统参数框 - 使用英文
    param_text = """System Parameters:
• Target MCU: ESP32-S3
• WiFi Mode: SoftAP Hotspot
• Motor Protocol: CAN-to-Serial
• Position Range: ±12.57 rad
• Velocity Range: ±44 rad/s
• Torque Range: ±17 Nm
• Kp Range: 0~500
• Kd Range: 0~5"""

    ax.text(0.5, 3.5, param_text, fontsize=8, va='top', ha='left',
           bbox=dict(boxstyle="round,pad=0.3", facecolor='lightgray', alpha=0.8))

    # 任务信息框
    task_text = """Main Tasks:
• UART_Parse_Task (20ms)
• Motion_Detection_Task
• buttonProcessTask
• velocity_tracking_task
• HTTP Server Tasks"""

    ax.text(11, 3.5, task_text, fontsize=8, va='top', ha='left',
           bbox=dict(boxstyle="round,pad=0.3", facecolor='lightblue', alpha=0.8))

    # 通信协议框
    proto_text = """Communication:
• UART1: TX-GPIO13, RX-GPIO12
• UART2: TX-GPIO1, RX-GPIO2
• Motor CAN-ID: 1, 2
• Voice: Chinese TTS
• Web Port: 80"""

    ax.text(16, 3.5, proto_text, fontsize=8, va='top', ha='left',
           bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow', alpha=0.8))

    # ============ 绘制连接线 ============
    # 主控到系统初始化
    draw_arrow(9.5, 12.5, 3, 11.5, colors['core'])
    draw_arrow(10.5, 12.5, 5.5, 11.5, colors['core'])

    # 主控到通信模块
    draw_arrow(12.5, 12.5, 15.8, 11.5, colors['comm'])
    draw_arrow(12.8, 12.5, 19, 11.5, colors['comm'])

    # 主控到运动控制层
    draw_arrow(9.2, 12.5, 1.8, 9.5, colors['control'])
    draw_arrow(9.6, 12.5, 4.7, 9.5, colors['control'])
    draw_arrow(10.4, 12.5, 7.6, 9.5, colors['control'])
    draw_arrow(10.8, 12.5, 10.5, 9.5, colors['control'])
    draw_arrow(11.2, 12.5, 13.4, 9.5, colors['control'])

    # 运动控制到UART接口
    draw_arrow(4.2, 8.8, 4.2, 7.8, colors['interface'])
    draw_arrow(13.4, 8.8, 17, 7.8, colors['interface'])

    # UART到执行器
    draw_arrow(3, 6.8, 1.9, 5.8, colors['actuator'])
    draw_arrow(5.5, 6.8, 5.2, 5.8, colors['actuator'])
    draw_arrow(17, 6.8, 16.8, 5.8, colors['sensor'])

    # 按键到主控
    draw_arrow(8.8, 5.8, 10, 12.5, colors['sensor'])

    # 网页到主控
    draw_arrow(19.5, 5.8, 12, 12.5, colors['interface'])

    # ============ 添加图例 ============
    legend_y = 0.5
    legend_items = [
        ('Core Modules', colors['core']),
        ('Control Modules', colors['control']),
        ('Communication', colors['comm']),
        ('Sensors/Input', colors['sensor']),
        ('Actuators', colors['actuator']),
        ('Interfaces', colors['interface'])
    ]

    ax.text(1, 1.2, 'Legend:', fontsize=12, weight='bold')

    for i, (label, color) in enumerate(legend_items):
        x_pos = 1.5 + (i % 3) * 5
        y_pos = legend_y - (i // 3) * 0.4

        # 绘制小方块
        rect = patches.Rectangle((x_pos, y_pos), 0.3, 0.2,
                               facecolor=color, edgecolor='black')
        ax.add_patch(rect)

        # 添加标签
        ax.text(x_pos + 0.4, y_pos + 0.1, label, fontsize=9, va='center')

    # 设置图形属性
    plt.tight_layout()

    return fig

def main():
    """主函数"""
    print("正在生成外骨骼WiFi控制系统架构图（无字体问题版本）...")

    # 创建系统框图
    fig = create_system_diagram()

    # 保存图片
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"Exoskeleton_System_Architecture_Final_{timestamp}.png"

    # 保存为高分辨率PNG
    plt.savefig(output_file, dpi=300, bbox_inches='tight',
               facecolor='white', edgecolor='none')

    print(f"系统架构图已生成: {output_file}")
    print("图片特点:")
    print("- 完整的系统架构概览")
    print("- 包含所有主要功能模块")
    print("- 显示模块间连接关系")
    print("- 标注关键技术参数")
    print("- 彩色编码不同模块类型")
    print("- 高分辨率PNG格式 (300 DPI)")
    print("- 英文标签，无字体显示问题")
    print("- 中文注释保留在代码中供参考")

    # 显示图片（如果在支持的环境中）
    # plt.show()

if __name__ == "__main__":
    main()