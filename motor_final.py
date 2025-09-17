#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机数据监控 - 最终版本
按照正确的38字节数据包格式解析
"""

import serial
import serial.tools.list_ports
import struct
import time
import csv
import datetime
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class MotorMonitor:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("电机数据监控器")
        self.root.geometry("1000x700")

        self.serial_conn = None
        self.running = False
        self.packet_count = 0
        self.csv_file = None
        self.csv_writer = None

        # 数据存储 (最近500个点)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.motor1_pos = deque(maxlen=self.max_points)
        self.motor1_vel = deque(maxlen=self.max_points)
        self.motor1_cur = deque(maxlen=self.max_points)
        self.motor2_pos = deque(maxlen=self.max_points)
        self.motor2_vel = deque(maxlen=self.max_points)
        self.motor2_cur = deque(maxlen=self.max_points)

        self.start_time = time.time()

        # 波峰波谷检测相关
        self.last_peaks_valleys = {
            'motor1_pos': {'peaks': [], 'valleys': []},  # [(time, value), ...]
            'motor2_pos': {'peaks': [], 'valleys': []}
        }
        self.peak_valley_annotations = []  # 存储图上的标注

        self.setup_ui()
        self.setup_plots()

    def setup_ui(self):
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(control_frame, text="串口:").grid(row=0, column=0)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="波特率:").grid(row=0, column=2, padx=(10,0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(control_frame, textvariable=self.baud_var,
                    values=["115200"], width=10).grid(row=0, column=3, padx=5)

        ttk.Button(control_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=4, padx=5)
        self.connect_btn = ttk.Button(control_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5)
        ttk.Button(control_frame, text="清空", command=self.clear_data).grid(row=0, column=6, padx=5)

        self.status_label = ttk.Label(control_frame, text="状态: 未连接")
        self.status_label.grid(row=0, column=7, padx=20)

        self.packet_label = ttk.Label(control_frame, text="数据包: 0")
        self.packet_label.grid(row=0, column=8, padx=10)

        # 波峰波谷信息显示
        self.peak_info_frame = ttk.LabelFrame(self.root, text="波峰波谷信息")
        self.peak_info_frame.pack(fill=tk.X, padx=5, pady=5)

        self.motor1_info = ttk.Label(self.peak_info_frame, text="电机1位置: 等待数据...", font=('Arial', 9))
        self.motor1_info.pack(side=tk.LEFT, padx=10)

        self.motor2_info = ttk.Label(self.peak_info_frame, text="电机2位置: 等待数据...", font=('Arial', 9))
        self.motor2_info.pack(side=tk.LEFT, padx=10)

        self.refresh_ports()

    def setup_plots(self):
        self.fig, self.axes = plt.subplots(2, 3, figsize=(12, 6))
        self.fig.suptitle('电机数据实时监控')

        titles = [
            ['电机1 位置 (rad)', '电机1 速度 (rad/s)', '电机1 电流 (A)'],
            ['电机2 位置 (rad)', '电机2 速度 (rad/s)', '电机2 电流 (A)']
        ]

        self.lines = []
        for i in range(2):
            row_lines = []
            for j in range(3):
                ax = self.axes[i, j]
                ax.set_title(titles[i][j])
                ax.grid(True, alpha=0.3)
                line, = ax.plot([], [], 'b-', linewidth=1)
                row_lines.append(line)
                ax.set_xlabel('时间 (s)')
            self.lines.append(row_lines)

        plt.tight_layout()

        canvas_frame = ttk.Frame(self.root)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas = FigureCanvasTkAgg(self.fig, canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.animation = FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.running:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("错误", "请选择串口")
            return

        try:
            baud = int(self.baud_var.get())
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            self.running = True

            # 创建CSV文件
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"motor_data_{timestamp}.csv"
            self.csv_file = open(csv_filename, 'w', newline='', encoding='utf-8-sig')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                '时间', '电机1_ID', '电机1_位置', '电机1_速度', '电机1_电流', '电机1_温度',
                '电机2_ID', '电机2_位置', '电机2_速度', '电机2_电流', '电机2_温度'
            ])

            # 启动读取线程
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

            self.connect_btn.config(text="断开")
            self.status_label.config(text=f"已连接: {port}")

        except Exception as e:
            messagebox.showerror("连接错误", f"无法连接: {e}")

    def disconnect(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

        if self.csv_file:
            self.csv_file.close()

        self.connect_btn.config(text="连接")
        self.status_label.config(text="未连接")

    def read_serial(self):
        """串口读取线程 - 38字节数据包"""
        buffer = bytearray()

        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)

                    # 查找38字节数据包 (0xAA开头, 0x55结尾)
                    while len(buffer) >= 38:
                        aa_idx = buffer.find(0xAA)
                        if aa_idx == -1:
                            buffer.clear()
                            break

                        # 移动到AA位置
                        if aa_idx > 0:
                            buffer = buffer[aa_idx:]

                        # 检查是否有完整的38字节数据包
                        if len(buffer) >= 38:
                            packet = buffer[:38]

                            # 检查包尾是否为0x55
                            if packet[37] == 0x55:  # 第38个字节(索引37)是包尾
                                self.parse_packet(packet)
                                buffer = buffer[38:]
                            else:
                                # 包尾不对，移动1字节继续查找
                                buffer = buffer[1:]
                        else:
                            break

                time.sleep(0.001)

            except Exception as e:
                print(f"串口读取错误: {e}")
                break

    def parse_packet(self, packet_data):
        """解析38字节数据包"""
        try:
            # 按照C结构体格式解析 (38字节)
            # header(1) + motor_count(1) + motor1_id(1) + 4*float(16) + motor2_id(1) + 4*float(16) + checksum(1) + footer(1)
            fmt = '<BBBffffBffffBB'
            unpacked = struct.unpack(fmt, packet_data)

            header = unpacked[0]
            motor_count = unpacked[1]
            motor1_id = unpacked[2]
            motor1_pos = unpacked[3]
            motor1_vel = unpacked[4]
            motor1_cur = unpacked[5]
            motor1_temp = unpacked[6]
            motor2_id = unpacked[7]
            motor2_pos = unpacked[8]
            motor2_vel = unpacked[9]
            motor2_cur = unpacked[10]
            motor2_temp = unpacked[11]
            checksum = unpacked[12]
            footer = unpacked[13]

            # 验证包头包尾
            if header == 0xAA and footer == 0x55:
                current_time = time.time() - self.start_time

                # 添加到绘图数据
                self.time_data.append(current_time)
                self.motor1_pos.append(motor1_pos)
                self.motor1_vel.append(motor1_vel)
                self.motor1_cur.append(motor1_cur)
                self.motor2_pos.append(motor2_pos)
                self.motor2_vel.append(motor2_vel)
                self.motor2_cur.append(motor2_cur)

                # 写入CSV
                if self.csv_writer:
                    self.csv_writer.writerow([
                        datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
                        motor1_id, motor1_pos, motor1_vel, motor1_cur, motor1_temp,
                        motor2_id, motor2_pos, motor2_vel, motor2_cur, motor2_temp
                    ])

                self.packet_count += 1

        except struct.error:
            pass

    def detect_peaks_valleys(self, data_x, data_y, motor_name):
        """检测波峰和波谷"""
        if len(data_y) < 10:  # 数据太少，无法检测
            return

        # 转换为numpy数组
        y_array = np.array(data_y)
        x_array = np.array(data_x)

        # 简单的波峰波谷检测算法
        peaks = []
        valleys = []

        # 寻找局部极大值和极小值
        for i in range(2, len(y_array) - 2):
            # 检测波峰 (当前点大于前后各2个点)
            if (y_array[i] > y_array[i-1] and y_array[i] > y_array[i+1] and
                y_array[i] > y_array[i-2] and y_array[i] > y_array[i+2]):
                peaks.append((x_array[i], y_array[i]))

            # 检测波谷 (当前点小于前后各2个点)
            if (y_array[i] < y_array[i-1] and y_array[i] < y_array[i+1] and
                y_array[i] < y_array[i-2] and y_array[i] < y_array[i+2]):
                valleys.append((x_array[i], y_array[i]))

        # 只保留最新的几个极值点
        max_points = 5
        self.last_peaks_valleys[motor_name]['peaks'] = peaks[-max_points:]
        self.last_peaks_valleys[motor_name]['valleys'] = valleys[-max_points:]

    def update_peak_valley_info(self):
        """更新波峰波谷信息显示"""
        for motor_idx, motor_name in enumerate(['motor1_pos', 'motor2_pos']):
            peaks = self.last_peaks_valleys[motor_name]['peaks']
            valleys = self.last_peaks_valleys[motor_name]['valleys']

            info_text = f"电机{motor_idx+1}位置: "

            if len(peaks) >= 1 and len(valleys) >= 1:
                # 获取最新的波峰和波谷
                latest_peak = peaks[-1]
                latest_valley = valleys[-1]

                # 计算差值和时间差
                value_diff = abs(latest_peak[1] - latest_valley[1])
                time_diff = abs(latest_peak[0] - latest_valley[0])

                info_text += f"波峰: {latest_peak[1]:.3f}, 波谷: {latest_valley[1]:.3f}, "
                info_text += f"差值: {value_diff:.3f}, 时间差: {time_diff:.2f}s"
            else:
                info_text += "等待检测波峰波谷..."

            # 更新显示
            if motor_name == 'motor1_pos':
                self.motor1_info.config(text=info_text)
            else:
                self.motor2_info.config(text=info_text)

    def update_plots(self, frame):
        """更新图形"""
        self.packet_label.config(text=f"数据包: {self.packet_count}")

        if len(self.time_data) > 0:
            time_list = list(self.time_data)

            # 更新电机1图形
            self.lines[0][0].set_data(time_list, list(self.motor1_pos))
            self.lines[0][1].set_data(time_list, list(self.motor1_vel))
            self.lines[0][2].set_data(time_list, list(self.motor1_cur))

            # 更新电机2图形
            self.lines[1][0].set_data(time_list, list(self.motor2_pos))
            self.lines[1][1].set_data(time_list, list(self.motor2_vel))
            self.lines[1][2].set_data(time_list, list(self.motor2_cur))

            # 检测位置曲线的波峰波谷
            if len(time_list) > 10:
                self.detect_peaks_valleys(time_list, list(self.motor1_pos), 'motor1_pos')
                self.detect_peaks_valleys(time_list, list(self.motor2_pos), 'motor2_pos')

                # 在位置图上标注波峰波谷
                self.draw_peak_valley_markers()

                # 更新信息显示
                self.update_peak_valley_info()

            # 自动调整坐标轴
            for i in range(2):
                for j in range(3):
                    self.axes[i, j].relim()
                    self.axes[i, j].autoscale_view()

    def draw_peak_valley_markers(self):
        """在位置图上绘制波峰波谷标记"""
        # 清除之前的标注
        for annotation in self.peak_valley_annotations:
            annotation.remove()
        self.peak_valley_annotations.clear()

        # 为电机1位置图添加标记
        peaks = self.last_peaks_valleys['motor1_pos']['peaks']
        valleys = self.last_peaks_valleys['motor1_pos']['valleys']

        ax1 = self.axes[0, 0]  # 电机1位置图

        # 标记波峰 (红色三角向上)
        for peak_time, peak_val in peaks:
            ax1.scatter(peak_time, peak_val, color='red', marker='^', s=50, zorder=5)
            # 添加数值标注
            annotation = ax1.annotate(f'{peak_val:.3f}',
                                    xy=(peak_time, peak_val),
                                    xytext=(5, 10), textcoords='offset points',
                                    fontsize=8, color='red',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            self.peak_valley_annotations.append(annotation)

        # 标记波谷 (蓝色三角向下)
        for valley_time, valley_val in valleys:
            ax1.scatter(valley_time, valley_val, color='blue', marker='v', s=50, zorder=5)
            # 添加数值标注
            annotation = ax1.annotate(f'{valley_val:.3f}',
                                    xy=(valley_time, valley_val),
                                    xytext=(5, -15), textcoords='offset points',
                                    fontsize=8, color='blue',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            self.peak_valley_annotations.append(annotation)

        # 为电机2位置图添加标记
        peaks = self.last_peaks_valleys['motor2_pos']['peaks']
        valleys = self.last_peaks_valleys['motor2_pos']['valleys']

        ax2 = self.axes[1, 0]  # 电机2位置图

        # 标记波峰 (红色三角向上)
        for peak_time, peak_val in peaks:
            ax2.scatter(peak_time, peak_val, color='red', marker='^', s=50, zorder=5)
            annotation = ax2.annotate(f'{peak_val:.3f}',
                                    xy=(peak_time, peak_val),
                                    xytext=(5, 10), textcoords='offset points',
                                    fontsize=8, color='red',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            self.peak_valley_annotations.append(annotation)

        # 标记波谷 (蓝色三角向下)
        for valley_time, valley_val in valleys:
            ax2.scatter(valley_time, valley_val, color='blue', marker='v', s=50, zorder=5)
            annotation = ax2.annotate(f'{valley_val:.3f}',
                                    xy=(valley_time, valley_val),
                                    xytext=(5, -15), textcoords='offset points',
                                    fontsize=8, color='blue',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            self.peak_valley_annotations.append(annotation)

    def clear_data(self):
        """清空数据"""
        for data in [self.time_data, self.motor1_pos, self.motor1_vel, self.motor1_cur,
                     self.motor2_pos, self.motor2_vel, self.motor2_cur]:
            data.clear()

        # 清空波峰波谷数据
        for motor_name in ['motor1_pos', 'motor2_pos']:
            self.last_peaks_valleys[motor_name]['peaks'].clear()
            self.last_peaks_valleys[motor_name]['valleys'].clear()

        # 清除图上的标注
        for annotation in self.peak_valley_annotations:
            annotation.remove()
        self.peak_valley_annotations.clear()

        self.packet_count = 0
        self.start_time = time.time()

        # 重置信息显示
        self.motor1_info.config(text="电机1位置: 等待数据...")
        self.motor2_info.config(text="电机2位置: 等待数据...")

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.disconnect()

if __name__ == "__main__":
    app = MotorMonitor()
    app.run()