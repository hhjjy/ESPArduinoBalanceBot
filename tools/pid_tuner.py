"""
PID調參工具 - 用於實時調整ESP32上運行的PID控制器參數
功能：
1. 讀取並顯示串口數據（目標RPM、當前RPM、誤差等）
2. 實時繪製RPM曲線圖
3. 提供滑桿和數字輸入框調整PID參數
4. 將調整後的參數發送到ESP32
5. 多線程設計確保UI響應性和數據實時性
"""

import sys
import time
import threading
import queue
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class PIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("PID調參工具")
        self.root.geometry("1000x700")
        
        # 串口設置
        self.serial_port = None
        self.baud_rate = 115200
        self.connected = False
        self.serial_thread = None
        self.thread_running = False
        
        # 數據隊列用於線程間通信
        self.data_queue = queue.Queue()
        
        # 數據存儲
        self.time_data = []
        self.target_rpm_data = []
        self.current_rpm_data = []
        self.error_data = []
        self.motor_output_data = []
        self.start_time = time.time()
        
        # PID參數 - 使用StringVar以支持直接輸入
        self.kp_var = tk.StringVar(value="0.50")
        self.ki_var = tk.StringVar(value="0.20")
        self.kd_var = tk.StringVar(value="0.10")
        self.target_rpm_var = tk.StringVar(value="50")
        
        # 創建GUI
        self.create_gui()
        
        # 設置動畫 - 提高更新頻率
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        
    def create_gui(self):
        # 創建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 創建左側控制面板
        control_frame = ttk.LabelFrame(main_frame, text="控制面板")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        # 串口選擇
        ttk.Label(control_frame, text="串口:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.port_combo = ttk.Combobox(control_frame, width=20)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_ports()
        
        # 連接按鈕
        self.connect_button = ttk.Button(control_frame, text="連接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        
        # 刷新串口按鈕
        refresh_button = ttk.Button(control_frame, text="刷新串口", command=self.refresh_ports)
        refresh_button.grid(row=1, column=0, columnspan=3, padx=5, pady=5)
        
        # 目標RPM設置 - 添加輸入框
        ttk.Label(control_frame, text="目標RPM:").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        
        # 創建一個包含滑桿和輸入框的框架
        rpm_frame = ttk.Frame(control_frame)
        rpm_frame.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # 滑桿
        self.target_rpm_scale = ttk.Scale(rpm_frame, from_=0, to=300, orient=tk.HORIZONTAL, 
                                         command=self.on_rpm_scale_change)
        self.target_rpm_scale.set(50)  # 初始值
        self.target_rpm_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        # 輸入框
        rpm_entry = ttk.Entry(rpm_frame, textvariable=self.target_rpm_var, width=5, justify=tk.RIGHT)
        rpm_entry.pack(side=tk.RIGHT)
        rpm_entry.bind("<Return>", self.on_rpm_entry_change)
        rpm_entry.bind("<FocusOut>", self.on_rpm_entry_change)
        
        # 發送目標RPM按鈕
        send_rpm_button = ttk.Button(control_frame, text="發送目標RPM", command=self.send_target_rpm)
        send_rpm_button.grid(row=3, column=0, columnspan=3, padx=5, pady=5)
        
        # PID參數調整 - 添加輸入框
        # Kp
        ttk.Label(control_frame, text="Kp:").grid(row=4, column=0, sticky="w", padx=5, pady=5)
        
        kp_frame = ttk.Frame(control_frame)
        kp_frame.grid(row=4, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.kp_scale = ttk.Scale(kp_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_kp_scale_change)
        self.kp_scale.set(0.5)  # 初始值
        self.kp_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        kp_entry = ttk.Entry(kp_frame, textvariable=self.kp_var, width=5, justify=tk.RIGHT)
        kp_entry.pack(side=tk.RIGHT)
        kp_entry.bind("<Return>", self.on_kp_entry_change)
        kp_entry.bind("<FocusOut>", self.on_kp_entry_change)
        
        # Ki
        ttk.Label(control_frame, text="Ki:").grid(row=5, column=0, sticky="w", padx=5, pady=5)
        
        ki_frame = ttk.Frame(control_frame)
        ki_frame.grid(row=5, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.ki_scale = ttk.Scale(ki_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_ki_scale_change)
        self.ki_scale.set(0.2)  # 初始值
        self.ki_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        ki_entry = ttk.Entry(ki_frame, textvariable=self.ki_var, width=5, justify=tk.RIGHT)
        ki_entry.pack(side=tk.RIGHT)
        ki_entry.bind("<Return>", self.on_ki_entry_change)
        ki_entry.bind("<FocusOut>", self.on_ki_entry_change)
        
        # Kd
        ttk.Label(control_frame, text="Kd:").grid(row=6, column=0, sticky="w", padx=5, pady=5)
        
        kd_frame = ttk.Frame(control_frame)
        kd_frame.grid(row=6, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.kd_scale = ttk.Scale(kd_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_kd_scale_change)
        self.kd_scale.set(0.1)  # 初始值
        self.kd_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        kd_entry = ttk.Entry(kd_frame, textvariable=self.kd_var, width=5, justify=tk.RIGHT)
        kd_entry.pack(side=tk.RIGHT)
        kd_entry.bind("<Return>", self.on_kd_entry_change)
        kd_entry.bind("<FocusOut>", self.on_kd_entry_change)
        
        # 發送PID參數按鈕
        send_pid_button = ttk.Button(control_frame, text="發送PID參數", command=self.send_pid_params)
        send_pid_button.grid(row=7, column=0, columnspan=3, padx=5, pady=5)
        
        # 清除數據按鈕
        clear_button = ttk.Button(control_frame, text="清除數據", command=self.clear_data)
        clear_button.grid(row=8, column=0, columnspan=3, padx=5, pady=5)
        
        # 狀態顯示
        self.status_var = tk.StringVar(value="未連接")
        status_label = ttk.Label(control_frame, textvariable=self.status_var, font=("Arial", 10, "bold"))
        status_label.grid(row=9, column=0, columnspan=3, padx=5, pady=5)
        
        # 創建右側圖表區域
        plot_frame = ttk.LabelFrame(main_frame, text="數據圖表")
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 創建圖表 - 使用3個子圖以分開顯示各項數據
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 配置圖表
        self.ax1.set_title("RPM vs Time")
        self.ax1.set_ylabel("RPM")
        self.ax1.grid(True)
        
        self.ax2.set_title("Error")
        self.ax2.set_ylabel("Error")
        self.ax2.grid(True)
        
        self.ax3.set_title("Motor Output")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("Output")
        self.ax3.grid(True)
        
        # 創建線條對象以便更新而不是重繪
        self.target_line, = self.ax1.plot([], [], 'r-', label="Target RPM")
        self.current_line, = self.ax1.plot([], [], 'b-', label="Current RPM")
        self.error_line, = self.ax2.plot([], [], 'm-', label="Error")
        self.output_line, = self.ax3.plot([], [], 'g-', label="Motor Output")
        
        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()
        
        self.fig.tight_layout()
        
        # 數據顯示區域
        data_frame = ttk.LabelFrame(control_frame, text="實時數據")
        data_frame.grid(row=10, column=0, columnspan=3, padx=5, pady=5, sticky="ew")
        
        self.current_rpm_display = tk.StringVar(value="當前RPM: 0")
        self.target_rpm_display = tk.StringVar(value="目標RPM: 0")
        self.error_display = tk.StringVar(value="誤差: 0")
        self.output_display = tk.StringVar(value="輸出: 0")
        
        ttk.Label(data_frame, textvariable=self.current_rpm_display).pack(anchor="w", padx=5, pady=2)
        ttk.Label(data_frame, textvariable=self.target_rpm_display).pack(anchor="w", padx=5, pady=2)
        ttk.Label(data_frame, textvariable=self.error_display).pack(anchor="w", padx=5, pady=2)
        ttk.Label(data_frame, textvariable=self.output_display).pack(anchor="w", padx=5, pady=2)
    
    # 處理滑桿和輸入框的值同步
    def on_rpm_scale_change(self, value):
        self.target_rpm_var.set(f"{float(value):.0f}")
    
    def on_rpm_entry_change(self, event=None):
        try:
            value = float(self.target_rpm_var.get())
            if value < 0:
                value = 0
            elif value > 300:
                value = 300
            self.target_rpm_var.set(f"{value:.0f}")
            self.target_rpm_scale.set(value)
        except ValueError:
            self.target_rpm_var.set("50")
            self.target_rpm_scale.set(50)
    
    def on_kp_scale_change(self, value):
        self.kp_var.set(f"{float(value):.2f}")
    
    def on_kp_entry_change(self, event=None):
        try:
            value = float(self.kp_var.get())
            if value < 0:
                value = 0
            elif value > 2:
                value = 2
            self.kp_var.set(f"{value:.2f}")
            self.kp_scale.set(value)
        except ValueError:
            self.kp_var.set("0.50")
            self.kp_scale.set(0.5)
    
    def on_ki_scale_change(self, value):
        self.ki_var.set(f"{float(value):.2f}")
    
    def on_ki_entry_change(self, event=None):
        try:
            value = float(self.ki_var.get())
            if value < 0:
                value = 0
            elif value > 2:
                value = 2
            self.ki_var.set(f"{value:.2f}")
            self.ki_scale.set(value)
        except ValueError:
            self.ki_var.set("0.20")
            self.ki_scale.set(0.2)
    
    def on_kd_scale_change(self, value):
        self.kd_var.set(f"{float(value):.2f}")
    
    def on_kd_entry_change(self, event=None):
        try:
            value = float(self.kd_var.get())
            if value < 0:
                value = 0
            elif value > 2:
                value = 2
            self.kd_var.set(f"{value:.2f}")
            self.kd_scale.set(value)
        except ValueError:
            self.kd_var.set("0.10")
            self.kd_scale.set(0.1)
        
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
            
    def toggle_connection(self):
        if not self.connected:
            try:
                port = self.port_combo.get()
                self.serial_port = serial.Serial(port, self.baud_rate, timeout=0.1)
                self.connected = True
                self.connect_button.config(text="斷開")
                self.status_var.set(f"已連接到 {port}")
                self.start_time = time.time()
                
                # 啟動串口讀取線程
                self.thread_running = True
                self.serial_thread = threading.Thread(target=self.serial_read_thread)
                self.serial_thread.daemon = True
                self.serial_thread.start()
            except Exception as e:
                self.status_var.set(f"連接錯誤: {str(e)}")
        else:
            # 停止線程
            self.thread_running = False
            if self.serial_thread:
                self.serial_thread.join(timeout=1.0)
            
            if self.serial_port:
                self.serial_port.close()
            self.connected = False
            self.connect_button.config(text="連接")
            self.status_var.set("已斷開連接")
    
    def serial_read_thread(self):
        """串口讀取線程，獨立於UI線程運行"""
        while self.thread_running and self.connected:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    # 讀取所有可用數據
                    raw_data = self.serial_port.read(self.serial_port.in_waiting)
                    try:
                        # 嘗試解碼為UTF-8
                        decoded_data = raw_data.decode('utf-8')
                        
                        # 按行處理數據
                        lines = decoded_data.strip().split('\n')
                        for line in lines:
                            if line.strip():  # 忽略空行
                                # 將數據放入隊列
                                self.data_queue.put(line.strip())
                    except UnicodeDecodeError:
                        # 解碼失敗，可能是二進制數據
                        self.data_queue.put(f"ERROR:無法解碼數據")
                
                # 短暫休眠，避免CPU佔用過高
                time.sleep(0.01)
            except Exception as e:
                # 將錯誤信息放入隊列
                self.data_queue.put(f"ERROR:{str(e)}")
                time.sleep(0.5)  # 錯誤後稍長休眠
            
    def parse_data(self, line):
        """解析一行數據"""
        # 檢查數據格式
        if line.startswith(">target_rpm:"):
            try:
                value = float(line.split(":")[1])
                self.target_rpm_data.append(value)
                self.target_rpm_display.set(f"目標RPM: {value:.1f}")
                self.time_data.append(time.time() - self.start_time)
            except Exception as e:
                print(f"解析目標RPM失敗: {str(e)}")
            
        elif line.startswith(">current_rpm:"):
            try:
                value = float(line.split(":")[1])
                self.current_rpm_data.append(value)
                self.current_rpm_display.set(f"當前RPM: {value:.1f}")
            except Exception as e:
                print(f"解析當前RPM失敗: {str(e)}")
            
        elif line.startswith(">error:"):
            try:
                value = float(line.split(":")[1])
                self.error_data.append(value)
                self.error_display.set(f"誤差: {value:.1f}")
            except Exception as e:
                print(f"解析誤差失敗: {str(e)}")
            
        elif line.startswith(">motor_output:"):
            try:
                value = float(line.split(":")[1])
                self.motor_output_data.append(value)
                self.output_display.set(f"輸出: {value:.1f}")
            except Exception as e:
                print(f"解析馬達輸出失敗: {str(e)}")
        elif line.startswith(">status:"):
            # 處理狀態信息
            status_msg = line.split(":", 1)[1] if len(line.split(":", 1)) > 1 else "收到狀態信息"
            self.status_var.set(status_msg)
        elif line.startswith("ERROR:"):
            # 處理錯誤信息
            error_msg = line.split(":", 1)[1] if len(line.split(":", 1)) > 1 else "未知錯誤"
            self.status_var.set(f"錯誤: {error_msg}")
            
    def update_plot(self, frame):
        """更新圖表 - 由FuncAnimation定期調用"""
        # 處理隊列中的所有數據
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                self.parse_data(line)
            except queue.Empty:
                break
        
        # 限制數據點數量，防止內存溢出
        max_points = 500
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.target_rpm_data = self.target_rpm_data[-max_points:]
            self.current_rpm_data = self.current_rpm_data[-max_points:]
            self.error_data = self.error_data[-max_points:]
            self.motor_output_data = self.motor_output_data[-max_points:]
        
        # 更新圖表 - 使用set_data而不是重繪整個圖表
        if len(self.time_data) > 0:
            # 更新時間軸範圍，只顯示最近30秒的數據
            current_time = time.time() - self.start_time
            self.ax1.set_xlim(max(0, current_time - 30), current_time + 0.5)
            self.ax2.set_xlim(max(0, current_time - 30), current_time + 0.5)
            self.ax3.set_xlim(max(0, current_time - 30), current_time + 0.5)
            
            # 更新數據線
            self.target_line.set_data(self.time_data, self.target_rpm_data)
            self.current_line.set_data(self.time_data, self.current_rpm_data)
            
            if self.error_data:
                self.error_line.set_data(self.time_data, self.error_data)
            
            if self.motor_output_data:
                self.output_line.set_data(self.time_data, self.motor_output_data)
            
            # 動態調整Y軸範圍
            if self.target_rpm_data and self.current_rpm_data:
                max_rpm = max(max(self.target_rpm_data), max(self.current_rpm_data)) * 1.1
                min_rpm = min(min(self.target_rpm_data), min(self.current_rpm_data)) * 0.9
                min_rpm = min(0, min_rpm)  # 確保下限不高於0
                self.ax1.set_ylim(min_rpm, max_rpm)
            
            if self.error_data:
                max_error = max(self.error_data) * 1.1
                min_error = min(self.error_data) * 1.1
                self.ax2.set_ylim(min_error, max_error)
            
            if self.motor_output_data:
                max_output = max(self.motor_output_data) * 1.1
                min_output = min(self.motor_output_data) * 0.9
                self.ax3.set_ylim(min_output, max_output)
        
        # 更新圖表
        self.fig.canvas.draw_idle()
        
        return self.target_line, self.current_line, self.error_line, self.output_line
            
    def send_pid_params(self):
        if not self.connected or not self.serial_port:
            self.status_var.set("未連接，無法發送參數")
            return
            
        try:
            # 獲取當前PID參數值
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            
            # 發送格式: "PID:Kp,Ki,Kd"
            command = f"PID:{kp:.2f},{ki:.2f},{kd:.2f}\n"
            self.serial_port.write(command.encode())
            self.status_var.set(f"已發送PID參數: {command.strip()}")
        except ValueError as e:
            self.status_var.set(f"參數格式錯誤: {str(e)}")
        except Exception as e:
            self.status_var.set(f"發送錯誤: {str(e)}")
            
    def send_target_rpm(self):
        if not self.connected or not self.serial_port:
            self.status_var.set("未連接，無法發送目標RPM")
            return
            
        try:
            # 獲取當前目標RPM值
            rpm = int(float(self.target_rpm_var.get()))
            
            # 發送格式: "RPM:value"
            command = f"RPM:{rpm}\n"
            self.serial_port.write(command.encode())
            self.status_var.set(f"已發送目標RPM: {command.strip()}")
        except ValueError as e:
            self.status_var.set(f"RPM格式錯誤: {str(e)}")
        except Exception as e:
            self.status_var.set(f"發送錯誤: {str(e)}")
            
    def clear_data(self):
        self.time_data = []
        self.target_rpm_data = []
        self.current_rpm_data = []
        self.error_data = []
        self.motor_output_data = []
        self.start_time = time.time()
        self.status_var.set("數據已清除")

if __name__ == "__main__":
    import argparse
    
    # 創建命令行參數解析器
    parser = argparse.ArgumentParser(description='PID調參工具')
    parser.add_argument('--port', help='串口設備名稱 (例如: COM3, /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='波特率 (默認: 115200)')
    parser.add_argument('--debug', action='store_true', help='啟用詳細調試輸出')
    
    args = parser.parse_args()
    
    # 設置全局調試標誌
    DEBUG = args.debug
    if DEBUG:
        print("調試模式已啟用")
    
    root = tk.Tk()
    app = PIDTuner(root)
    
    # 如果指定了串口，自動連接
    if args.port:
        app.port_combo.set(args.port)
        app.baud_rate = args.baud
        print(f"嘗試自動連接到 {args.port}, 波特率: {args.baud}")
        app.toggle_connection()
    
    root.mainloop()