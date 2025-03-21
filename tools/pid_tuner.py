"""
PID調參工具 - 用於實時調整ESP32上運行的PID控制器參數
功能：
1. 讀取並顯示串口數據（目標RPM、當前RPM、誤差等）
2. 實時繪製RPM曲線圖
3. 提供滑桿和數字輸入框調整PID參數
4. 使用JSON格式與ESP32通信
5. 支持自動測試模式，可以設置起始RPM、中止RPM和間隔時間
"""

import sys
import time
import threading
import queue
import json
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
        self.root.geometry("1000x750")
        
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
        
        # 測試模式參數
        self.start_rpm_var = tk.StringVar(value="50")
        self.end_rpm_var = tk.StringVar(value="200")
        self.step_rpm_var = tk.StringVar(value="50")
        self.interval_var = tk.StringVar(value="5")  # 間隔時間(秒)
        self.test_running = False
        self.test_thread = None
        
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
        
        # 創建右側圖表區域
        plot_frame = ttk.LabelFrame(main_frame, text="數據圖表")
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 串口連接區域
        conn_frame = ttk.LabelFrame(control_frame, text="串口連接")
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 串口選擇
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(port_frame, text="串口:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(port_frame, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        
        # 連接和刷新按鈕
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.connect_button = ttk.Button(btn_frame, text="連接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        
        refresh_button = ttk.Button(btn_frame, text="刷新串口", command=self.refresh_ports)
        refresh_button.pack(side=tk.LEFT, padx=5)
        
        # PID參數調整區域
        pid_frame = ttk.LabelFrame(control_frame, text="PID參數調整")
        pid_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 目標RPM設置
        rpm_container = ttk.Frame(pid_frame)
        rpm_container.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(rpm_container, text="目標RPM:").pack(side=tk.LEFT)
        
        # 滑桿和輸入框框架
        rpm_frame = ttk.Frame(rpm_container)
        rpm_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
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
        
        # Kp
        kp_container = ttk.Frame(pid_frame)
        kp_container.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(kp_container, text="Kp:").pack(side=tk.LEFT)
        
        kp_frame = ttk.Frame(kp_container)
        kp_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.kp_scale = ttk.Scale(kp_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_kp_scale_change)
        self.kp_scale.set(0.5)  # 初始值
        self.kp_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        kp_entry = ttk.Entry(kp_frame, textvariable=self.kp_var, width=5, justify=tk.RIGHT)
        kp_entry.pack(side=tk.RIGHT)
        kp_entry.bind("<Return>", self.on_kp_entry_change)
        kp_entry.bind("<FocusOut>", self.on_kp_entry_change)
        
        # Ki
        ki_container = ttk.Frame(pid_frame)
        ki_container.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(ki_container, text="Ki:").pack(side=tk.LEFT)
        
        ki_frame = ttk.Frame(ki_container)
        ki_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.ki_scale = ttk.Scale(ki_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_ki_scale_change)
        self.ki_scale.set(0.2)  # 初始值
        self.ki_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        ki_entry = ttk.Entry(ki_frame, textvariable=self.ki_var, width=5, justify=tk.RIGHT)
        ki_entry.pack(side=tk.RIGHT)
        ki_entry.bind("<Return>", self.on_ki_entry_change)
        ki_entry.bind("<FocusOut>", self.on_ki_entry_change)
        
        # Kd
        kd_container = ttk.Frame(pid_frame)
        kd_container.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(kd_container, text="Kd:").pack(side=tk.LEFT)
        
        kd_frame = ttk.Frame(kd_container)
        kd_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.kd_scale = ttk.Scale(kd_frame, from_=0, to=2, orient=tk.HORIZONTAL, 
                                 command=self.on_kd_scale_change)
        self.kd_scale.set(0.1)  # 初始值
        self.kd_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        kd_entry = ttk.Entry(kd_frame, textvariable=self.kd_var, width=5, justify=tk.RIGHT)
        kd_entry.pack(side=tk.RIGHT)
        kd_entry.bind("<Return>", self.on_kd_entry_change)
        kd_entry.bind("<FocusOut>", self.on_kd_entry_change)
        
        # 操作按鈕區域 - 將三個按鈕放在同一行
        btn_container = ttk.Frame(pid_frame)
        btn_container.pack(fill=tk.X, padx=5, pady=5)
        
        # 發送參數按鈕
        send_params_button = ttk.Button(btn_container, text="發送參數", command=self.send_all_params)
        send_params_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # 歸零按鈕
        zero_button = ttk.Button(btn_container, text="歸零", command=self.send_zero_rpm)
        zero_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # 清除數據按鈕
        clear_button = ttk.Button(btn_container, text="清除數據", command=self.clear_data)
        clear_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # 測試模式框架
        test_frame = ttk.LabelFrame(control_frame, text="測試模式")
        test_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 測試參數設置 - 使用網格佈局
        test_grid = ttk.Frame(test_frame)
        test_grid.pack(fill=tk.X, padx=5, pady=5)
        
        # 第一行
        ttk.Label(test_grid, text="起始RPM:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(test_grid, textvariable=self.start_rpm_var, width=5, justify=tk.RIGHT).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(test_grid, text="結束RPM:").grid(row=0, column=2, sticky="w", padx=5, pady=2)
        ttk.Entry(test_grid, textvariable=self.end_rpm_var, width=5, justify=tk.RIGHT).grid(row=0, column=3, padx=5, pady=2)
        
        # 第二行
        ttk.Label(test_grid, text="步進值:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(test_grid, textvariable=self.step_rpm_var, width=5, justify=tk.RIGHT).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(test_grid, text="間隔(秒):").grid(row=1, column=2, sticky="w", padx=5, pady=2)
        ttk.Entry(test_grid, textvariable=self.interval_var, width=5, justify=tk.RIGHT).grid(row=1, column=3, padx=5, pady=2)
        
        # 測試按鈕
        self.test_button = ttk.Button(test_frame, text="開始測試", command=self.toggle_test_mode)
        self.test_button.pack(fill=tk.X, padx=5, pady=5)
        
        # 狀態顯示區域
        status_frame = ttk.Frame(control_frame)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(status_frame, text="狀態:", font=("Arial", 10)).pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value="未連接")
        ttk.Label(status_frame, textvariable=self.status_var, font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=5)
        
        # 實時數據顯示區域
        data_frame = ttk.LabelFrame(control_frame, text="實時數據")
        data_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.current_rpm_display = tk.StringVar(value="當前RPM: 0")
        self.target_rpm_display = tk.StringVar(value="目標RPM: 0")
        self.error_display = tk.StringVar(value="誤差: 0")
        self.output_display = tk.StringVar(value="輸出: 0")
        
        # 使用網格佈局排列數據顯示
        data_grid = ttk.Frame(data_frame)
        data_grid.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(data_grid, textvariable=self.target_rpm_display, width=15).grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(data_grid, textvariable=self.current_rpm_display, width=15).grid(row=0, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(data_grid, textvariable=self.error_display, width=15).grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(data_grid, textvariable=self.output_display, width=15).grid(row=1, column=1, sticky="w", padx=5, pady=2)
        
        # 日誌框架
        log_frame = ttk.LabelFrame(control_frame, text="系統日誌")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 添加滾動條
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log_text = tk.Text(log_frame, height=8, wrap=tk.WORD, yscrollcommand=log_scroll.set)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        log_scroll.config(command=self.log_text.yview)
        self.log_text.config(state=tk.DISABLED)
        
        # 創建圖表 - 使用2個子圖以分開顯示各項數據
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 配置圖表
        self.ax1.set_title("RPM vs Time")
        self.ax1.set_ylabel("RPM")
        self.ax1.grid(True)
        
        self.ax2.set_title("Motor Output & Error")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Value")
        self.ax2.grid(True)
        
        # 創建線條對象以便更新而不是重繪
        self.target_line, = self.ax1.plot([], [], 'r-', label="Target RPM")
        self.current_line, = self.ax1.plot([], [], 'b-', label="Current RPM")
        self.error_line, = self.ax2.plot([], [], 'm-', label="Error")
        self.output_line, = self.ax2.plot([], [], 'g-', label="Motor Output")
        
        self.ax1.legend()
        self.ax2.legend()
        
        self.fig.tight_layout()
    
    def add_log(self, message):
        """添加日誌到日誌框"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
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
                self.add_log(f"已連接到 {port}, 波特率: {self.baud_rate}")
                
                # 啟動串口讀取線程
                self.thread_running = True
                self.serial_thread = threading.Thread(target=self.serial_read_thread)
                self.serial_thread.daemon = True
                self.serial_thread.start()
            except Exception as e:
                self.status_var.set(f"連接錯誤: {str(e)}")
                self.add_log(f"連接錯誤: {str(e)}")
        else:
            # 停止測試模式（如果正在運行）
            if self.test_running:
                self.toggle_test_mode()
                
            # 停止線程
            self.thread_running = False
            if self.serial_thread:
                self.serial_thread.join(timeout=1.0)
            
            if self.serial_port:
                self.serial_port.close()
            self.connected = False
            self.connect_button.config(text="連接")
            self.status_var.set("已斷開連接")
            self.add_log("已斷開連接")
    
    def serial_read_thread(self):
        """串口讀取線程，獨立於UI線程運行"""
        buffer = ""
        
        while self.thread_running and self.connected:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    # 讀取所有可用數據
                    new_data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='replace')
                    buffer += new_data
                    
                    # 按行處理數據
                    lines = buffer.split('\n')
                    
                    # 保留最後一個不完整的行
                    buffer = lines.pop()
                    
                    # 處理完整的行
                    for line in lines:
                        line = line.strip()
                        if line:  # 忽略空行
                            # 將數據放入隊列
                            self.data_queue.put(line)
                
                # 短暫休眠，避免CPU佔用過高
                time.sleep(0.01)
            except Exception as e:
                # 將錯誤信息放入隊列
                self.data_queue.put(f"ERROR:{str(e)}")
                time.sleep(0.5)  # 錯誤後稍長休眠
    
    def parse_json_data(self, json_str):
        """解析JSON格式的數據"""
        try:
            data = json.loads(json_str)
            
            # 檢查data是否為字典類型
            if not isinstance(data, dict):
                self.add_log(f"[警告] 收到非字典JSON數據: {json_str}")
                return False
                
            data_type = data.get("type", "")
            
            if data_type == "data":
                # 處理遙測數據
                target_rpm = data.get("target_rpm", 0)
                current_rpm = data.get("current_rpm", 0)
                error = data.get("error", 0)
                motor_output = data.get("motor_output", 0)
                
                # 更新數據
                self.time_data.append(time.time() - self.start_time)
                self.target_rpm_data.append(target_rpm)
                self.current_rpm_data.append(current_rpm)
                self.error_data.append(error)
                self.motor_output_data.append(motor_output)
                
                # 更新顯示
                self.target_rpm_display.set(f"目標RPM: {target_rpm:.1f}")
                self.current_rpm_display.set(f"當前RPM: {current_rpm:.1f}")
                self.error_display.set(f"誤差: {error:.1f}")
                self.output_display.set(f"輸出: {motor_output:.1f}")
                
            elif data_type == "response":
                # 處理命令響應
                status = data.get("status", "")
                message = data.get("message", "")
                
                if status == "success":
                    self.status_var.set(f"成功: {message}")
                else:
                    self.status_var.set(f"錯誤: {message}")
                
                self.add_log(f"[響應] {message}")
                
            elif data_type == "log" or data_type == "status":
                # 處理日誌和狀態信息
                message = data.get("message", "")
                self.add_log(f"[{data_type}] {message}")
                
            elif data_type == "error":
                # 處理錯誤信息
                message = data.get("message", "")
                self.status_var.set(f"錯誤: {message}")
                self.add_log(f"[錯誤] {message}")
                
            return True
        except json.JSONDecodeError:
            return False
        except Exception as e:
            # 捕獲所有其他異常，避免程序崩潰
            self.add_log(f"[錯誤] JSON解析異常: {str(e)}")
            return False
    
    def update_plot(self, frame):
        """更新圖表 - 由FuncAnimation定期調用"""
        # 處理隊列中的所有數據
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                
                # 嘗試解析為JSON
                if not self.parse_json_data(line):
                    # 如果不是JSON，則按舊格式處理
                    if line.startswith("ERROR:"):
                        error_msg = line.split(":", 1)[1] if len(line.split(":", 1)) > 1 else "未知錯誤"
                        self.status_var.set(f"錯誤: {error_msg}")
                        self.add_log(f"[錯誤] {error_msg}")
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
                
                # 確保上限和下限不相同
                if max_rpm == min_rpm:
                    if max_rpm == 0:
                        max_rpm = 10  # 如果都是0，設置一個默認範圍
                    else:
                        max_rpm *= 1.1  # 增加上限
                        min_rpm *= 0.9  # 減少下限
                
                self.ax1.set_ylim(min_rpm, max_rpm)
            
            if self.error_data and self.motor_output_data:
                all_values = self.error_data + self.motor_output_data
                max_val = max(all_values) * 1.1
                min_val = min(all_values) * 1.1
                
                # 確保上限和下限不相同
                if max_val == min_val:
                    if max_val == 0:
                        max_val = 10
                        min_val = -10
                    else:
                        max_val *= 1.1
                        min_val *= 0.9
                
                self.ax2.set_ylim(min_val, max_val)
        
        # 更新圖表
        self.fig.canvas.draw_idle()
        
        return self.target_line, self.current_line, self.error_line, self.output_line
    
    def send_all_params(self):
        """發送所有參數（合併PID參數和目標RPM）"""
        if not self.connected or not self.serial_port:
            self.status_var.set("未連接，無法發送參數")
            return
            
        try:
            # 獲取當前PID參數值
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            rpm = int(float(self.target_rpm_var.get()))
            
            # 先發送PID參數
            pid_command = {
                "command": "set_pid",
                "kp": kp,
                "ki": ki,
                "kd": kd
            }
            
            json_str = json.dumps(pid_command) + "\n"
            self.serial_port.write(json_str.encode())
            self.add_log(f"已發送PID參數: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
            
            # 稍微延遲，確保命令不會太快發送
            time.sleep(0.1)
            
            # 再發送目標RPM
            rpm_command = {
                "command": "set_rpm",
                "value": rpm
            }
            
            json_str = json.dumps(rpm_command) + "\n"
            self.serial_port.write(json_str.encode())
            self.add_log(f"已發送目標RPM: {rpm}")
            
            self.status_var.set(f"已發送所有參數")
        except ValueError as e:
            self.status_var.set(f"參數格式錯誤: {str(e)}")
        except Exception as e:
            self.status_var.set(f"發送錯誤: {str(e)}")
    
    def send_zero_rpm(self):
        """發送RPM=0命令（歸零）"""
        if not self.connected or not self.serial_port:
            self.status_var.set("未連接，無法發送歸零命令")
            return
            
        try:
            # 發送RPM=0命令，但不改變界面上的目標RPM值
            command = {
                "command": "set_rpm",
                "value": 0
            }
            
            json_str = json.dumps(command) + "\n"
            self.serial_port.write(json_str.encode())
            self.status_var.set("已發送歸零命令 (RPM=0)")
            self.add_log("已發送歸零命令 (RPM=0)")
        except Exception as e:
            self.status_var.set(f"發送錯誤: {str(e)}")
    
    def toggle_test_mode(self):
        """切換測試模式"""
        if not self.test_running:
            # 檢查是否已連接
            if not self.connected:
                messagebox.showerror("錯誤", "請先連接設備")
                return
                
            # 獲取測試參數
            try:
                start_rpm = int(float(self.start_rpm_var.get()))
                end_rpm = int(float(self.end_rpm_var.get()))
                step_rpm = int(float(self.step_rpm_var.get()))
                interval = float(self.interval_var.get())
                
                if start_rpm < 0 or end_rpm < 0 or step_rpm <= 0 or interval <= 0:
                    messagebox.showerror("錯誤", "參數必須為正數")
                    return
                    
                if start_rpm > end_rpm and step_rpm > 0:
                    messagebox.showerror("錯誤", "起始RPM大於結束RPM，但步進值為正")
                    return
                    
                if start_rpm < end_rpm and step_rpm < 0:
                    messagebox.showerror("錯誤", "起始RPM小於結束RPM，但步進值為負")
                    return
            except ValueError:
                messagebox.showerror("錯誤", "請輸入有效的數字")
                return
                
            # 開始測試
            self.test_running = True
            self.test_button.config(text="停止測試")
            self.status_var.set("測試模式運行中...")
            
            # 啟動測試線程
            self.test_thread = threading.Thread(target=self.run_test, 
                                              args=(start_rpm, end_rpm, step_rpm, interval))
            self.test_thread.daemon = True
            self.test_thread.start()
        else:
            # 停止測試
            self.test_running = False
            self.test_button.config(text="開始測試")
            self.status_var.set("測試已停止")
            self.add_log("測試模式已停止")
    
    def run_test(self, start_rpm, end_rpm, step_rpm, interval):
        """運行測試模式"""
        self.add_log(f"開始測試: 從{start_rpm}RPM到{end_rpm}RPM, 步進={step_rpm}, 間隔={interval}秒")
        
        # 獲取當前PID參數
        kp = float(self.kp_var.get())
        ki = float(self.ki_var.get())
        kd = float(self.kd_var.get())
        
        # 發送PID參數
        pid_command = {
            "command": "set_pid",
            "kp": kp,
            "ki": ki,
            "kd": kd
        }
        
        json_str = json.dumps(pid_command) + "\n"
        self.serial_port.write(json_str.encode())
        self.add_log(f"測試使用PID參數: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")
        
        # 計算步進方向
        if step_rpm > 0:
            rpm_range = range(start_rpm, end_rpm + step_rpm, step_rpm)
        else:
            rpm_range = range(start_rpm, end_rpm + step_rpm, step_rpm)
        
        # 執行測試
        for rpm in rpm_range:
            if not self.test_running:
                break
                
            # 更新UI
            self.root.after(0, lambda r=rpm: self.target_rpm_var.set(str(r)))
            self.root.after(0, lambda r=rpm: self.target_rpm_scale.set(r))
            self.root.after(0, lambda r=rpm: self.status_var.set(f"測試中: 當前RPM={r}"))
            
            # 發送RPM命令
            rpm_command = {
                "command": "set_rpm",
                "value": rpm
            }
            
            json_str = json.dumps(rpm_command) + "\n"
            self.serial_port.write(json_str.encode())
            self.add_log(f"測試設置RPM: {rpm}")
            
            # 等待指定間隔
            start_wait = time.time()
            while time.time() - start_wait < interval and self.test_running:
                time.sleep(0.1)
        
        # 測試完成
        if self.test_running:
            self.root.after(0, lambda: self.status_var.set("測試完成"))
            self.add_log("測試序列已完成")
            self.root.after(0, lambda: self.test_button.config(text="開始測試"))
            self.test_running = False
    
    def clear_data(self):
        self.time_data = []
        self.target_rpm_data = []
        self.current_rpm_data = []
        self.error_data = []
        self.motor_output_data = []
        self.start_time = time.time()
        self.status_var.set("數據已清除")
        self.add_log("圖表數據已清除")

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTuner(root)
    root.mainloop()