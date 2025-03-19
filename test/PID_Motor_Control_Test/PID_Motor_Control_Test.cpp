/**
 * PID_Motor_Control_Test.cpp
 * 
 * 用於測試馬達PID控制內環參數的程式 (RTOS版本)
 * 功能：
 * 1. 顯示目標RPM和當前RPM的示波器視圖
 * 2. 按下Boot按鈕增加目標RPM（每次增加50 RPM）
 * 3. 使用PID控制器調整馬達速度以達到目標RPM
 * 4. 可以調整PID參數
 * 5. 使用RTOS確保任務準時執行
 */

 #include <Arduino.h>
 #include "motor.h"
 #include "encoder.h"
 #include "OLED_Manager.h"
 #include "pages/DebugPage.h"
 #include "config.h"
 #include "PID_v1.h"
 
 // RTOS相關定義
 #define STACK_SIZE 4096
 #define PID_TASK_PRIORITY 3
 #define SENSOR_TASK_PRIORITY 3
 #define DISPLAY_TASK_PRIORITY 1
 #define SERIAL_TASK_PRIORITY 2
 #define BUTTON_TASK_PRIORITY 1
 
 // 任務句柄
 TaskHandle_t pidTaskHandle = NULL;
 TaskHandle_t sensorTaskHandle = NULL;
 TaskHandle_t displayTaskHandle = NULL;
 TaskHandle_t serialTaskHandle = NULL;
 TaskHandle_t buttonTaskHandle = NULL;
 
 // 互斥鎖，用於保護共享資源
 SemaphoreHandle_t dataMutex;
 
 // 調試控制標誌
 #define DEBUG_LEVEL 3  // 0: 無調試輸出, 1: 基本調試, 2: 詳細調試, 3: 所有數據
 
 // 創建馬達對象
 Motor motor1(MOTOR1_PWM, MOTOR1_AIN1, MOTOR1_AIN2, MOTOR_STBY, "motor1");
 Motor motor2(MOTOR2_PWM, MOTOR2_AIN1, MOTOR2_AIN2, MOTOR_STBY, "motor2");
 
 // 創建編碼器對象
 Encoder encoder1(MOTOR1_ENA, MOTOR1_ENB, "encoder1", 440);
 Encoder encoder2(MOTOR2_ENA, MOTOR2_ENB, "encoder2", 440);
 
 // 創建 OLED 管理器
 OLED_Manager oled;
 
 // PID 變量
 double targetRPM = 0;    // 目標RPM
 double currentRPM = 0;   // 當前RPM
 double motorOutput = 0;  // 馬達輸出 (0-255)
 
 // PID 參數 - 這些是初始值，可以根據實際情況調整
 double Kp = 0.5;
 double Ki = 0.2;
 double Kd = 0.1;
 
 // 創建 PID 控制器
 PID motorPID(&currentRPM, &motorOutput, &targetRPM, Kp, Ki, Kd, DIRECT);
 
 // 創建調試頁面
 DebugPage debugPage(&targetRPM, &currentRPM, &Kp, &Ki, &Kd);
 
 // 按鈕定義
 #define BUTTON_PIN 0  // BOOT 按鈕
 #define PARAM_BUTTON_PIN 1  // 參數調整按鈕
 
 // 按鈕狀態
 bool lastButtonState = HIGH;
 bool lastParamButtonState = HIGH;
 unsigned long lastDebounceTime = 0;
 unsigned long lastParamDebounceTime = 0;
 unsigned long debounceDelay = 50;
 
 // 調試變量
 unsigned long lastDebugTime = 0;
 const unsigned long DEBUG_INTERVAL = 500;  // 每0.5秒輸出一次調試信息
 
 // 數據記錄陣列 (用於繪製示波器)
 #define GRAPH_POINTS 64
 double rpmHistory[GRAPH_POINTS] = {0};
 int historyIndex = 0;
 
 // 標誌變量
 volatile bool motorsEnabled = false;
 
 // 函數聲明
 void updateRPMHistory(double rpm);
 void processSerialCommands();
 
 /**
  * PID控制任務 - 計算PID並設置馬達輸出
  * 頻率: 100Hz (10ms)
  */
 void pidControlTask(void *pvParameters) {
   TickType_t xLastWakeTime;
   const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
   
   // 初始化xLastWakeTime變數
   xLastWakeTime = xTaskGetTickCount();
   
   if (DEBUG_LEVEL >= 1) {
     Serial.println("PID控制任務已啟動");
   }
   
   while (1) {
     // 等待下一個週期
     vTaskDelayUntil(&xLastWakeTime, xFrequency);
     
     // 獲取互斥鎖
     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
       // 計算PID
       motorPID.Compute();
       
       // 如果馬達已啟用，設置馬達輸出
       if (motorsEnabled) {
         motor1.setSpeed(motorOutput);
         motor2.setSpeed(motorOutput);
       }
       
       // 釋放互斥鎖
       xSemaphoreGive(dataMutex);
     }
   }
 }
 
 /**
  * 感測器讀取任務 - 讀取編碼器並更新RPM
  * 頻率: 200Hz (5ms)
  */
 void sensorReadTask(void *pvParameters) {
   TickType_t xLastWakeTime;
   const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz
   
   xLastWakeTime = xTaskGetTickCount();
   
   if (DEBUG_LEVEL >= 1) {
     Serial.println("感測器讀取任務已啟動");
   }
   
   while (1) {
     vTaskDelayUntil(&xLastWakeTime, xFrequency);
     
     // 更新編碼器狀態
     encoder1.update();
     encoder2.update();
     
     // 獲取互斥鎖
     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
       // 計算當前 RPM (兩個馬達的平均值)
       currentRPM = (encoder1.getRPM() + encoder2.getRPM()) / 2.0;
       
       // 更新RPM歷史記錄
       updateRPMHistory(currentRPM);
       
       // 釋放互斥鎖
       xSemaphoreGive(dataMutex);
     }
   }
 }
 
 /**
  * 顯示更新任務 - 更新OLED顯示
  * 頻率: 20Hz (50ms)
  */
 void displayUpdateTask(void *pvParameters) {
   TickType_t xLastWakeTime;
   const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
   
   xLastWakeTime = xTaskGetTickCount();
   
   if (DEBUG_LEVEL >= 1) {
     Serial.println("顯示更新任務已啟動");
   }
   
   while (1) {
     vTaskDelayUntil(&xLastWakeTime, xFrequency);
     
     // 更新OLED顯示 (不需要互斥鎖，因為DebugPage通過指針訪問數據)
     oled.update();
   }
 }
 
 /**
  * 串口通信任務 - 處理串口命令並發送遙測數據
  * 頻率: 50Hz (20ms)
  */
 void serialCommunicationTask(void *pvParameters) {
   TickType_t xLastWakeTime;
   const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
   unsigned long currentTime;
   
   xLastWakeTime = xTaskGetTickCount();
   
   if (DEBUG_LEVEL >= 1) {
     Serial.println("串口通信任務已啟動");
   }
   
   while (1) {
     vTaskDelayUntil(&xLastWakeTime, xFrequency);
     
     // 處理串口命令
     processSerialCommands();
     
     // 定期輸出調試信息和遙測數據
     currentTime = millis();
     if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
       lastDebugTime = currentTime;
       
       // 獲取互斥鎖
       if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
         if (DEBUG_LEVEL >= 1) {
           Serial.println("\n--- 馬達 PID 控制狀態 ---");
           Serial.print("目標 RPM: ");
           Serial.println(targetRPM);
           Serial.print("當前 RPM: ");
           Serial.println(currentRPM);
           Serial.print("馬達輸出: ");
           Serial.println(motorOutput);
           Serial.print("PID 參數 - Kp: ");
           Serial.print(Kp);
           Serial.print(", Ki: ");
           Serial.print(Ki);
           Serial.print(", Kd: ");
           Serial.println(Kd);
           Serial.println("-------------------------");
         }
         
         // 輸出 Teleplot 格式的數據 (為pid_tuner.py提供)
         if (DEBUG_LEVEL >= 2) {
           Serial.print(">target_rpm:");
           Serial.println(targetRPM);
           Serial.print(">current_rpm:");
           Serial.println(currentRPM);
           Serial.print(">error:");
           Serial.println(targetRPM - currentRPM);
           Serial.print(">motor_output:");
           Serial.println(motorOutput);
         }
         
         // 釋放互斥鎖
         xSemaphoreGive(dataMutex);
       }
     }
   }
 }
 
 /**
  * 按鈕處理任務 - 處理按鈕輸入
  * 頻率: 50Hz (20ms)
  */
 void buttonHandlingTask(void *pvParameters) {
   TickType_t xLastWakeTime;
   const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
   bool buttonState, paramButtonState;
   
   xLastWakeTime = xTaskGetTickCount();
   
   if (DEBUG_LEVEL >= 1) {
     Serial.println("按鈕處理任務已啟動");
   }
   
   while (1) {
     vTaskDelayUntil(&xLastWakeTime, xFrequency);
     
     // 讀取按鈕狀態
     buttonState = digitalRead(BUTTON_PIN);
     paramButtonState = digitalRead(PARAM_BUTTON_PIN);
     
     // BOOT按鈕處理 (用於增加目標RPM)
     if (buttonState != lastButtonState) {
       lastDebounceTime = millis();
     }
     
     if ((millis() - lastDebounceTime) > debounceDelay) {
       if (buttonState == LOW && lastButtonState == HIGH) {
         // 獲取互斥鎖
         if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
           // BOOT按鈕被按下，增加目標RPM
           targetRPM += 50;
           if (targetRPM > 300) {  // 設置上限為300 RPM
             targetRPM = 50;  // 超過上限後重置為初始值
           }
           
           if (DEBUG_LEVEL >= 1) {
             Serial.print("目標RPM設置為: ");
             Serial.println(targetRPM);
           }
           
           // 釋放互斥鎖
           xSemaphoreGive(dataMutex);
           
           // 顯示新的目標RPM
           char buffer[20];
           sprintf(buffer, "Target: %d RPM", (int)targetRPM);
           oled.displayMessage("RPM Changed", buffer, 1000);
         }
       }
     }
     
     lastButtonState = buttonState;
     
     // 參數按鈕處理
     if (paramButtonState != lastParamButtonState) {
       lastParamDebounceTime = millis();
     }
     
     if ((millis() - lastParamDebounceTime) > debounceDelay) {
       if (paramButtonState == LOW && lastParamButtonState == HIGH) {
         // 參數按鈕被按下，切換調試頁面的參數調整模式
         debugPage.nextParamMode();
         
         if (DEBUG_LEVEL >= 1) {
           Serial.print("參數調整模式切換到: ");
           Serial.println(debugPage.getParamMode());
         }
       }
     }
     
     lastParamButtonState = paramButtonState;
   }
 }
 
 /**
  * 更新RPM歷史記錄
  */
 void updateRPMHistory(double rpm) {
   rpmHistory[historyIndex] = rpm;
   historyIndex = (historyIndex + 1) % GRAPH_POINTS;
 }
 
 /**
  * 處理串口命令
  */
 void processSerialCommands() {
   if (Serial.available() > 0) {
     String command = Serial.readStringUntil('\n');
     command.trim();
     
     if (command.startsWith("PID:")) {
       // 解析PID參數
       String params = command.substring(4);
       int firstComma = params.indexOf(',');
       int secondComma = params.indexOf(',', firstComma + 1);
       
       if (firstComma > 0 && secondComma > 0) {
         double newKp = params.substring(0, firstComma).toDouble();
         double newKi = params.substring(firstComma + 1, secondComma).toDouble();
         double newKd = params.substring(secondComma + 1).toDouble();
         
         // 獲取互斥鎖
         if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
           // 更新PID參數
           motorPID.SetTunings(newKp, newKi, newKd);
           Kp = newKp;
           Ki = newKi;
           Kd = newKd;
           
           // 釋放互斥鎖
           xSemaphoreGive(dataMutex);
           
           Serial.print(">status:PID參數已更新 Kp=");
           Serial.print(newKp);
           Serial.print(" Ki=");
           Serial.print(newKi);
           Serial.print(" Kd=");
           Serial.println(newKd);
         }
       }
     } else if (command.startsWith("RPM:")) {
       // 設置目標RPM
       int newRPM = command.substring(4).toInt();
       
       // 獲取互斥鎖
       if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
         targetRPM = newRPM;
         
         // 釋放互斥鎖
         xSemaphoreGive(dataMutex);
         
         Serial.print(">status:目標RPM已設為 ");
         Serial.println(newRPM);
       }
     }
   }
 }
 
 void setup() {
   // 初始化串口
   Serial.begin(115200);
   delay(1000);  // 給串口一些時間初始化
   Serial.println("\n=== 馬達 PID 控制測試 (RTOS版本) ===");
   
   // 創建互斥鎖
   dataMutex = xSemaphoreCreateMutex();
   if (dataMutex == NULL) {
     Serial.println("互斥鎖創建失敗!");
     while(1); // 嚴重錯誤，停止執行
   }
   
   // 初始化 OLED
   if (DEBUG_LEVEL >= 1) Serial.println("初始化 OLED 顯示器...");
   if (!oled.begin(I2C_SDA, I2C_SCL)) {
     Serial.println("OLED 初始化失敗!");
     while (1) {
       delay(1000);  // 如果 OLED 初始化失敗，停止執行
     }
   }
   
   // 顯示歡迎訊息
   oled.displayMessage("PID Motor Test", "RTOS Version", 1000);
   
   // 添加調試頁面
   if (DEBUG_LEVEL >= 1) Serial.println("添加頁面到 OLED 管理器...");
   if (!oled.addPage(&debugPage)) {
     Serial.println("添加調試頁面失敗!");
   }
   
   // 初始化馬達
   if (DEBUG_LEVEL >= 1) Serial.println("初始化馬達...");
   motor1.begin();
   motor2.begin();
   
   // 初始化編碼器
   if (DEBUG_LEVEL >= 1) Serial.println("初始化編碼器...");
   encoder1.begin(0);
   encoder2.begin(1);
   
   // 設定編碼器方向反轉（如果需要）
   encoder1.setInverted(false);
   encoder2.setInverted(true);  // 反轉 encoder2 的方向讀數
   
   // 設置按鈕引腳
   if (DEBUG_LEVEL >= 1) Serial.println("設置按鈕引腳...");
   pinMode(BUTTON_PIN, INPUT_PULLUP);
   pinMode(PARAM_BUTTON_PIN, INPUT_PULLUP);
   
   // 初始化 PID
   motorPID.SetMode(AUTOMATIC);
   motorPID.SetOutputLimits(0, 255);  // 輸出限制在 0-255 之間
   motorPID.SetSampleTime(10);  // 設置 PID 計算間隔為 10ms
   
   // 等待 BOOT 按鈕按下後才開始
   if (DEBUG_LEVEL >= 1) Serial.println("等待 BOOT 按鈕按下...");
   oled.displayMessage("Press BOOT", "to start motors");
   
   // 等待按鈕按下
   while (digitalRead(BUTTON_PIN) == HIGH) {
     delay(10);
   }
   delay(100);  // 防抖動
   if (DEBUG_LEVEL >= 1) Serial.println("BOOT 按鈕已按下");
   
   // 啟動馬達
   motor1.setRunning(true);
   motor2.setRunning(true);
   motorsEnabled = true;
   
   if (DEBUG_LEVEL >= 1) Serial.println(">status:馬達已啟動運行");
   oled.displayMessage("Motors", "Started!", 1000);
   
   // 設置初始目標 RPM
   targetRPM = 50;
   
   // 創建RTOS任務
   xTaskCreatePinnedToCore(
     pidControlTask,           // 任務函數
     "PID Control",            // 任務名稱
     STACK_SIZE,               // 堆疊大小
     NULL,                     // 參數
     PID_TASK_PRIORITY,        // 優先級
     &pidTaskHandle,           // 任務句柄
     1                         // 在核心1上執行
   );
   
   xTaskCreatePinnedToCore(
     sensorReadTask,           // 任務函數
     "Sensor Read",            // 任務名稱
     STACK_SIZE,               // 堆疊大小
     NULL,                     // 參數
     SENSOR_TASK_PRIORITY,     // 優先級
     &sensorTaskHandle,        // 任務句柄
     1                         // 在核心1上執行
   );
   
   xTaskCreatePinnedToCore(
     displayUpdateTask,        // 任務函數
     "Display Update",         // 任務名稱
     STACK_SIZE,               // 堆疊大小
     NULL,                     // 參數
     DISPLAY_TASK_PRIORITY,    // 優先級
     &displayTaskHandle,       // 任務句柄
     0                         // 在核心0上執行
   );
   
   xTaskCreatePinnedToCore(
     serialCommunicationTask,  // 任務函數
     "Serial Comm",            // 任務名稱
     STACK_SIZE,               // 堆疊大小
     NULL,                     // 參數
     SERIAL_TASK_PRIORITY,     // 優先級
     &serialTaskHandle,        // 任務句柄
     0                         // 在核心0上執行
   );
   
   xTaskCreatePinnedToCore(
     buttonHandlingTask,       // 任務函數
     "Button Handling",        // 任務名稱
     STACK_SIZE,               // 堆疊大小
     NULL,                     // 參數
     BUTTON_TASK_PRIORITY,     // 優先級
     &buttonTaskHandle,        // 任務句柄
     0                         // 在核心0上執行
   );
   
   if (DEBUG_LEVEL >= 1) Serial.println("所有任務已創建，開始RTOS調度");
 }
 
 void loop() {
   // 主循環不需要做任何事情，所有工作都由RTOS任務處理
   delay(1000);
   
   // 可以在這裡監控系統狀態，例如堆棧使用情況
   if (DEBUG_LEVEL >= 1) {
     Serial.print("空閒堆棧大小 (字節): Core0=");
     Serial.print(uxTaskGetStackHighWaterMark(NULL));
     
     if (pidTaskHandle) {
       Serial.print(", PID任務=");
       Serial.print(uxTaskGetStackHighWaterMark(pidTaskHandle));
     }
     
     if (serialTaskHandle) {
       Serial.print(", 串口任務=");
       Serial.print(uxTaskGetStackHighWaterMark(serialTaskHandle));
     }
     
     Serial.println();
   }
 }