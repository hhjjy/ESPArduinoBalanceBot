/**
 * PID_Motor_Control_Test.cpp
 * 
 * 用於測試馬達PID控制內環參數的程式
 * 功能：
 * 1. 顯示目標RPM和當前RPM的示波器視圖
 * 2. 按下Boot按鈕增加目標RPM（每次增加50 RPM）
 * 3. 使用PID控制器調整馬達速度以達到目標RPM
 * 4. 可以調整PID參數
 */

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "OLED_Manager.h"
#include "pages/DebugPage.h"
#include "config.h"
#include "PID_v1.h"

// 調試控制標誌
#define DEBUG_LEVEL 1  // 0: 無調試輸出, 1: 基本調試, 2: 詳細調試, 3: 所有數據

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

void updateRPMHistory(double rpm) {
  rpmHistory[historyIndex] = rpm;
  historyIndex = (historyIndex + 1) % GRAPH_POINTS;
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // 給串口一些時間初始化
  Serial.println("\n=== 馬達 PID 控制測試 ===");
  
  // 初始化 OLED
  if (DEBUG_LEVEL >= 1) Serial.println("初始化 OLED 顯示器...");
  if (!oled.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("OLED 初始化失敗!");
    while (1) {
      delay(1000);  // 如果 OLED 初始化失敗，停止執行
    }
  }
  
  // 顯示歡迎訊息
  oled.displayMessage("PID Motor Test", "Starting...", 1000);
  
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
  motorPID.SetSampleTime(20);  // 設置 PID 計算間隔為 20ms
  
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
  
  if (DEBUG_LEVEL >= 1) Serial.println(">status:馬達已啟動運行");
  oled.displayMessage("Motors", "Started!", 1000);
  
  // 設置初始目標 RPM
  targetRPM = 50;
  
  if (DEBUG_LEVEL >= 1) Serial.println("設置完成，開始主循環");
}

void loop() {
  // 更新編碼器狀態
  encoder1.update();
  encoder2.update();
  
  // 計算當前 RPM (兩個馬達的平均值)
  currentRPM = (encoder1.getRPM() + encoder2.getRPM()) / 2.0;
  
  // 更新 RPM 歷史數據 (用於繪製示波器)
  updateRPMHistory(currentRPM);
  
  // 計算 PID
  motorPID.Compute();
  
  // 設置馬達速度
  motor1.setSpeed(motorOutput);
  motor2.setSpeed(motorOutput);
  
  // 讀取 BOOT 按鈕狀態 (用於增加目標 RPM)
  bool buttonState = digitalRead(BUTTON_PIN);
  
  // 按鈕防抖動
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // 如果按鈕狀態穩定且為按下狀態
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      // BOOT 按鈕被按下，增加目標 RPM
      targetRPM += 50;
      if (targetRPM > 300) {  // 設置上限為 300 RPM
        targetRPM = 50;  // 超過上限後重置為初始值
      }
      
      if (DEBUG_LEVEL >= 1) {
        Serial.print("目標 RPM 設置為: ");
        Serial.println(targetRPM);
      }
      
      // 顯示新的目標 RPM
      char buffer[20];
      sprintf(buffer, "Target: %d RPM", (int)targetRPM);
      oled.displayMessage("RPM Changed", buffer, 1000);
    }
  }
  
  lastButtonState = buttonState;
  
  // 讀取參數調整按鈕狀態
  bool paramButtonState = digitalRead(PARAM_BUTTON_PIN);
  
  // 參數按鈕防抖動
  if (paramButtonState != lastParamButtonState) {
    lastParamDebounceTime = millis();
  }
  
  // 如果參數按鈕狀態穩定且為按下狀態
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
  
  // 更新 OLED 顯示
  oled.update();
  
  // 定期輸出調試信息
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL && DEBUG_LEVEL >= 1) {
    lastDebugTime = currentTime;
    
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
    
    // 輸出 Teleplot 格式的數據
    if (DEBUG_LEVEL >= 2) {
      Serial.print(">target_rpm:");
      Serial.println(targetRPM);
      Serial.print(">current_rpm:");
      Serial.println(currentRPM);
      Serial.print(">motor_output:");
      Serial.println(motorOutput);
    }
    
    Serial.println("-------------------------");
  }
  
  delay(10);  // 更新頻率限制
}