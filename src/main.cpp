#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "IMU.h"
#include "OLED_Manager.h"
#include "pages/MotorPage.h"
#include "pages/IMUPage.h"
#include "config.h"

// 調試控制標誌
#define DEBUG_LEVEL 1  // 0: 無調試輸出, 1: 基本調試, 2: 詳細調試, 3: 所有數據
#define ENABLE_MOTORS true  // 設置為 false 可以在測試時禁用馬達

// 創建馬達對象
Motor motor1(MOTOR1_PWM, MOTOR1_AIN1, MOTOR1_AIN2, MOTOR_STBY, "motor1");
Motor motor2(MOTOR2_PWM, MOTOR2_AIN1, MOTOR2_AIN2, MOTOR_STBY, "motor2");

// 創建編碼器對象
Encoder encoder1(MOTOR1_ENA, MOTOR1_ENB, "encoder1", 440);
Encoder encoder2(MOTOR2_ENA, MOTOR2_ENB, "encoder2", 440);

// 創建 IMU 對象
IMU imu;

// 創建 OLED 管理器
OLED_Manager oled;

// 創建馬達頁面
MotorPage motorPage(&motor1, &motor2, &encoder1, &encoder2);

// 創建 IMU 頁面
IMUPage imuPage(&imu);

// 按鈕定義
#define BUTTON_PIN 0  // BOOT 按鈕
#define CALIBRATE_BUTTON_PIN 1  // 假設校準按鈕連接到 GPIO1

// 按鈕狀態
bool lastButtonState = HIGH;
bool lastCalButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastCalDebounceTime = 0;
unsigned long debounceDelay = 50;

// 調試變量
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000;  // 每秒輸出一次調試信息

void setup() {
  Serial.begin(115200);
  delay(1000);  // 給串口一些時間初始化
  Serial.println("\n=== ESPArduinoBalanceBot 啟動 ===");
  
  // 初始化 OLED
  if (DEBUG_LEVEL >= 1) Serial.println("初始化 OLED 顯示器...");
  if (!oled.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("OLED 初始化失敗!");
    while (1) {
      delay(1000);  // 如果 OLED 初始化失敗，停止執行
    }
  }
  
  // 顯示歡迎訊息
  oled.displayMessage("Balance Bot", "Starting...", 1000);
  
  // 初始化 IMU
  if (DEBUG_LEVEL >= 1) Serial.println("初始化 MPU6050...");
  oled.displayMessage("Initializing", "MPU6050...");
  if (!imu.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("MPU6050 初始化失敗!");
    oled.displayMessage("MPU6050 Init", "Failed!");
    delay(2000);
  } else {
    if (DEBUG_LEVEL >= 1) Serial.println("MPU6050 初始化成功!");
    
    // 嘗試載入校準值
    if (imu.loadCalibration()) {
      if (DEBUG_LEVEL >= 1) Serial.println("已載入 IMU 校準數據");
      oled.displayMessage("IMU Calibration", "Loaded!", 1000);
    } else {
      if (DEBUG_LEVEL >= 1) Serial.println("未找到 IMU 校準數據");
      oled.displayMessage("No IMU Cal Data", "Found", 1000);
    }
  }
  
  // 添加頁面
  if (DEBUG_LEVEL >= 1) Serial.println("添加頁面到 OLED 管理器...");
  if (!oled.addPage(&motorPage)) {
    Serial.println("添加馬達頁面失敗!");
  }
  if (!oled.addPage(&imuPage)) {
    Serial.println("添加 IMU 頁面失敗!");
  }
  
  if (DEBUG_LEVEL >= 1) {
    Serial.println("已添加頁面:");
    Serial.print("0: ");
    Serial.println(motorPage.getName());
    Serial.print("1: ");
    Serial.println(imuPage.getName());
    Serial.print("頁面總數: ");
    Serial.println(oled.getPageCount());
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
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  
  // 設置初始頁面為馬達頁面（索引 0）
  if (DEBUG_LEVEL >= 1) Serial.println("設置初始頁面為馬達頁面");
  oled.setPage(0);
  if (DEBUG_LEVEL >= 1) {
    Serial.print("當前頁面索引: ");
    Serial.println(oled.getCurrentPageIndex());
  }
  
  if (ENABLE_MOTORS) {
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
  } else {
    if (DEBUG_LEVEL >= 1) Serial.println("馬達已禁用 (ENABLE_MOTORS = false)");
    oled.displayMessage("Motors", "Disabled", 1000);
  }
  
  if (DEBUG_LEVEL >= 1) Serial.println("設置完成，開始主循環");
}

void loop() {
  // 更新 IMU 數據
  imu.update();
  
  // 讀取頁面切換按鈕狀態
  bool buttonState = digitalRead(BUTTON_PIN);
  
  // 按鈕防抖動
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // 如果按鈕狀態穩定且為按下狀態
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      if (DEBUG_LEVEL >= 1) Serial.println("BOOT 按鈕被按下");
      
      // 檢查當前是否在 IMU 頁面
      if (oled.getCurrentPageIndex() == 1) {  // IMU 頁面是第二個頁面 (索引 1)
        // 在 IMU 頁面內切換子頁面
        if (DEBUG_LEVEL >= 1) Serial.println("在 IMU 頁面內切換子頁面");
        imuPage.handleButtonPress();
        if (DEBUG_LEVEL >= 1) {
          Serial.print("IMU 頁面模式切換到: ");
          Serial.println(imuPage.getDisplayMode());
        }
      } else {
        // 否則切換到下一個主頁面
        if (DEBUG_LEVEL >= 1) Serial.println("切換到下一個主頁面");
        oled.nextPage();
        if (DEBUG_LEVEL >= 1) {
          Serial.print("當前頁面索引: ");
          Serial.println(oled.getCurrentPageIndex());
        }
      }
    }
  }
  
  lastButtonState = buttonState;
  
  // 讀取校準按鈕狀態
  bool calButtonState = digitalRead(CALIBRATE_BUTTON_PIN);
  
  // 校準按鈕防抖動
  if (calButtonState != lastCalButtonState) {
    lastCalDebounceTime = millis();
  }
  
  // 如果校準按鈕狀態穩定且為按下狀態
  if ((millis() - lastCalDebounceTime) > debounceDelay) {
    if (calButtonState == LOW && lastCalButtonState == HIGH) {
      // 校準按鈕被按下，執行 IMU 校準
      if (DEBUG_LEVEL >= 1) Serial.println("校準按鈕被按下，開始 IMU 校準");
      oled.displayMessage("Calibrating", "Keep Device Still");
      delay(2000);
      
      // 執行校準，並使用 lambda 函數顯示進度
      imu.calibrate(6, [&](const char* message, int progress) {
        // 使用 OLED 顯示校準進度
        if (DEBUG_LEVEL >= 1) {
          Serial.print(message);
          Serial.print(": ");
          Serial.print(progress);
          Serial.println("%");
        }
        oled.displayProgress(message, progress);
      });
      
      if (DEBUG_LEVEL >= 1) Serial.println("IMU 校準完成");
      oled.displayMessage("Calibration", "Complete!", 1000);
    }
  }
  
  lastCalButtonState = calButtonState;
  
  if (ENABLE_MOTORS) {
    // 設定馬達速度 (-255 到 255)
    motor1.setSpeed(150);
    motor2.setSpeed(150);
  }
  
  // 更新編碼器狀態
  encoder1.update();
  encoder2.update();
  
  // 更新 OLED 顯示
  oled.update();
  
  // 定期輸出調試信息
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL && DEBUG_LEVEL >= 1) {
    lastDebugTime = currentTime;
    
    // 輸出當前頁面信息
    Serial.println("\n--- 系統狀態 ---");
    Serial.print("運行時間: ");
    Serial.print(currentTime / 1000);
    Serial.println(" 秒");
    Serial.print("當前頁面: ");
    Serial.print(oled.getCurrentPageIndex());
    Serial.print(" (");
    if (oled.getCurrentPageIndex() == 0) {
      Serial.print(motorPage.getName());
    } else if (oled.getCurrentPageIndex() == 1) {
      Serial.print(imuPage.getName());
      Serial.print(", 模式: ");
      Serial.print(imuPage.getDisplayMode());
    }
    Serial.println(")");
    
    // 只在詳細調試模式下輸出更多信息
    if (DEBUG_LEVEL >= 2) {
      // 輸出 IMU 數據
      Serial.print("IMU 狀態: ");
      Serial.print(imu.isInitialized() ? "已初始化" : "未初始化");
      Serial.print(", ");
      Serial.println(imu.isCalibrated() ? "已校準" : "未校準");
      
      Serial.print("姿態: Pitch=");
      Serial.print(imu.getPitch() * 180 / M_PI, 1);
      Serial.print("°, Roll=");
      Serial.print(imu.getRoll() * 180 / M_PI, 1);
      Serial.print("°, Yaw=");
      Serial.print(imu.getYaw() * 180 / M_PI, 1);
      Serial.println("°");
      
      // 輸出馬達數據
      Serial.print("馬達: M1=");
      Serial.print(motor1.getSpeed());
      Serial.print(", M2=");
      Serial.println(motor2.getSpeed());
      
      // 輸出編碼器數據
      Serial.print("編碼器: E1=");
      Serial.print(encoder1.getRPM(), 1);
      Serial.print(" RPM, E2=");
      Serial.print(encoder2.getRPM(), 1);
      Serial.println(" RPM");
    }
    
    Serial.println("----------------");
  }
  
  // 只在最高調試級別輸出 Teleplot 數據
  if (DEBUG_LEVEL >= 3) {
    // 使用 Teleplot 格式輸出數據
    motor1.teleplotOutput();
    motor2.teleplotOutput();
    encoder1.teleplotOutput();
    encoder2.teleplotOutput();
    
    // 輸出兩個馬達的平均 RPM
    float avgRPM = (encoder1.getRPM() + encoder2.getRPM()) / 2.0;
    Serial.print(">average_rpm:");
    Serial.println(avgRPM);
    
    // 輸出 IMU 數據
    Serial.print(">imu_pitch:");
    Serial.println(imu.getPitch() * 180 / M_PI);
    Serial.print(">imu_roll:");
    Serial.println(imu.getRoll() * 180 / M_PI);
  }
  
  delay(10);  // 更新頻率限制
}