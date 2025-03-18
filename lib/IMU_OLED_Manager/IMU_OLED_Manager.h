/**
 * IMU_OLED_Manager.h
 * 整合MPU6050與SH1106 OLED顯示的管理庫
 * 
 * 功能概述:
 * - 將MPU6050慣性測量單元與SH1106 OLED顯示器整合在一個易用的類別中
 * - 提供多種資料顯示模式: YPR (偏航、俯仰、翻滾)、加速度/陀螺儀數據、校準值、自定義數據
 * - 簡化感測器的初始化、校準和資料獲取流程
 * - 提供簡單的界面用於顯示和切換不同數據視圖
 * - 支持校準數據的儲存與載入，使用ESP32的Preferences庫
 * 
 * 使用方法:
 * 1. 建立實例: IMU_OLED_Manager imuDisplay;
 * 2. 初始化: imuDisplay.begin(SDA_PIN, SCL_PIN);
 * 3. 在loop中更新: imuDisplay.update();
 * 4. 切換顯示模式: imuDisplay.nextDisplayMode() 或 imuDisplay.setDisplayMode(mode);
 * 5. 校準: imuDisplay.calibrateMPU(samples); // samples為校準樣本數
 * 6. 獲取數據: imuDisplay.getYPR(ypr), imuDisplay.getYaw(), imuDisplay.getPitch(), imuDisplay.getRoll()
 * 7. 自定義顯示: imuDisplay.displayCustomData("標題", "數據1", "數據2", "數據3");
 * 8. 顯示訊息: imuDisplay.displayMessage("第一行", "第二行");
 * 
 * 範例用法參見 IMU_OLED_Manager_Demo.cpp
 */

 #ifndef IMU_OLED_MANAGER_H
 #define IMU_OLED_MANAGER_H
 
 #include <Arduino.h>
 #include <Wire.h>
 #include <U8g2lib.h>
 #include "MPU6050_6Axis_MotionApps612.h"
 #include "config.h"
 #include <Preferences.h>
 
 /**
  * 顯示模式枚舉
  * MODE_YPR - 顯示偏航、俯仰、翻滾角度，適合平衡控制
  * MODE_ACCEL_GYRO - 顯示加速度計和陀螺儀原始數據，適合診斷感測器
  * MODE_CALIBRATION_VALUES - 顯示校準偏移值，便於檢查校準狀態
  * MODE_CUSTOM_DATA - 顯示自定義數據（例如：電池電壓、運行時間等）
  */
 enum DisplayMode {
     MODE_YPR,                // 顯示偏航、俯仰、翻滾角度
     MODE_ACCEL_GYRO,         // 顯示加速度計和陀螺儀數據
     MODE_CALIBRATION_VALUES, // 顯示校準值
     MODE_CUSTOM_DATA,        // 自定義數據顯示
     MODE_COUNT               // 模式總數（用於循環切換）
 };
 
 class IMU_OLED_Manager {
 private:
     // MPU6050 相關變數
     MPU6050 mpu;
     bool dmpReady;
     uint8_t mpuIntStatus;
     uint8_t devStatus;
     uint16_t packetSize;
     uint16_t fifoCount;
     uint8_t fifoBuffer[64];
     
     // 方向/運動變數
     Quaternion q;
     VectorFloat gravity;
     float ypr[3];
     
     // 校準偏移值
     int16_t ax_offset, ay_offset, az_offset;
     int16_t gx_offset, gy_offset, gz_offset;
     
     // OLED顯示物件
     U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
     
     // 顯示更新相關
     unsigned long lastDisplayUpdate;
     unsigned long displayUpdateInterval;
     
     // 顯示模式
     DisplayMode currentMode;
     
     // Preferences對象，用於存儲校準值
     Preferences preferences;
     
     // 偏好設置的命名空間
     const char* prefsNamespace = "imu_cal";
     
     // 私有方法
     void drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress);
     void drawCalibrationScreen(const char* message, int progress);
     void drawSplashScreen();
     
 public:
     /**
      * 建構函數
      * 初始化所有成員變數到默認狀態
      */
     IMU_OLED_Manager();
     
     /**
      * 初始化IMU和OLED
      * @param sda I2C SDA腳位
      * @param scl I2C SCL腳位
      * @return 初始化成功返回true，失敗返回false
      */
     bool begin(int sda, int scl);
     
     /**
      * 初始化MPU6050
      * @return 初始化成功返回true，失敗返回false
      */
     bool initMPU();
     
     /**
      * 校準MPU6050
      * @param samples 校準樣本數量，越多越精確但耗時越長
      * @return 校準成功返回true
      */
     bool calibrateMPU(uint8_t samples = 6);
     
     /**
      * 從Preferences載入儲存的校準值
      * @return 存在有效校準數據時返回true
      */
     bool loadCalibration();
     
     /**
      * 將校準值儲存到Preferences
      * @return 儲存成功返回true
      */
     bool saveCalibration();
     
     /**
      * 初始化OLED顯示器
      * @return 初始化成功返回true
      */
     bool initDisplay();
     
     /**
      * 顯示YPR (偏航、俯仰、翻滾) 數據
      */
     void displayYPR();
     
     /**
      * 顯示加速度計和陀螺儀數據
      */
     void displayAccelGyro();
     
     /**
      * 顯示校準值
      */
     void displayCalibrationValues();
     
     /**
      * 顯示自定義數據
      * @param title 標題
      * @param data1 第一行數據
      * @param data2 第二行數據（可選）
      * @param data3 第三行數據（可選）
      */
     void displayCustomData(const char* title, const char* data1, const char* data2 = NULL, const char* data3 = NULL);
     
     /**
      * 顯示一般訊息
      * @param line1 第一行訊息
      * @param line2 第二行訊息（可選）
      */
     void displayMessage(const char* line1, const char* line2 = NULL);
     
     /**
      * 設置顯示模式
      * @param mode 要設置的顯示模式
      */
     void setDisplayMode(DisplayMode mode);
     
     /**
      * 獲取目前顯示模式
      * @return 當前顯示模式
      */
     DisplayMode getDisplayMode();
     
     /**
      * 切換到下一個顯示模式
      */
     void nextDisplayMode();
     
     /**
      * 獲取YPR數據
      * @param yawPitchRoll 用於儲存結果的數組，按順序為偏航、俯仰、翻滾角度（弧度）
      * @return 獲取成功返回true
      */
     bool getYPR(float* yawPitchRoll);
     
     /**
      * 獲取偏航角（Yaw）
      * @return 偏航角（弧度）
      */
     float getYaw();
     
     /**
      * 獲取俯仰角（Pitch）
      * @return 俯仰角（弧度）
      */
     float getPitch();
     
     /**
      * 獲取翻滾角（Roll）
      * @return 翻滾角（弧度）
      */
     float getRoll();
     
     /**
      * 更新函數 - 應在主程式的loop()中調用
      * 更新感測器數據並根據當前模式更新顯示
      */
     void update();
 };
 
 #endif // IMU_OLED_MANAGER_H