# IMU_OLED_Manager 庫

這個庫提供了一個整合MPU6050慣性測量單元與SH1106 OLED顯示的解決方案，特別適用於平衡機器人項目。

## 功能特色

- **多種顯示模式**：提供多種數據視圖，包括偏航角/俯仰/翻滾(YPR)、加速度/陀螺儀數據、校準值和自定義數據
- **簡化的API**：簡單易用的函數封裝了複雜的感測器操作
- **自動校準與存儲**：支援MPU6050的校準並使用ESP32的Preferences庫儲存校準值
- **彈性的顯示系統**：提供多種顯示函數，支援自定義數據顯示

## 硬體需求

- ESP32/ESP32-S3開發板
- MPU6050慣性測量單元
- SH1106 OLED顯示器（128x64像素）
- 按鈕（用於模式切換/校準）

## 接線方式
ESP32 --- MPU6050/OLED SDA --- SDA SCL --- SCL 3.3V --- VCC GND --- GND

ESP32 --- MPU6050/OLED SDA --- SDA SCL --- SCL 3.3V --- VCC GND --- GND


按鈕接線可參考示範代碼。

## 使用方法

### 基本用法

```cpp
#include "IMU_OLED_Manager.h"

// 創建實例
IMU_OLED_Manager imuDisplay;

void setup() {
  // 初始化，傳入I2C的SDA和SCL腳位
  if (!imuDisplay.begin(5, 6)) {
    // 處理錯誤
  }
}

void loop() {
  // 更新顯示和感測器數據
  imuDisplay.update();
  
  // 獲取角度數據
  float ypr[3];
  if (imuDisplay.getYPR(ypr)) {
    // 使用ypr[0]（偏航角）、ypr[1]（俯仰角）和ypr[2]（翻滾角）
  }
  
  // 切換顯示模式
  imuDisplay.nextDisplayMode();
  
  // 顯示自定義數據
  imuDisplay.displayCustomData("標題", "數據1", "數據2", "數據3");
}