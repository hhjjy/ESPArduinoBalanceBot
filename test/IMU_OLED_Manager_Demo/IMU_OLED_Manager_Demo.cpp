/**
 * IMU_OLED_Manager Demo
 * 
 * 此示範展示如何使用IMU_OLED_Manager類別:
 * - 使用單一按鈕控制不同功能 
 * - 短按: 切換顯示模式
 * - 長按: 進入校準模式
 * - 自定義數據顯示功能
 * 
 * 硬體需求:
 * - ESP32開發板
 * - MPU6050感應器
 * - SH1106 OLED顯示器
 * - 一個按鈕（連接到MODE_BUTTON_PIN）
 */

 #include <Arduino.h>
 #include "IMU_OLED_Manager.h"
 
 // 創建IMU_OLED_Manager實例
 IMU_OLED_Manager imuDisplay;
 
 // 用於切換顯示模式的按鈕
 #define MODE_BUTTON_PIN 0    // 使用ESP32的BOOT按鈕作為多功能按鈕
 
 // 按鈕處理變數
 bool buttonPressed = false;
 unsigned long buttonPressTime = 0;
 unsigned long lastButtonPress = 0;
 const unsigned long DEBOUNCE_DELAY = 300;  // 防抖延遲（毫秒）
 const unsigned long LONG_PRESS_TIME = 2000; // 長按判定時間（毫秒）
 
 void setup() {
   Serial.begin(115200);
   
   // 設置按鈕輸入
   pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
   
   // 初始化感測器與顯示器
   if (!imuDisplay.begin(I2C_SDA, I2C_SCL)) {
     Serial.println("初始化失敗！請檢查連接。");
     while (1) {
       delay(100);  // 無限循環
     }
   }
   
   Serial.println("初始化成功！");
   Serial.println("按鈕操作說明:");
   Serial.println("- 短按: 切換顯示模式");
   Serial.println("- 長按: 開始校準");
   
   // 顯示指導訊息
   imuDisplay.displayMessage("Button Control", "Short/Long Press");
   delay(2000);
 }
 
 void loop() {
   // 更新IMU和顯示
   imuDisplay.update();
   
   // 處理按鈕
   if (digitalRead(MODE_BUTTON_PIN) == LOW) {
     // 按鈕按下
     if (!buttonPressed) {
       // 按鈕初次按下
       buttonPressed = true;
       buttonPressTime = millis(); // 記錄按下時間
     }
     
     // 檢查是否達到長按時間且尚未執行長按動作
     if ((millis() - buttonPressTime > LONG_PRESS_TIME) && 
         (millis() - lastButtonPress > LONG_PRESS_TIME)) {
       // 長按動作 - 校準
       lastButtonPress = millis();
       
       Serial.println("長按偵測: 開始校準");
       
       // 顯示開始校準訊息
       imuDisplay.displayMessage("Starting", "Calibration");
       delay(1000);
       
       // 執行校準
       imuDisplay.calibrateMPU(6);  // 使用6個樣本進行校準
     }
   } 
   else if (buttonPressed) {
     // 按鈕釋放
     unsigned long pressDuration = millis() - buttonPressTime;
     
     // 短按動作 - 切換模式（只有在不是長按的情況下）
     if (pressDuration < LONG_PRESS_TIME && millis() - lastButtonPress > DEBOUNCE_DELAY) {
       lastButtonPress = millis();
       
       // 切換到下一個顯示模式
       imuDisplay.nextDisplayMode();
       
       // 獲取並打印當前模式
       DisplayMode mode = imuDisplay.getDisplayMode();
       Serial.print("短按偵測: 切換顯示模式: ");
       Serial.println(mode);
       
       // 顯示模式變更通知
       char modeText[20] = "Mode: ";
       switch (mode) {
         case MODE_YPR:
           strcat(modeText, "YPR");
           break;
         case MODE_ACCEL_GYRO:
           strcat(modeText, "Accel/Gyro");
           break;
         case MODE_CALIBRATION_VALUES:
           strcat(modeText, "Calib Values");
           break;
         case MODE_CUSTOM_DATA:
           strcat(modeText, "Custom");
           break;
       }
       
       imuDisplay.displayMessage("Mode Changed", modeText);
       delay(1000);  // 短暫顯示模式變更通知
     }
     
     buttonPressed = false;
   }
   
   // 如果是自定義數據模式，更新一些示範數據
   if (imuDisplay.getDisplayMode() == MODE_CUSTOM_DATA) {
     char data1[20], data2[20], data3[20];
     
     // 取得電池電壓（示範）
     float batteryVoltage = analogRead(34) * 2 * 3.3 / 4095; // 假設有分壓器
     snprintf(data1, sizeof(data1), "Battery: %.2fV", batteryVoltage);
     
     // 運行時間
     unsigned long runtime = millis() / 1000;
     snprintf(data2, sizeof(data2), "Runtime: %lus", runtime);
     
     // WiFi狀態（示範）
     snprintf(data3, sizeof(data3), "WiFi: Connected");
     
     imuDisplay.displayCustomData("System Info", data1, data2, data3);
     delay(100);  // 減少更新頻率
   }
   
   // 輸出YPR數據到串口（僅在YPR模式）
   if (imuDisplay.getDisplayMode() == MODE_YPR) {
     float ypr[3];
     if (imuDisplay.getYPR(ypr)) {
       Serial.print("Yaw: ");
       Serial.print(ypr[0] * 180 / M_PI);
       Serial.print(", Pitch: ");
       Serial.print(ypr[1] * 180 / M_PI);
       Serial.print(", Roll: ");
       Serial.println(ypr[2] * 180 / M_PI);
     }
   }
   
   delay(20);  // 短暫延遲避免CPU過載
 }