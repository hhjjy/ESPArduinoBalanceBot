#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

// 定義 BOOT 按鈕的引腳
#define BOOT_PIN 0  // 通常 ESP32 的 BOOT 按鈕在 GPIO0

class Motor {
  private:
    uint8_t pwmPin;       // 速度控制 PWM 腳位
    uint8_t in1Pin;       // 方向控制腳位 1
    uint8_t in2Pin;       // 方向控制腳位 2
    uint8_t encA;         // 霍爾感測器 A 腳位
    uint8_t encB;         // 霍爾感測器 B 腳位
    uint8_t stdbyPin;     // 待機控制腳位
    
    int16_t speed;        // 目前馬達速度
    int32_t position;     // 累計編碼器計數
    float rpm;            // 轉速 (每分鐘轉數)
    int32_t pulseCount;   // 脈衝計數（不重置）
    
    uint32_t lastTime;    // 用於計算轉速的時間變數
    uint16_t pulsesPerRev; // 每轉一圈的脈衝數
    String motorName;     // 馬達名稱，用於 Teleplot 輸出
    bool isRunning;       // 馬達是否已啟動運行
    
    // 霍爾感測器計數中斷使用的靜態方法和變數
    static Motor* instances[2];  // 馬達實例的靜態陣列
    static void encoderISR_0();  // 霍爾感測器中斷處理函數 0
    static void encoderISR_1();  // 霍爾感測器中斷處理函數 1
    void handleEncoder();        // 處理編碼器脈衝的方法
    
  public:
    /**
     * 馬達類別的建構函數
     * @param pwm PWM 速度控制腳位
     * @param in1 方向控制腳位 1
     * @param in2 方向控制腳位 2
     * @param encoderA 霍爾感測器 A 腳位
     * @param encoderB 霍爾感測器 B 腳位
     * @param stby 待機控制腳位
     * @param name 馬達名稱，用於 Teleplot 輸出
     * @param pulses 每轉一圈的脈衝數
     */
    Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t encoderA, uint8_t encoderB, uint8_t stby, 
          String name = "motor", uint16_t pulses = 11);
    
    /**
     * 初始化馬達
     * @param instanceNumber 馬達實例編號 (0 或 1)
     */
    void begin(uint8_t instanceNumber);
    
    /**
     * 設定馬達速度和方向
     * @param motorSpeed 速度值 (-255 到 255)，負值表示反向轉動
     */
    void setSpeed(int16_t motorSpeed);
    
    /**
     * 停止馬達
     * @param brake 是否啟用煞車 (true) 或滑行 (false)
     */
    void stop(bool brake = true);
    
    /**
     * 更新馬達轉速計算
     * 應在主迴圈中定期呼叫
     */
    void update();
    
    /**
     * 獲取目前位置計數
     * @return 編碼器計數值
     */
    int32_t getPosition();
    
    /**
     * 獲取目前每分鐘轉數 (RPM)
     * @return 每分鐘轉數
     */
    float getRPM();
    
    /**
     * 獲取總脈衝計數
     * @return 總脈衝計數
     */
    int32_t getPulseCount();
    
    /**
     * 重設位置計數器
     */
    void resetPosition();
    
    /**
     * 設定每轉一圈的脈衝數
     * @param pulses 每轉一圈的脈衝數
     */
    void setPulsesPerRev(uint16_t pulses);
    
    /**
     * 獲取每轉一圈的脈衝數
     * @return 每轉一圈的脈衝數
     */
    uint16_t getPulsesPerRev();
    
    /**
     * 以 Teleplot 格式輸出馬達數據
     * 格式為: >馬達名稱_屬性:值
     */
    void teleplotOutput();
    
    /**
     * 設定馬達運行狀態
     * @param state 是否運行
     */
    void setRunning(bool state);
    
    /**
     * 獲取馬達是否正在運行
     * @return 運行狀態
     */
    bool getRunning();
    
    /**
     * 等待 BOOT 按鈕按下後開始
     * 靜態方法，用於所有馬達共用
     */
    static bool waitForBootButton();
};

#endif // MOTOR_H