#include "motor.h"
#include "encoder.h"
#include "config.h"
// 創建兩個馬達對象
Motor motor1(MOTOR1_PWM, MOTOR1_AIN1, MOTOR1_AIN2, MOTOR_STBY, "motor1");
Motor motor2(MOTOR2_PWM, MOTOR2_AIN1, MOTOR2_AIN2, MOTOR_STBY, "motor2");

// 創建兩個編碼器對象
Encoder encoder1(MOTOR1_ENA, MOTOR1_ENB, "encoder1", 440);
Encoder encoder2(MOTOR2_ENA, MOTOR2_ENB, "encoder2", 440);

void setup() {
  Serial.begin(115200);
  
  // 初始化馬達
  motor1.begin();
  motor2.begin();
  
  // 初始化編碼器
  encoder1.begin(0);
  encoder2.begin(1);
  
  
  // 等待 BOOT 按鈕按下後才開始
  Motor::waitForBootButton();
  
  // 按下 BOOT 按鈕後，啟動馬達
  motor1.setRunning(true);
  motor2.setRunning(true);
  
  Serial.println(">status:馬達已啟動運行");
}

void loop() {
  // 設定馬達速度 (-255 到 255)
  motor1.setSpeed(150);
  motor2.setSpeed(150);
  
  // 更新編碼器狀態
  encoder1.update();
  encoder2.update();
  
  // 使用 Teleplot 格式輸出數據
  motor1.teleplotOutput();
  motor2.teleplotOutput();
  encoder1.teleplotOutput();
  encoder2.teleplotOutput();
  
  delay(100);
}