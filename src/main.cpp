#include "motor.h"

// 創建兩個馬達對象
Motor motor1(MOTOR1_PWM, MOTOR1_AIN1, MOTOR1_AIN2, MOTOR1_ENA, MOTOR1_ENB, MOTOR_STBY, "motor1", 11);
Motor motor2(MOTOR2_PWM, MOTOR2_AIN1, MOTOR2_AIN2, MOTOR2_ENA, MOTOR2_ENB, MOTOR_STBY, "motor2", 11);

void setup() {
  Serial.begin(115200);
  
  // 初始化馬達
  motor1.begin(0); // 第一個馬達實例
  motor2.begin(1); // 第二個馬達實例
  
  // 設定每圈的脈衝數，如果與預設值不同
  motor1.setPulsesPerRev(11);
  motor2.setPulsesPerRev(11);
  
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
  
  // 更新馬達狀態
  motor1.update();
  motor2.update();
  
  // 使用 Teleplot 格式輸出數據
  motor1.teleplotOutput();
  motor2.teleplotOutput();
  
  delay(100);
}