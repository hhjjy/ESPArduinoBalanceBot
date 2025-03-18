#include "motor.h"

// 初始化靜態成員
Motor* Motor::instances[2] = {nullptr, nullptr};

Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t encoderA, uint8_t encoderB, uint8_t stby, 
             String name, uint16_t pulses) {
  pwmPin = pwm;
  in1Pin = in1;
  in2Pin = in2;
  encA = encoderA;
  encB = encoderB;
  stdbyPin = stby;
  motorName = name;
  pulsesPerRev = pulses;
  
  speed = 0;
  position = 0;
  rpm = 0.0;
  pulseCount = 0;
  lastTime = 0;
  isRunning = false;
}

void Motor::begin(uint8_t instanceNumber) {
  // 設定馬達控制腳位
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(stdbyPin, OUTPUT);
  
  // 設定霍爾感測器腳位
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  
  // 啟用 TB6612 
  digitalWrite(stdbyPin, HIGH);
  
  // 初始化馬達狀態
  stop();
  
  // 設定中斷處理
  if (instanceNumber < 2) {
    instances[instanceNumber] = this;
    
    if (instanceNumber == 0) {
      attachInterrupt(digitalPinToInterrupt(encA), encoderISR_0, RISING);
    } else {
      attachInterrupt(digitalPinToInterrupt(encA), encoderISR_1, RISING);
    }
  }
}

void Motor::setSpeed(int16_t motorSpeed) {
  if (!isRunning) return; // 如果馬達未啟動，不執行任何操作
  
  speed = constrain(motorSpeed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    stop(true);
  }
}

void Motor::stop(bool brake) {
  speed = 0;
  
  if (brake) {
    // 煞車模式 - 兩個輸入都為高電平
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
  } else {
    // 滑行模式 - 兩個輸入都為低電平
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(pwmPin, 0);
}

void Motor::update() {
  if (!isRunning) return; // 如果馬達未啟動，不執行任何操作
  
  // 計算轉速 (RPM)
  uint32_t currentTime = millis();
  if (currentTime - lastTime > 100) { // 每 100ms 更新一次
    // 計算每分鐘轉數 = (脈衝數 / 每圈脈衝數) * (60000ms / 時間間隔ms)
    // 使用靜態變數來跟踪間隔之間的脈衝
    static int32_t lastPosition = 0;
    
    int32_t deltaPosition = position - lastPosition;
    float timeInterval = (currentTime - lastTime) / 1000.0; // 轉換為秒
    
    rpm = (deltaPosition / (float)pulsesPerRev) * (60.0 / timeInterval);
    
    lastPosition = position;
    lastTime = currentTime;
  }
}

int32_t Motor::getPosition() {
  return position;
}

float Motor::getRPM() {
  return rpm;
}

int32_t Motor::getPulseCount() {
  return pulseCount;
}

void Motor::resetPosition() {
  position = 0;
}

void Motor::setPulsesPerRev(uint16_t pulses) {
  pulsesPerRev = pulses;
}

uint16_t Motor::getPulsesPerRev() {
  return pulsesPerRev;
}

void Motor::setRunning(bool state) {
  isRunning = state;
  if (!state) {
    stop(true);
  }
}

bool Motor::getRunning() {
  return isRunning;
}

bool Motor::waitForBootButton() {
  // 設定 BOOT 按鈕引腳為輸入並啟用上拉電阻
  pinMode(BOOT_PIN, INPUT_PULLUP);
  
  Serial.println(">status:等待按下 BOOT 按鈕開始...");
  
  // 等待 BOOT 按鈕被按下（高電平變低電平）
  while (digitalRead(BOOT_PIN) == HIGH) {
    delay(50);
  }
  
  // 等待釋放按鈕以避免抖動
  delay(50);
  while (digitalRead(BOOT_PIN) == LOW) {
    delay(50);
  }
  
  Serial.println(">status:BOOT 按鈕已按下，開始運行！");
  return true;
}

void Motor::teleplotOutput() {
  // 以 Teleplot 格式輸出資料
  // 格式: >name:value
  Serial.print(">");
  Serial.print(motorName);
  Serial.print("_speed:");
  Serial.println(speed);
  
  Serial.print(">");
  Serial.print(motorName);
  Serial.print("_position:");
  Serial.println(position);
  
  Serial.print(">");
  Serial.print(motorName);
  Serial.print("_rpm:");
  Serial.println(rpm);
  
  Serial.print(">");
  Serial.print(motorName);
  Serial.print("_pulses:");
  Serial.println(pulseCount);
  
  Serial.print(">");
  Serial.print(motorName);
  Serial.print("_pulses_per_rev:");
  Serial.println(pulsesPerRev);
}

// 霍爾感測器中斷處理函數
void Motor::encoderISR_0() {
  if (instances[0] != nullptr) {
    instances[0]->handleEncoder();
  }
}

void Motor::encoderISR_1() {
  if (instances[1] != nullptr) {
    instances[1]->handleEncoder();
  }
}

void Motor::handleEncoder() {
  // 增加總脈衝計數
  pulseCount++;
  
  // 讀取霍爾感測器 B 的狀態，確定方向
  if (digitalRead(encB) == HIGH) {
    position++;
  } else {
    position--;
  }
}