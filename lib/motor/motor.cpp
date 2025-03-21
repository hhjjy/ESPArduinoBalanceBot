#include "motor.h"

Motor::Motor(uint8_t pwmPin, uint8_t ain1Pin, uint8_t ain2Pin, uint8_t stbyPin, String name) {
    _pwmPin = pwmPin;
    _ain1Pin = ain1Pin;
    _ain2Pin = ain2Pin;
    _stbyPin = stbyPin;
    _name = name;
    
    _speed = 0;
    _isRunning = false;
}

void Motor::begin() {
    // Setup TB6612 control pins
    pinMode(_pwmPin, OUTPUT);
    pinMode(_ain1Pin, OUTPUT);
    pinMode(_ain2Pin, OUTPUT);
    pinMode(_stbyPin, OUTPUT);
    
    // Set initial state
    digitalWrite(_stbyPin, HIGH);  // Disable standby
    analogWrite(_pwmPin, 0);       // Set speed to 0
    digitalWrite(_ain1Pin, LOW);   // Set direction
    digitalWrite(_ain2Pin, LOW);   // Set direction
}

void Motor::setSpeed(int speed) {
    // Constrain speed to valid range
    speed = constrain(speed, -255, 255);
    _speed = speed;
    
    if (!_isRunning) {
        return;
    }
    
    // Set direction based on speed sign
    if (speed > 0) {
        // 正向旋轉
        digitalWrite(_ain1Pin, HIGH);
        digitalWrite(_ain2Pin, LOW);
        analogWrite(_pwmPin, speed);
    } else if (speed < 0) {
        // 反向旋轉
        digitalWrite(_ain1Pin, LOW);
        digitalWrite(_ain2Pin, HIGH);
        analogWrite(_pwmPin, -speed);
    } else {
        // 速度為0時，將兩個方向引腳都設為低電平，實現滑行停止
        digitalWrite(_ain1Pin, LOW);
        digitalWrite(_ain2Pin, LOW);
        analogWrite(_pwmPin, 0);
    }
}
void Motor::setRunning(bool isRunning) {
    _isRunning = isRunning;
    
    if (isRunning) {
        digitalWrite(_stbyPin, HIGH);  // Disable standby
        setSpeed(_speed);              // Apply current speed
    } else {
        stop();
    }
}

void Motor::stop() {
    analogWrite(_pwmPin, 0);
    digitalWrite(_ain1Pin, LOW);
    digitalWrite(_ain2Pin, LOW);
}

void Motor::brake() {
    analogWrite(_pwmPin, 255);
    digitalWrite(_ain1Pin, LOW);
    digitalWrite(_ain2Pin, LOW);
}

void Motor::coast() {
    analogWrite(_pwmPin, 0);
    digitalWrite(_stbyPin, LOW);  // Enable standby
}

int Motor::getSpeed() const {
    return _speed;
}

String Motor::getName() const {
    return _name;
}

void Motor::teleplotOutput() const {
    Serial.print(">"); Serial.print(_name); Serial.print("_speed:");
    Serial.println(_speed);
}

void Motor::waitForBootButton() {
    // Assuming BOOT button is connected to GPIO 0
    pinMode(0, INPUT_PULLUP);
    Serial.println(">status:等待按下 BOOT 按鈕...");
    while (digitalRead(0) == HIGH) {
        delay(100);
    }
    delay(100);  // Debounce
}