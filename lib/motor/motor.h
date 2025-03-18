#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    // TB6612 control pins
    uint8_t _pwmPin;      // PWM speed control
    uint8_t _ain1Pin;     // Direction control 1
    uint8_t _ain2Pin;     // Direction control 2
    uint8_t _stbyPin;     // Standby pin
    
    // Motor properties
    String _name;         // Motor name for identification
    int _speed;           // Current speed setting (-255 to 255)
    bool _isRunning;      // Motor running state
    
public:
    Motor(uint8_t pwmPin, uint8_t ain1Pin, uint8_t ain2Pin, uint8_t stbyPin, String name);
    
    // Setup functions
    void begin();
    
    // Control functions
    void setSpeed(int speed);
    void setRunning(bool isRunning);
    void stop();
    void brake();
    void coast();
    
    // Status functions
    int getSpeed() const;
    String getName() const;
    
    // Utility functions
    void teleplotOutput() const;
    static void waitForBootButton();
};

#endif // MOTOR_H