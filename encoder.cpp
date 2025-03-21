#include "encoder.h"

// Initialize static member
Encoder* Encoder::instances[2] = {nullptr, nullptr};

Encoder::Encoder(uint8_t pinA, uint8_t pinB, String name, int pulsesPerRev) {
    _pinA = pinA;
    _pinB = pinB;
    _name = name;
    _pulsesPerRev = pulsesPerRev;
    
    _pulseCount = 0;
    _rpm = 0.0;
    _lastTime = 0;
    _lastStateA = false;
    _lastStateB = false;
    _direction = STOPPED;
    _inverted = false;
    
    // Add low-pass filter for RPM smoothing
    _lastRPM = 0.0;
    // 修改濾波係數為0.9，提供更快的反應速度，但仍保持一定的平滑效果
    _filterCoef = 0.8; // 調整為0.9，較高的值提供更快的反應，但濾波效果較弱
}

void Encoder::begin(int encoderIndex) {
    // Setup encoder pins as inputs with pull-up resistors
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Initialize last states
    _lastStateA = digitalRead(_pinA);
    _lastStateB = digitalRead(_pinB);
    
    // Register this encoder instance for interrupt handling
    if (encoderIndex >= 0 && encoderIndex < 2) {
        instances[encoderIndex] = this;
        
        // Attach interrupts for encoder pins
        if (encoderIndex == 0) {
            attachInterrupt(digitalPinToInterrupt(_pinA), encoderISR0, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_pinB), encoderISR0, CHANGE);
        } else if (encoderIndex == 1) {
            attachInterrupt(digitalPinToInterrupt(_pinA), encoderISR1, CHANGE);
            attachInterrupt(digitalPinToInterrupt(_pinB), encoderISR1, CHANGE);
        }
    }
    
    // 使用micros()代替millis()以獲得更高精度的時間測量
    _lastTime = micros();
}

void Encoder::setPulsesPerRev(int pulsesPerRev) {
    _pulsesPerRev = pulsesPerRev;
}

void Encoder::setInverted(bool inverted) {
    _inverted = inverted;
}

bool Encoder::isInverted() const {
    return _inverted;
}

float Encoder::getRPM() const {
    return _rpm;
}

long Encoder::getPulseCount() const {
    return _pulseCount;
}

EncoderDirection Encoder::getDirection() const {
    return _direction;
}

String Encoder::getName() const {
    return _name;
}

void Encoder::resetPulseCount() {
    _pulseCount = 0;
}

void Encoder::update() {
    // 使用micros()代替millis()以獲得更高精度的時間測量
    unsigned long currentTime = micros();
    unsigned long timeElapsed = currentTime - _lastTime;
    
    // 將時間間隔從毫秒轉換為微秒 (10ms = 10000微秒)
    if (timeElapsed >= 10000) {
        // Save the current pulse count to a local variable
        volatile long currentPulseCount = _pulseCount;
        
        // 調整計算公式以適應微秒計時
        // 原公式: (pulses / pulses_per_rev) * (60000 / timeElapsed) [毫秒]
        // 新公式: (pulses / pulses_per_rev) * (60000000 / timeElapsed) [微秒]
        float instantRPM = (float)(abs(currentPulseCount) * 60000000.0) / (_pulsesPerRev * timeElapsed);
        
        // Apply low-pass filter for smoothing with new coefficient 0.9
        _rpm = _filterCoef * instantRPM + (1.0 - _filterCoef) * _lastRPM;
        _lastRPM = _rpm;
        
        // Set direction based on pulse count, considering inversion
        if (currentPulseCount == 0) {
            _direction = STOPPED;
        } else {
            // Apply inversion if needed
            bool isForward = (currentPulseCount > 0);
            if (_inverted) {
                isForward = !isForward;
            }
            
            _direction = isForward ? FORWARD : BACKWARD;
        }

        // Reset the pulse count and update the last time
        _pulseCount = 0;
        _lastTime = currentTime;
        
        // Debug output for troubleshooting
        Serial.print(">"); Serial.print(_name); Serial.print("_pulses:");
        Serial.println(currentPulseCount);
        Serial.print(">"); Serial.print(_name); Serial.print("_timeElapsed_us:");
        Serial.println(timeElapsed);
    }
}

void Encoder::teleplotOutput() const {
    Serial.print(">"); Serial.print(_name); Serial.print("_rpm:");
    Serial.println(_rpm);
    
    Serial.print(">"); Serial.print(_name); Serial.print("_direction:");
    Serial.println(static_cast<int>(_direction));
}

void Encoder::handleEncoderInterrupt() {
    // Read current states
    bool stateA = digitalRead(_pinA);
    bool stateB = digitalRead(_pinB);
    
    // Quadrature decoding logic
    if (stateA != _lastStateA || stateB != _lastStateB) {
        // Determine direction based on the sequence of signals
        if (stateA != _lastStateA) {
            if (stateA == stateB) {
                _pulseCount--;
            } else {
                _pulseCount++;
            }
        } else { // stateB changed
            if (stateA == stateB) {
                _pulseCount++;
            } else {
                _pulseCount--;
            }
        }
        
        _lastStateA = stateA;
        _lastStateB = stateB;
    }
}

void Encoder::encoderISR0() {
    if (instances[0] != nullptr) {
        instances[0]->handleEncoderInterrupt();
    }
}

void Encoder::encoderISR1() {
    if (instances[1] != nullptr) {
        instances[1]->handleEncoderInterrupt();
    }
}