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
    _direction = 0;
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
    
    _lastTime = millis();
}

void Encoder::setPulsesPerRev(int pulsesPerRev) {
    _pulsesPerRev = pulsesPerRev;
}

float Encoder::getRPM() const {
    return _rpm;
}

long Encoder::getPulseCount() const {
    return _pulseCount;
}

int8_t Encoder::getDirection() const {
    return _direction;
}

String Encoder::getName() const {
    return _name;
}

void Encoder::resetPulseCount() {
    _pulseCount = 0;
}

void Encoder::update() {
    // Calculate RPM based on pulse count
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - _lastTime;
    
    if (timeElapsed >= 100) {  // Update every 100ms
        // Calculate RPM: (pulses / pulses_per_rev) * (60000 / timeElapsed)
        _rpm = (float)(abs(_pulseCount) * 600) / (_pulsesPerRev * timeElapsed);
        
        // Set direction to 0 if not moving
        if (_pulseCount == 0) {
            _direction = 0;
        } else {
            _direction = (_pulseCount > 0) ? 1 : -1;
        }
        
        _lastTime = currentTime;
        _pulseCount = 0;  // Reset pulse count for next calculation
    }
}

void Encoder::teleplotOutput() const {
    Serial.print(">"); Serial.print(_name); Serial.print("_rpm:");
    Serial.println(_rpm);
    
    Serial.print(">"); Serial.print(_name); Serial.print("_direction:");
    Serial.println(_direction);
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