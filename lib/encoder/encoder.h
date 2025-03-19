#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Direction enum for standardized direction values
enum EncoderDirection {
    BACKWARD = -1,
    STOPPED = 0,
    FORWARD = 1
};

class Encoder {
private:
    uint8_t _pinA;        // Encoder channel A pin
    uint8_t _pinB;        // Encoder channel B pin
    String _name;         // Encoder name for identification
    
    volatile long _pulseCount;    // Encoder pulse count
    int _pulsesPerRev;    // Pulses per revolution for encoder
    float _rpm;           // Calculated RPM
    float _lastRPM;       // Previous RPM for filtering
    float _filterCoef;    // Low-pass filter coefficient
    unsigned long _lastTime;      // Last time speed was calculated
    
    // For direction detection
    volatile bool _lastStateA;
    volatile bool _lastStateB;
    EncoderDirection _direction;  // Direction using the enum
    
    bool _inverted;       // Whether to invert the direction reading
    
public:
    Encoder(uint8_t pinA, uint8_t pinB, String name, int pulsesPerRev = 11);
    
    // Setup functions
    void begin(int encoderIndex);
    void setPulsesPerRev(int pulsesPerRev);
    
    // Direction control
    void setInverted(bool inverted);
    bool isInverted() const;
    
    // Status functions
    float getRPM() const;
    long getPulseCount() const;
    EncoderDirection getDirection() const;
    void resetPulseCount();
    String getName() const;
    
    // Update function to be called in the loop
    void update();
    
    // Utility functions
    void teleplotOutput() const;
    
    // Encoder interrupt handlers
    void handleEncoderInterrupt();
    
    // Static function to handle encoder interrupts
    static void encoderISR0();
    static void encoderISR1();
    
    // Pointer to encoder instances for ISR
    static Encoder* instances[2];
};

#endif // ENCODER_H