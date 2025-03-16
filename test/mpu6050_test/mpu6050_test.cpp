// MPU6050 Calibration and Basic Reading
// Simplified from Jeff Rowberg's original code

#include "I2Cdev.h"
#include "config.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU6050 class instance
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // onboard LED

// MPU control/status vars
bool dmpReady = false;     // set true if DMP init was successful
uint8_t mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t devStatus;         // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;        // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];    // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

// Calibration offset values
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL, 400000);  // Set I2C clock to 400kHz


  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect

  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Wait for user input to begin calibration
  Serial.println(F("\nSend any character to begin calibration: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // Initialize DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Print original values before calibration
  Serial.println(F("Original sensor offsets before calibration:"));
  Serial.print(F("accel: "));
  Serial.print(mpu.getXAccelOffset()); Serial.print("\t");
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t");
  Serial.println(mpu.getZAccelOffset());
  Serial.print(F("gyro: "));
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t");
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t");
  Serial.println(mpu.getZGyroOffset());
  Serial.println();

  // Proceed if DMP initialization was successful
  if (devStatus == 0) {
    // Perform calibration
    Serial.println(F("Performing calibration..."));
    Serial.println(F("KEEP DEVICE STILL ON A FLAT SURFACE"));
    mpu.CalibrateAccel(6);  // 6 iterations for accuracy
    mpu.CalibrateGyro(6);   // 6 iterations for accuracy
    Serial.println();
    
    // Print calculated offsets
    Serial.println(F("Calculated sensor offsets:"));
    mpu.PrintActiveOffsets();
    
    // Save calibration offsets to variables
    ax_offset = mpu.getXAccelOffset();
    ay_offset = mpu.getYAccelOffset();
    az_offset = mpu.getZAccelOffset();
    gx_offset = mpu.getXGyroOffset();
    gy_offset = mpu.getYGyroOffset();
    gz_offset = mpu.getZGyroOffset();
    
    // Print the final calibration values clearly
    Serial.println(F("\nFinal calibrated offsets (STATIC ERROR VALUES):"));
    Serial.print(F("accel offsets (X,Y,Z): ")); 
    Serial.print(ax_offset); Serial.print(", ");
    Serial.print(ay_offset); Serial.print(", ");
    Serial.println(az_offset);
    Serial.print(F("gyro offsets (X,Y,Z): "));
    Serial.print(gx_offset); Serial.print(", ");
    Serial.print(gy_offset); Serial.print(", ");
    Serial.println(gz_offset);
    Serial.println();

    // Enable the DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Setup interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP ready flag
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Report DMP initialization error
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

    
    // Display angles in degrees
    Serial.print("Orientation: ");
    Serial.print("Yaw=");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("° Pitch=");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("° Roll=");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.println("°");
    
    // Blink LED to indicate activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Small delay to avoid serial port flooding
    delay(50);
  }
}