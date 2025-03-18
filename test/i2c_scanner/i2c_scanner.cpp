#include <Arduino.h>
#include <Wire.h>
#include "config.h"

void setup() {
  Serial.begin(115200);
  
  // Wait for serial port to connect
  while (!Serial) delay(10);
  
  Serial.println("\nI2C Scanner");
  
  // Initialize I2C with pins defined in config.h
  Wire.begin(I2C_SDA, I2C_SCL);
  
  Serial.println("Scanning I2C bus...");
}

void loop() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("Scanning...");
  
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");

      deviceCount++;
      
      // Check if this might be an OLED display
      if (address == 0x3C || address == 0x3D || address == 0x78 || address == 0x7A) {
        Serial.println("This could be an OLED display!");
}
      
      // Check if this might be an MPU6050
      if (address == 0x68 || address == 0x69) {
        Serial.println("This could be an MPU6050 sensor!");
      }
      
      delay(100);
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done scanning!\n");
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  
  delay(5000); // Wait 5 seconds before scanning again
}