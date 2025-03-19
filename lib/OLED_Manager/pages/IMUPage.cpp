 #include "IMUPage.h"

IMUPage::IMUPage(IMU* imuPtr) : imu(imuPtr), currentMode(IMU_MODE_YPR) {
}
 void IMUPage::setDisplayMode(IMUDisplayMode mode) {
     if (mode < IMU_MODE_COUNT) {
         currentMode = mode;
     }
 }
 
 IMUDisplayMode IMUPage::getDisplayMode() {
     return currentMode;
 }
 
 void IMUPage::nextDisplayMode() {
     currentMode = static_cast<IMUDisplayMode>((currentMode + 1) % IMU_MODE_COUNT);
    Serial.print("IMU頁面模式切換到: ");
    Serial.println(currentMode);
 }
 
 void IMUPage::handleButtonPress() {
     nextDisplayMode();
 }
 
 void IMUPage::draw(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    // 根據當前模式繪製不同的頁面
     switch (currentMode) {
         case IMU_MODE_YPR:
             drawYPR(u8g2);
             break;
         case IMU_MODE_ACCEL_GYRO:
             drawAccelGyro(u8g2);
             break;
         case IMU_MODE_CALIBRATION:
             drawCalibrationValues(u8g2);
             break;
         default:
             drawYPR(u8g2);
             break;
     }
 }
 
 void IMUPage::drawYPR(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    u8g2.clearBuffer();
    
     // 設置字體
     u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 12, "IMU Data");
     
     u8g2.setFont(u8g2_font_ncenB08_tr);
     
    // 獲取 YPR 數據
    float ypr[3];
    imu->getYPR(ypr);
    
    // 轉換為角度
    float yaw = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;
    float roll = ypr[2] * 180 / M_PI;
    
    // 顯示數據
     char buffer[20];
     
    snprintf(buffer, sizeof(buffer), "Yaw  : %6.2f", yaw);
     u8g2.drawStr(0, 28, buffer);
     
    snprintf(buffer, sizeof(buffer), "Pitch: %6.2f", pitch);
    u8g2.drawStr(0, 42, buffer);
     
    snprintf(buffer, sizeof(buffer), "Roll : %6.2f", roll);
    u8g2.drawStr(0, 56, buffer);
     
    // 顯示模式指示器
    u8g2.drawStr(110, 12, "YPR");
    
    u8g2.sendBuffer();
}
 void IMUPage::drawAccelGyro(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    u8g2.clearBuffer();
     // 設置字體
     u8g2.setFont(u8g2_font_ncenB08_tr);
     u8g2.drawStr(0, 10, "Accelerometer:");
     
    // 獲取加速度計數據
    int16_t ax, ay, az;
    imu->getAcceleration(&ax, &ay, &az);
     char buffer[20];
     
    // 顯示加速度計數據
    snprintf(buffer, sizeof(buffer), "X:%5d", ax);
     u8g2.drawStr(0, 22, buffer);
     
    snprintf(buffer, sizeof(buffer), "Y:%5d", ay);
    u8g2.drawStr(55, 22, buffer);
     
    snprintf(buffer, sizeof(buffer), "Z:%5d", az);
     u8g2.drawStr(0, 34, buffer);
     
    // 顯示陀螺儀數據
    u8g2.drawStr(0, 46, "Gyroscope:");
     
    // 獲取陀螺儀數據
    int16_t gx, gy, gz;
    imu->getRotation(&gx, &gy, &gz);
    snprintf(buffer, sizeof(buffer), "X:%5d", gx);
     u8g2.drawStr(0, 58, buffer);
     
    snprintf(buffer, sizeof(buffer), "Y:%5d", gy);
    u8g2.drawStr(55, 58, buffer);
     
    snprintf(buffer, sizeof(buffer), "Z:%5d", gz);
    u8g2.drawStr(90, 58, buffer);
    
    // 顯示模式指示器
    u8g2.drawStr(110, 10, "A/G");
    
    u8g2.sendBuffer();
 }

void IMUPage::drawCalibrationValues(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Accel Offsets:");
    
    // 獲取校準值
    int16_t accelOffset[3], gyroOffset[3];
    imu->getCalibrationValues(accelOffset, gyroOffset);
    
    char buffer[20];
    
    // 顯示加速度計偏移
    snprintf(buffer, sizeof(buffer), "X:%5d", accelOffset[0]);
    u8g2.drawStr(0, 22, buffer);
    
    snprintf(buffer, sizeof(buffer), "Y:%5d", accelOffset[1]);
    u8g2.drawStr(60, 22, buffer);
    
    snprintf(buffer, sizeof(buffer), "Z:%5d", accelOffset[2]);
    u8g2.drawStr(0, 34, buffer);
    
    // 顯示陀螺儀偏移
    u8g2.drawStr(0, 46, "Gyro Offsets:");
    
    snprintf(buffer, sizeof(buffer), "X:%5d", gyroOffset[0]);
    u8g2.drawStr(0, 58, buffer);
    
    snprintf(buffer, sizeof(buffer), "Y:%5d", gyroOffset[1]);
    u8g2.drawStr(60, 58, buffer);
    
    // 顯示模式指示器
    u8g2.drawStr(110, 10, "CAL");
    
    u8g2.sendBuffer();
}

const char* IMUPage::getName() {
    return "IMU";
}

void IMUPage::update() {
    // 更新 IMU 數據
    imu->update();
}