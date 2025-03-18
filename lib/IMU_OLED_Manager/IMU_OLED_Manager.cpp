/**
 * IMU_OLED_Manager.cpp
 * 整合MPU6050與SH1106 OLED顯示的管理庫實現
 */

#include "IMU_OLED_Manager.h"

// 建構函數
IMU_OLED_Manager::IMU_OLED_Manager() 
    : mpu(), 
      u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE),
      dmpReady(false),
      lastDisplayUpdate(0),
      displayUpdateInterval(100),  // 預設更新間隔為100ms
      currentMode(MODE_YPR)        // 預設為YPR顯示模式
{
    // 初始化ypr陣列
    ypr[0] = ypr[1] = ypr[2] = 0.0f;
}

// 初始化全部功能
bool IMU_OLED_Manager::begin(int sda, int scl ) {
    // 初始化preferences
    preferences.begin(prefsNamespace, false);
    
    // 初始化Wire (I2C)
    Wire.begin(sda, scl);
    
    // 初始化顯示器
    if (!initDisplay()) {
        return false;
    }
    
    // 顯示啟動畫面
    drawSplashScreen();
    delay(1000);
    
    // 顯示初始化MPU訊息
    displayMessage("Initializing", "MPU6050...");
    
    // 初始化MPU
    if (!initMPU()) {
        displayMessage("MPU6050 Init", "Failed!");
        delay(2000);
        return false;
    }
    
    // 嘗試載入校準值
    if (loadCalibration()) {
        displayMessage("Calibration", "Loaded!");
        delay(1000);
    } else {
        displayMessage("No Calibration", "Data Found");
        delay(1000);
    }
    
    return true;
}

// 初始化MPU6050
bool IMU_OLED_Manager::initMPU() {
    // 初始化MPU6050
    mpu.initialize();
    
    // 測試連接
    if (!mpu.testConnection()) {
        return false;
    }
    
    // 初始化DMP
    devStatus = mpu.dmpInitialize();
    
    // 如果已載入校準值，則設置偏移
    if (loadCalibration()) {
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);
        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);
    }
    
    // 檢查初始化是否成功
    if (devStatus == 0) {
        // 啟用DMP
        mpu.setDMPEnabled(true);
        
        // 取得每個數據包的大小
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        return true;
    } else {
        return false;
    }
}

// 初始化OLED顯示
bool IMU_OLED_Manager::initDisplay() {
    u8g2.begin();
    u8g2.clearBuffer();
    return true;
}

// 校準MPU6050
bool IMU_OLED_Manager::calibrateMPU(uint8_t samples) {
    // 設置靜態提示
    displayMessage("Calibrating", "Keep Device Still");
    delay(2000);
    
    // 校準加速度計
    displayMessage("Calibrating", "Accelerometer");
    for (int i = 0; i < samples; i++) {
        drawCalibrationScreen("Accelerometer", (i * 100) / samples);
        mpu.CalibrateAccel(1);  // 每次迭代校準一次
        delay(50);
    }
    
    // 校準陀螺儀
    displayMessage("Calibrating", "Gyroscope");
    for (int i = 0; i < samples; i++) {
        drawCalibrationScreen("Gyroscope", (i * 100) / samples);
        mpu.CalibrateGyro(1);  // 每次迭代校準一次
        delay(50);
    }
    
    // 儲存校準後的偏移值
    ax_offset = mpu.getXAccelOffset();
    ay_offset = mpu.getYAccelOffset();
    az_offset = mpu.getZAccelOffset();
    gx_offset = mpu.getXGyroOffset();
    gy_offset = mpu.getYGyroOffset();
    gz_offset = mpu.getZGyroOffset();
    
    // 儲存到Preferences
    saveCalibration();
    
    // 顯示完成訊息
    displayMessage("Calibration", "Complete!");
    delay(1000);
    
    return true;
}

// 將校準值儲存到Preferences
bool IMU_OLED_Manager::saveCalibration() {
    // 儲存加速度計偏移
    preferences.putShort("ax_offset", ax_offset);
    preferences.putShort("ay_offset", ay_offset);
    preferences.putShort("az_offset", az_offset);
    
    // 儲存陀螺儀偏移
    preferences.putShort("gx_offset", gx_offset);
    preferences.putShort("gy_offset", gy_offset);
    preferences.putShort("gz_offset", gz_offset);
    
    // 儲存有效標記
    preferences.putBool("cal_valid", true);
    
    return true;
}

// 從Preferences載入校準值
bool IMU_OLED_Manager::loadCalibration() {
    // 檢查是否有有效校準數據
    if (!preferences.getBool("cal_valid", false)) {
        return false;
    }
    
    // 讀取加速度計偏移
    ax_offset = preferences.getShort("ax_offset", 0);
    ay_offset = preferences.getShort("ay_offset", 0);
    az_offset = preferences.getShort("az_offset", 0);
    
    // 讀取陀螺儀偏移
    gx_offset = preferences.getShort("gx_offset", 0);
    gy_offset = preferences.getShort("gy_offset", 0);
    gz_offset = preferences.getShort("gz_offset", 0);
    
    return true;
}

// 顯示YPR (偏航、俯仰、翻滾) 資料
void IMU_OLED_Manager::displayYPR() {
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(10, 12, "Balance Bot");
    
    // 顯示YPR數據
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    char buffer[20];
    
    // 顯示偏航角 (Yaw)
    snprintf(buffer, sizeof(buffer), "Yaw: %.2f", ypr[0] * 180 / M_PI);
    u8g2.drawStr(0, 28, buffer);
    
    // 顯示俯仰角 (Pitch)
    snprintf(buffer, sizeof(buffer), "Pitch: %.2f", ypr[1] * 180 / M_PI);
    u8g2.drawStr(0, 40, buffer);
    
    // 顯示翻滾角 (Roll)
    snprintf(buffer, sizeof(buffer), "Roll: %.2f", ypr[2] * 180 / M_PI);
    u8g2.drawStr(0, 52, buffer);
    
    // 繪製傾斜指示器
    int16_t pitch_indicator = map(constrain((int)(ypr[1] * 180 / M_PI), -90, 90), -90, 90, -20, 20);
    int16_t roll_indicator = map(constrain((int)(ypr[2] * 180 / M_PI), -90, 90), -90, 90, -20, 20);
    
    u8g2.drawFrame(95, 28, 26, 26);
    u8g2.drawDisc(108 + roll_indicator, 41 + pitch_indicator, 3);
    
    u8g2.sendBuffer();
}

// 顯示加速度計和陀螺儀原始數據
void IMU_OLED_Manager::displayAccelGyro() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Accelerometer:");
    
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
    
    snprintf(buffer, sizeof(buffer), "X:%5d", gx);
    u8g2.drawStr(0, 58, buffer);
    
    snprintf(buffer, sizeof(buffer), "Y:%5d", gy);
    u8g2.drawStr(55, 58, buffer);
    
    snprintf(buffer, sizeof(buffer), "Z:%5d", gz);
    u8g2.drawStr(90, 58, buffer);
    
    u8g2.sendBuffer();
}

// 顯示校準值
void IMU_OLED_Manager::displayCalibrationValues() {
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Accel Offsets:");
    
    char buffer[20];
    
    // 顯示加速度計偏移
    snprintf(buffer, sizeof(buffer), "X:%5d", ax_offset);
    u8g2.drawStr(0, 22, buffer);
    
    snprintf(buffer, sizeof(buffer), "Y:%5d", ay_offset);
    u8g2.drawStr(60, 22, buffer);
    
    snprintf(buffer, sizeof(buffer), "Z:%5d", az_offset);
    u8g2.drawStr(0, 34, buffer);
    
    // 顯示陀螺儀偏移
    u8g2.drawStr(0, 46, "Gyro Offsets:");
    
    snprintf(buffer, sizeof(buffer), "X:%5d", gx_offset);
    u8g2.drawStr(0, 58, buffer);
    
    snprintf(buffer, sizeof(buffer), "Y:%5d", gy_offset);
    u8g2.drawStr(60, 58, buffer);
    
    snprintf(buffer, sizeof(buffer), "Z:%5d", gz_offset);
    u8g2.drawStr(0, 46, buffer);
    
    u8g2.sendBuffer();
}

// 顯示自定義數據
void IMU_OLED_Manager::displayCustomData(const char* title, const char* data1, const char* data2, const char* data3) {
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB10_tr);
    if (title) {
        u8g2.drawStr((128 - u8g2.getStrWidth(title)) / 2, 12, title);
    }
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    // 顯示數據行
    if (data1) {
        u8g2.drawStr(0, 26, data1);
    }
    
    if (data2) {
        u8g2.drawStr(0, 40, data2);
    }
    
    if (data3) {
        u8g2.drawStr(0, 54, data3);
    }
    
    u8g2.sendBuffer();
}

// 顯示一般訊息
void IMU_OLED_Manager::displayMessage(const char* line1, const char* line2) {
    u8g2.clearBuffer();
    
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB10_tr);
    
    // 顯示第一行訊息
    if (line1) {
        u8g2.drawStr((128 - u8g2.getStrWidth(line1)) / 2, 24, line1);
    }
    
    // 顯示第二行訊息
    if (line2) {
        u8g2.drawStr((128 - u8g2.getStrWidth(line2)) / 2, 44, line2);
    }
    
    u8g2.sendBuffer();
}

// 設置顯示模式
void IMU_OLED_Manager::setDisplayMode(DisplayMode mode) {
    if (mode < MODE_COUNT) {
        currentMode = mode;
    }
}

// 獲取目前顯示模式
DisplayMode IMU_OLED_Manager::getDisplayMode() {
    return currentMode;
}

// 切換到下一個顯示模式
void IMU_OLED_Manager::nextDisplayMode() {
    currentMode = static_cast<DisplayMode>((currentMode + 1) % MODE_COUNT);
}

// 獲取YPR數據
bool IMU_OLED_Manager::getYPR(float* yawPitchRoll) {
    if (!dmpReady) return false;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // 複製到傳出參數
        if (yawPitchRoll) {
            yawPitchRoll[0] = ypr[0];
            yawPitchRoll[1] = ypr[1];
            yawPitchRoll[2] = ypr[2];
        }
        
        return true;
    }
    
    return false;
}

// 獲取偏航角 (Yaw)
float IMU_OLED_Manager::getYaw() {
    return ypr[0];
}

// 獲取俯仰角 (Pitch)
float IMU_OLED_Manager::getPitch() {
    return ypr[1];
}

// 獲取翻滾角 (Roll)
float IMU_OLED_Manager::getRoll() {
    return ypr[2];
}

// 更新函數 - 應在主程式的loop()中調用
void IMU_OLED_Manager::update() {
    // 更新YPR數據
    getYPR(NULL);
    
    // 定期更新顯示
    unsigned long currentMillis = millis();
    if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
        lastDisplayUpdate = currentMillis;
        
        // 根據當前模式更新顯示
        switch (currentMode) {
            case MODE_YPR:
                displayYPR();
                break;
                
            case MODE_ACCEL_GYRO:
                displayAccelGyro();
                break;
                
            case MODE_CALIBRATION_VALUES:
                displayCalibrationValues();
                break;
                
            case MODE_CUSTOM_DATA:
                // 使用自定義數據方法時，由外部调用displayCustomData
                break;
                
            default:
                displayYPR();
                break;
        }
    }
}

// 繪製進度條
void IMU_OLED_Manager::drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress) {
    u8g2.drawFrame(x, y, width, height);
    u8g2.drawBox(x + 2, y + 2, (width - 4) * progress / 100, height - 4);
}

// 繪製校準畫面
void IMU_OLED_Manager::drawCalibrationScreen(const char* message, int progress) {
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr((128 - u8g2.getStrWidth("Calibrating")) / 2, 15, "Calibrating");
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr((128 - u8g2.getStrWidth(message)) / 2, 35, message);
    
    drawProgressBar(14, 45, 100, 10, progress);
    
    u8g2.sendBuffer();
}

// 繪製啟動畫面
void IMU_OLED_Manager::drawSplashScreen() {
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr((128 - u8g2.getStrWidth("Balance Bot")) / 2, 15, "Balance Bot");
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr((128 - u8g2.getStrWidth("Sensor System")) / 2, 35, "Sensor System");
    
    // 繪製簡單的機器人圖示
    u8g2.drawCircle(64, 50, 8, U8G2_DRAW_ALL);
    u8g2.drawLine(64, 58, 64, 62);
    u8g2.drawLine(64, 62, 58, 68);
    u8g2.drawLine(64, 62, 70, 68);
    
    u8g2.sendBuffer();
}