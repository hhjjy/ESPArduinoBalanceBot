/**
 * IMU.cpp
 * MPU6050 IMU 傳感器管理庫實現
 */

#include "IMU.h"

// 建構函數
IMU::IMU(unsigned long updateIntervalMs, float alpha)
    : mpu(),
      dmpReady(false),
      lastUpdate(0),
      updateInterval(updateIntervalMs),
      filterAlpha(alpha),
      initialized(false)
{
    // 初始化ypr陣列
    ypr[0] = ypr[1] = ypr[2] = 0.0f;
}

// 析構函數
IMU::~IMU() {
    preferences.end();
}

// 初始化IMU
bool IMU::begin(int sda, int scl, uint8_t address) {
    // 初始化preferences
    preferences.begin(prefsNamespace, false);
    
    // 初始化Wire (I2C)
    Wire.begin(sda, scl);
    
    // 初始化MPU6050 - 注意：MPU6050類沒有setAddress方法
    mpu.initialize();
    
    // 測試連接
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
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
        initialized = true;
        return true;
    } else {
        // DMP初始化失敗
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        return false;
    }
}

// 校準MPU6050
bool IMU::calibrate(uint8_t samples, std::function<void(const char*, int)> progressCallback) {
    if (!initialized) {
        return false;
    }
    
    // 校準加速度計
    for (int i = 0; i < samples; i++) {
        if (progressCallback) {
            progressCallback("Accelerometer", (i * 100) / samples);
        }
        mpu.CalibrateAccel(1);  // 每次迭代校準一次
        delay(50);
    }
    
    // 校準陀螺儀
    for (int i = 0; i < samples; i++) {
        if (progressCallback) {
            progressCallback("Gyroscope", (i * 100) / samples);
        }
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
    
    return true;
}

// 將校準值儲存到Preferences
bool IMU::saveCalibration() {
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
bool IMU::loadCalibration() {
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

// 獲取校準值
void IMU::getCalibrationValues(int16_t accelOffset[3], int16_t gyroOffset[3]) {
    accelOffset[0] = ax_offset;
    accelOffset[1] = ay_offset;
    accelOffset[2] = az_offset;
    
    gyroOffset[0] = gx_offset;
    gyroOffset[1] = gy_offset;
    gyroOffset[2] = gz_offset;
}

// 設置校準值
void IMU::setCalibrationValues(const int16_t accelOffset[3], const int16_t gyroOffset[3]) {
    ax_offset = accelOffset[0];
    ay_offset = accelOffset[1];
    az_offset = accelOffset[2];
    
    gx_offset = gyroOffset[0];
    gy_offset = gyroOffset[1];
    gz_offset = gyroOffset[2];
    
    // 應用偏移值
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    
    // 儲存到Preferences
    saveCalibration();
}

// 更新IMU數據
bool IMU::update() {
    if (!dmpReady) return false;
    
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate < updateInterval) {
        return false;  // 尚未到更新時間
    }
    
    lastUpdate = currentTime;
    
    // 讀取DMP數據
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        return true;
    }
    
    return false;
}

// 獲取YPR數據
bool IMU::getYPR(float* yawPitchRoll) {
    if (!dmpReady) return false;
    
    // 複製到傳出參數
    if (yawPitchRoll) {
        yawPitchRoll[0] = ypr[0];
        yawPitchRoll[1] = ypr[1];
        yawPitchRoll[2] = ypr[2];
    }
    
    return true;
}

// 獲取偏航角 (Yaw)
float IMU::getYaw() {
    return ypr[0];
}

// 獲取俯仰角 (Pitch)
float IMU::getPitch() {
    return ypr[1];
}

// 獲取翻滾角 (Roll)
float IMU::getRoll() {
    return ypr[2];
}

// 獲取原始加速度計數據
void IMU::getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
    mpu.getAcceleration(ax, ay, az);
}

// 獲取原始陀螺儀數據
void IMU::getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu.getRotation(gx, gy, gz);
}

// 獲取溫度
float IMU::getTemperature() {
    return mpu.getTemperature() / 340.0 + 36.53;  // 根據MPU6050數據手冊的公式
}

// 設置濾波器係數
void IMU::setFilterAlpha(float alpha) {
    filterAlpha = constrain(alpha, 0.0f, 1.0f);
}

// 設置更新間隔
void IMU::setUpdateInterval(unsigned long intervalMs) {
    updateInterval = intervalMs;
}

// 檢查IMU是否已初始化
bool IMU::isInitialized() {
    return initialized;
}

// 獲取MPU6050對象引用
MPU6050& IMU::getMPU() {
    return mpu;
}

// 重置DMP
bool IMU::resetDMP() {
    if (!initialized) {
        return false;
    }
    
    // 禁用DMP
    mpu.setDMPEnabled(false);
    
    // 重新初始化DMP
    devStatus = mpu.dmpInitialize();
    
    // 應用校準值
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    
    // 檢查初始化是否成功
    if (devStatus == 0) {
        // 啟用DMP
        mpu.setDMPEnabled(true);
        
        // 取得每個數據包的大小
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        return true;
    }
    
    return false;
}

// 獲取校準狀態
bool IMU::isCalibrated() {
    return preferences.getBool("cal_valid", false);
}