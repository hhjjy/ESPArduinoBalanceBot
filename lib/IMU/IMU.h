/**
 * IMU.h
 * MPU6050 IMU 傳感器管理庫
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Preferences.h>

class IMU {
private:
    // MPU6050 相關變數
    MPU6050 mpu;
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    
    // 方向/運動變數
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    
    // 校準偏移值
    int16_t ax_offset, ay_offset, az_offset;
    int16_t gx_offset, gy_offset, gz_offset;
    
    // Preferences對象，用於存儲校準值
    Preferences preferences;
    
    // 偏好設置的命名空間
    const char* prefsNamespace = "imu_cal";
    
    // 上次更新時間
    unsigned long lastUpdate;
    
    // 更新間隔 (ms)
    unsigned long updateInterval;
    
    // 濾波器係數 (0-1)，越大表示原始數據權重越大
    float filterAlpha;
    
    // 是否已初始化
    bool initialized;
    
public:
    /**
     * 建構函數
     * @param updateIntervalMs 更新間隔，單位毫秒
     * @param alpha 濾波器係數 (0-1)，越大表示原始數據權重越大
     */
    IMU(unsigned long updateIntervalMs = 10, float alpha = 0.98);
    
    /**
     * 析構函數
     */
    ~IMU();
    
    /**
     * 初始化IMU
     * @param sda I2C SDA腳位
     * @param scl I2C SCL腳位
     * @param address MPU6050 I2C地址 (默認0x68)
     * @return 初始化成功返回true，失敗返回false
     */
    bool begin(int sda, int scl, uint8_t address = 0x68);
    
    /**
     * 校準MPU6050
     * @param samples 校準樣本數量，越多越精確但耗時越長
     * @param progressCallback 進度回調函數，用於更新校準進度
     * @return 校準成功返回true
     */
    bool calibrate(uint8_t samples = 6, std::function<void(const char*, int)> progressCallback = nullptr);
    
    /**
     * 從Preferences載入儲存的校準值
     * @return 存在有效校準數據時返回true
     */
    bool loadCalibration();
    
    /**
     * 將校準值儲存到Preferences
     * @return 儲存成功返回true
     */
    bool saveCalibration();
    
    /**
     * 獲取校準值
     * @param accelOffset 加速度計偏移值數組 [ax, ay, az]
     * @param gyroOffset 陀螺儀偏移值數組 [gx, gy, gz]
     */
    void getCalibrationValues(int16_t accelOffset[3], int16_t gyroOffset[3]);
    
    /**
     * 設置校準值
     * @param accelOffset 加速度計偏移值數組 [ax, ay, az]
     * @param gyroOffset 陀螺儀偏移值數組 [gx, gy, gz]
     */
    void setCalibrationValues(const int16_t accelOffset[3], const int16_t gyroOffset[3]);
    
    /**
     * 更新IMU數據
     * 應在主循環中定期調用
     * @return 如果有新數據更新則返回true
     */
    bool update();
    
    /**
     * 獲取YPR數據
     * @param yawPitchRoll 用於儲存結果的數組，按順序為偏航、俯仰、翻滾角度（弧度）
     * @return 獲取成功返回true
     */
    bool getYPR(float* yawPitchRoll);
    
    /**
     * 獲取偏航角（Yaw）
     * @return 偏航角（弧度）
     */
    float getYaw();
    
    /**
     * 獲取俯仰角（Pitch）
     * @return 俯仰角（弧度）
     */
    float getPitch();
    
    /**
     * 獲取翻滾角（Roll）
     * @return 翻滾角（弧度）
     */
    float getRoll();
    
    /**
     * 獲取原始加速度計數據
     * @param ax X軸加速度
     * @param ay Y軸加速度
     * @param az Z軸加速度
     */
    void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
    
    /**
     * 獲取原始陀螺儀數據
     * @param gx X軸角速度
     * @param gy Y軸角速度
     * @param gz Z軸角速度
     */
    void getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
    
    /**
     * 獲取溫度
     * @return 溫度（攝氏度）
     */
    float getTemperature();
    
    /**
     * 設置濾波器係數
     * @param alpha 濾波器係數 (0-1)，越大表示原始數據權重越大
     */
    void setFilterAlpha(float alpha);
    
    /**
     * 設置更新間隔
     * @param intervalMs 更新間隔，單位毫秒
     */
    void setUpdateInterval(unsigned long intervalMs);
    
    /**
     * 檢查IMU是否已初始化
     * @return 初始化成功返回true
     */
    bool isInitialized();
    
    /**
     * 獲取MPU6050對象引用
     * @return MPU6050對象引用
     */
    MPU6050& getMPU();
    
    /**
     * 重置DMP
     * 當DMP出現問題時調用
     * @return 重置成功返回true
     */
    bool resetDMP();
    
    /**
     * 獲取校準狀態
     * @return 已校準返回true
     */
    bool isCalibrated();
};

#endif // IMU_H