/**
 * IMUPage.h
 * MPU6050 IMU 數據顯示頁面
 */

#ifndef IMU_PAGE_H
#define IMU_PAGE_H

#include "../DisplayPage.h"
#include "IMU.h"

// 顯示模式
enum IMUDisplayMode {
    IMU_MODE_YPR,              // 偏航、俯仰、翻滾顯示
    IMU_MODE_ACCEL_GYRO,       // 加速度計和陀螺儀數據顯示
    IMU_MODE_CALIBRATION,      // 校準值顯示
    IMU_MODE_COUNT             // 模式總數（用於循環切換）
};

class IMUPage : public DisplayPage {
private:
    // IMU 對象指針
    IMU* imu;
    
    // 顯示模式
    IMUDisplayMode currentMode;
    
    // 繪製不同模式的頁面
    void drawYPR(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2);
    void drawAccelGyro(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2);
    void drawCalibrationValues(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2);
    
public:
    /**
     * 建構函數
     * @param imuPtr IMU對象指針
     */
    IMUPage(IMU* imuPtr);
    
    /**
     * 設置顯示模式
     * @param mode 要設置的顯示模式
     */
    void setDisplayMode(IMUDisplayMode mode);
    
    /**
     * 獲取目前顯示模式
     * @return 當前顯示模式
     */
    IMUDisplayMode getDisplayMode();
    
    /**
     * 切換到下一個顯示模式
     */
    void nextDisplayMode();
    
    /**
     * 處理按鈕按下事件
     */
    virtual void handleButtonPress() override;
    
    /**
     * 繪製頁面
     * @param u8g2 U8G2 對象引用
     */
    virtual void draw(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) override;
    
    /**
     * 獲取頁面名稱
     * @return 頁面名稱
     */
    virtual const char* getName() override;
    
    /**
     * 更新頁面數據
     */
    virtual void update() override;
};

#endif // IMU_PAGE_H