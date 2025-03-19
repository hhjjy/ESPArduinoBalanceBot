/**
 * MotorPage.h
 * 馬達數據顯示頁面
 */

#ifndef MOTOR_PAGE_H
#define MOTOR_PAGE_H

#include "OLED_Manager.h"
#include "motor.h"
#include "encoder.h"

class MotorPage : public DisplayPage {
private:
    Motor* motor1;
    Motor* motor2;
    Encoder* encoder1;
    Encoder* encoder2;
    
    // 顯示數據
    float motor1_rpm;
    float motor2_rpm;
    int motor1_speed;
    int motor2_speed;
    int motor1_direction;
    int motor2_direction;
    
public:
    /**
     * 建構函數
     * @param m1 馬達1指針
     * @param m2 馬達2指針
     * @param e1 編碼器1指針
     * @param e2 編碼器2指針
     */
    MotorPage(Motor* m1, Motor* m2, Encoder* e1, Encoder* e2);
    
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

#endif // MOTOR_PAGE_H