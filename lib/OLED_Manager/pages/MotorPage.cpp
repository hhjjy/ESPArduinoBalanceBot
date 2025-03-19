/**
 * MotorPage.cpp
 * 馬達數據顯示頁面實現
 */

#include "MotorPage.h"

// 建構函數
MotorPage::MotorPage(Motor* m1, Motor* m2, Encoder* e1, Encoder* e2)
    : motor1(m1), motor2(m2), encoder1(e1), encoder2(e2),
      motor1_rpm(0), motor2_rpm(0),
      motor1_speed(0), motor2_speed(0),
      motor1_direction(0), motor2_direction(0)
{
}

void MotorPage::draw(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0, 12, "Motor Status");
    
    // 設置較小字體
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    char buffer[20];
    
    // 馬達1數據
    u8g2.drawStr(0, 25, "Motor 1:");
    
    // 速度
    snprintf(buffer, sizeof(buffer), "Spd:%d", motor1_speed);
    u8g2.drawStr(0, 35, buffer);
    
    // RPM
    snprintf(buffer, sizeof(buffer), "RPM:%.1f", motor1_rpm);
    u8g2.drawStr(60, 35, buffer);
    
    // 馬達2數據
    u8g2.drawStr(0, 48, "Motor 2:");
    
    // 速度
    snprintf(buffer, sizeof(buffer), "Spd:%d", motor2_speed);
    u8g2.drawStr(0, 58, buffer);
    
    // RPM
    snprintf(buffer, sizeof(buffer), "RPM:%.1f", motor2_rpm);
    u8g2.drawStr(60, 58, buffer);
    
    // 繪製方向指示器
    // 馬達1方向
    int x1 = 120;
    int y1 = 30;
    if (motor1_direction > 0) {
        // 向前箭頭
        u8g2.drawTriangle(x1, y1-5, x1-5, y1+5, x1+5, y1+5);
    } else if (motor1_direction < 0) {
        // 向後箭頭
        u8g2.drawTriangle(x1, y1+5, x1-5, y1-5, x1+5, y1-5);
    } else {
        // 停止方塊
        u8g2.drawBox(x1-4, y1-4, 8, 8);
    }
    
    // 馬達2方向
    int x2 = 120;
    int y2 = 53;
    if (motor2_direction > 0) {
        // 向前箭頭
        u8g2.drawTriangle(x2, y2-5, x2-5, y2+5, x2+5, y2+5);
    } else if (motor2_direction < 0) {
        // 向後箭頭
        u8g2.drawTriangle(x2, y2+5, x2-5, y2-5, x2+5, y2-5);
    } else {
        // 停止方塊
        u8g2.drawBox(x2-4, y2-4, 8, 8);
    }
    
    // 注意：不需要在這裡調用 u8g2.sendBuffer()
    // 因為 OLED_Manager::update() 方法會處理緩衝區的發送
}

// 獲取頁面名稱
const char* MotorPage::getName() {
    return "Motor Status";
}

// 更新頁面數據
void MotorPage::update() {
    if (motor1 && encoder1) {
        motor1_speed = motor1->getSpeed();
        motor1_rpm = encoder1->getRPM();
        motor1_direction = encoder1->getDirection();
    }
    
    if (motor2 && encoder2) {
        motor2_speed = motor2->getSpeed();
        motor2_rpm = encoder2->getRPM();
        motor2_direction = encoder2->getDirection();
    }
}