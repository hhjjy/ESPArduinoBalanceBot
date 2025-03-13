#ifndef PINS_H
#define PINS_H

// I2C 介面腳位
#define I2C_SDA 5
#define I2C_SCL 6
#define I2C_OLED_ADDR 0x3C
#define I2C_MPU_ADDR 0x68


// 馬達 1 控制腳位
#define MOTOR1_PWM 7
#define MOTOR1_AIN1 15
#define MOTOR1_AIN2 16
#define MOTOR1_ENA 12
#define MOTOR1_ENB 11

// 馬達 2 控制腳位
#define MOTOR2_PWM 17
#define MOTOR2_AIN1 18
#define MOTOR2_AIN2 8
#define MOTOR2_ENA 2
#define MOTOR2_ENB 1

// 馬達共用控制腳位
#define MOTOR_STBY 17

// OLED 
#define OLED_WIDTH 128      // OLED 顯示器寬度，常見值為 128 像素
#define OLED_HEIGHT 64      // OLED 顯示器高度，常見值為 32 或 64 像
// MPU (慣性測量單元) 腳位
#define MPU_INT 18

// WS2812 燈珠 
#define WS2812_DI 48 

#endif // PINS_H
