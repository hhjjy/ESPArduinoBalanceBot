/**
 * DebugPage.cpp
 * 用於顯示調試信息的OLED頁面實現
 */

#include "DebugPage.h"

// 外部定義的RPM歷史數據陣列
extern double rpmHistory[64];
extern int historyIndex;

DebugPage::DebugPage(double* targetRPM, double* currentRPM, double* kp, double* ki, double* kd)
    : targetRPM(targetRPM),
      currentRPM(currentRPM),
      kp(kp),
      ki(ki),
      kd(kd),
      currentParamMode(PARAM_NONE)
{
}

void DebugPage::draw(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    // 設置字體
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    // 繪製頁面標題
    u8g2.drawStr(0, 10, "Motor PID Debug");
    
    // 繪製當前目標和實際RPM
    char buffer[20];
    sprintf(buffer, "T:%d C:%d", (int)*targetRPM, (int)*currentRPM);
    u8g2.drawStr(0, 20, buffer);
    
    // 繪製RPM示波器
    drawRPMGraph(u8g2);
    
    // 繪製參數
    drawParams(u8g2);
}

void DebugPage::drawRPMGraph(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    // 繪製示波器框架
    u8g2.drawFrame(0, 22, 128, 30);
    
    // 繪製目標RPM水平線
    int targetY = 52 - (int)((*targetRPM / 300.0) * 30.0);
    targetY = constrain(targetY, 22, 52);
    u8g2.drawHLine(0, targetY, 128);
    
    // 繪製RPM曲線
    for (int i = 0; i < 63; i++) {
        int idx1 = (historyIndex + i) % 64;
        int idx2 = (historyIndex + i + 1) % 64;
        
        int y1 = 52 - (int)((rpmHistory[idx1] / 300.0) * 30.0);
        int y2 = 52 - (int)((rpmHistory[idx2] / 300.0) * 30.0);
        
        // 確保y值在框架內
        y1 = constrain(y1, 22, 52);
        y2 = constrain(y2, 22, 52);
        
        u8g2.drawLine(i * 2, y1, (i + 1) * 2, y2);
    }
}

void DebugPage::drawParams(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) {
    char buffer[32];
    
    // 顯示參數值
    sprintf(buffer, "P:%.2f I:%.2f D:%.2f", *kp, *ki, *kd);
    u8g2.drawStr(0, 62, buffer);
    
    // 顯示當前選擇的參數
    switch (currentParamMode) {
        case PARAM_KP:
            u8g2.drawBox(0, 54, 8, 2);
            break;
        case PARAM_KI:
            u8g2.drawBox(42, 54, 8, 2);
            break;
        case PARAM_KD:
            u8g2.drawBox(84, 54, 8, 2);
            break;
        default:
            break;
    }
}

const char* DebugPage::getName() {
    return "Debug";
}

void DebugPage::update() {
    // 這裡不需要做什麼，因為數據是通過指針更新的
}

void DebugPage::nextParamMode() {
    currentParamMode = static_cast<ParamMode>((currentParamMode + 1) % PARAM_COUNT);
}

ParamMode DebugPage::getParamMode() {
    return currentParamMode;
}

void DebugPage::adjustParam(double delta) {
    switch (currentParamMode) {
        case PARAM_KP:
            *kp += delta;
            if (*kp < 0) *kp = 0;
            break;
        case PARAM_KI:
            *ki += delta;
            if (*ki < 0) *ki = 0;
            break;
        case PARAM_KD:
            *kd += delta;
            if (*kd < 0) *kd = 0;
            break;
        default:
            break;
    }
}