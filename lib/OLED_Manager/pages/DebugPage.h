/**
 * DebugPage.h
 * 用於顯示調試信息的OLED頁面
 */

#ifndef DEBUG_PAGE_H
#define DEBUG_PAGE_H

#include "DisplayPage.h"
#include <U8g2lib.h>

// 參數調整模式
enum ParamMode {
    PARAM_NONE,    // 不調整參數
    PARAM_KP,      // 調整 Kp
    PARAM_KI,      // 調整 Ki
    PARAM_KD,      // 調整 Kd
    PARAM_COUNT    // 參數總數
};

class DebugPage : public DisplayPage {
private:
    double* targetRPM;    // 目標RPM指針
    double* currentRPM;   // 當前RPM指針
    double* kp;           // Kp參數指針
    double* ki;           // Ki參數指針
    double* kd;           // Kd參數指針
    
    ParamMode currentParamMode;  // 當前參數調整模式
    
    // 繪製RPM示波器
    void drawRPMGraph(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2);
    
    // 繪製參數顯示
    void drawParams(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2);

public:
    /**
     * 建構函數
     * @param targetRPM 目標RPM指針
     * @param currentRPM 當前RPM指針
     * @param kp Kp參數指針
     * @param ki Ki參數指針
     * @param kd Kd參數指針
     */
    DebugPage(double* targetRPM, double* currentRPM, double* kp, double* ki, double* kd);
    
    /**
     * 繪製頁面
     * @param u8g2 U8G2對象
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
    
    /**
     * 切換到下一個參數調整模式
     */
    void nextParamMode();
    
    /**
     * 獲取當前參數調整模式
     * @return 當前參數調整模式
     */
    ParamMode getParamMode();
    
    /**
     * 調整當前選擇的參數
     * @param delta 參數變化量
     */
    void adjustParam(double delta);
};

#endif // DEBUG_PAGE_H