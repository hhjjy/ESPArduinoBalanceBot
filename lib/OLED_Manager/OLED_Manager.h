/**
 * OLED_Manager.h
 * 通用 OLED 顯示管理庫
 * 
 * 功能概述:
 * - 提供通用的 OLED 顯示管理功能
 * - 支持多頁面系統，可以輕鬆添加和切換不同的顯示頁面
 * - 提供簡單的 API 用於顯示文本、圖形和自定義數據
 * - 可擴展設計，允許添加特定模組的顯示頁面
 */

#ifndef OLED_MANAGER_H
#define OLED_MANAGER_H

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "DisplayPage.h"

// 最大頁面數量
#define MAX_PAGES 8

class OLED_Manager {
private:
    // U8G2 顯示器對象
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
    
    // 頁面數組
    DisplayPage* pages[MAX_PAGES];
    
    // 當前頁面索引
    int currentPageIndex;
    
    // 頁面總數
    int pageCount;
    
    // 上次更新時間
    unsigned long lastUpdateTime;
    
    // 更新間隔 (ms)
    unsigned long updateInterval;

    // 繪製進度條
    void drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress);
    
    // 繪製啟動畫面
    void drawSplashScreen(const char* title, const char* subtitle);

public:
    /**
     * 建構函數
     */
    OLED_Manager();
    
    /**
     * 析構函數
     */
    ~OLED_Manager();
    
    /**
     * 初始化 OLED 顯示器
     * @param sda I2C SDA 引腳
     * @param scl I2C SCL 引腳
     * @param showSplash 是否顯示啟動畫面
     * @return 初始化成功返回 true
     */
    bool begin(int sda, int scl, bool showSplash = true);
    
    /**
     * 添加頁面
     * @param page 頁面指針
     * @return 添加成功返回 true
     */
    bool addPage(DisplayPage* page);
    
    /**
     * 設置當前頁面
     * @param index 頁面索引
     */
    void setPage(int index);
    
    /**
     * 獲取當前頁面索引
     * @return 當前頁面索引
     */
    int getCurrentPageIndex();
    
    /**
     * 獲取頁面總數
     * @return 頁面總數
     */
    int getPageCount();
    
    /**
     * 切換到下一個頁面
     */
    void nextPage();
    
    /**
     * 切換到上一個頁面
     */
    void prevPage();
    
    /**
     * 更新顯示
     * 應在主循環中定期調用
     */
    void update();
    
    /**
     * 顯示訊息
     * @param line1 第一行文字
     * @param line2 第二行文字
     * @param delay_ms 顯示時間 (ms)，0 表示不自動清除
     */
    void displayMessage(const char* line1, const char* line2 = nullptr, unsigned long delay_ms = 0);
    
    /**
     * 顯示進度條
     * @param message 顯示訊息
     * @param progress 進度值 (0-100)
     */
    void displayProgress(const char* message, int progress);
    
    /**
     * 設置更新間隔
     * @param interval 更新間隔 (ms)
     */
    void setUpdateInterval(unsigned long interval);
    
    /**
     * 獲取 U8G2 對象引用
     * @return U8G2 對象引用
     */
    U8G2_SH1106_128X64_NONAME_F_HW_I2C& getU8G2();
};

#endif // OLED_MANAGER_H