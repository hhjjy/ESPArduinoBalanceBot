/**
 * OLED_Manager.cpp
 * OLED 顯示管理器實現
 */

#include "OLED_Manager.h"

// 建構函數
OLED_Manager::OLED_Manager()
    : u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE),
      currentPageIndex(0),
      pageCount(0),
      lastUpdateTime(0),
      updateInterval(50)
{
    // 初始化頁面數組
    for (int i = 0; i < MAX_PAGES; i++) {
        pages[i] = nullptr;
    }
}

// 析構函數
OLED_Manager::~OLED_Manager() {
    // 注意：不刪除頁面對象，因為它們可能是在外部創建的
}

// 初始化 OLED 顯示器
bool OLED_Manager::begin(int sda, int scl, bool showSplash) {
    // 初始化 I2C
    Wire.begin(sda, scl);
    
    // 初始化 U8G2
    u8g2.begin();
    
    // 清空顯示
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    
    // 顯示啟動畫面
    if (showSplash) {
        drawSplashScreen("Balance Bot", "OLED Manager");
        delay(1000);
    }
    
    return true;
}

// 添加頁面
bool OLED_Manager::addPage(DisplayPage* page) {
    if (pageCount >= MAX_PAGES || page == nullptr) {
        return false;
    }
    
    pages[pageCount] = page;
    pageCount++;
    return true;
    }
    
// 設置當前頁面
void OLED_Manager::setPage(int index) {
    if (index >= 0 && index < pageCount) {
        currentPageIndex = index;
    }
}

// 獲取當前頁面索引
int OLED_Manager::getCurrentPageIndex() {
    return currentPageIndex;
}

// 獲取頁面總數
int OLED_Manager::getPageCount() {
    return pageCount;
}

// 切換到下一個頁面
void OLED_Manager::nextPage() {
    if (pageCount <= 0) return;
    
    currentPageIndex = (currentPageIndex + 1) % pageCount;
}

// 切換到上一個頁面
void OLED_Manager::prevPage() {
    if (pageCount <= 0) return;
    
    currentPageIndex = (currentPageIndex + pageCount - 1) % pageCount;
}

// 顯示訊息
void OLED_Manager::displayMessage(const char* line1, const char* line2, unsigned long delay_ms) {
            u8g2.clearBuffer();
            
    u8g2.setFont(u8g2_font_ncenB10_tr);
    if (line1) {
        u8g2.drawStr((128 - u8g2.getStrWidth(line1)) / 2, 24, line1);
    }
    
    if (line2) {
        u8g2.drawStr((128 - u8g2.getStrWidth(line2)) / 2, 44, line2);
    }
    
    u8g2.sendBuffer();
    
    if (delay_ms > 0) {
        delay(delay_ms);
}
}

// 顯示進度條
void OLED_Manager::displayProgress(const char* message, int progress) {
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (message) {
        u8g2.drawStr((128 - u8g2.getStrWidth(message)) / 2, 20, message);
    }
    
    // 繪製進度條
    drawProgressBar(14, 30, 100, 10, progress);
    
    // 顯示百分比
    char percentText[5];
    sprintf(percentText, "%d%%", progress);
    u8g2.drawStr((128 - u8g2.getStrWidth(percentText)) / 2, 55, percentText);
    
    u8g2.sendBuffer();
}

// 更新顯示
void OLED_Manager::update() {
    unsigned long currentMillis = millis();
    
    // 限制更新頻率
    if (currentMillis - lastUpdateTime < updateInterval) {
        return;
    }
    
    lastUpdateTime = currentMillis;
    
    // 確保有頁面可顯示
    if (pageCount > 0 && currentPageIndex >= 0 && currentPageIndex < pageCount) {
        // 更新當前頁面數據
        pages[currentPageIndex]->update();
        
        // 清除緩衝區
        u8g2.clearBuffer();
        
        // 繪製當前頁面
        pages[currentPageIndex]->draw(u8g2);
        
        // 發送緩衝區到顯示器
        u8g2.sendBuffer();
    }
}
// 設置更新間隔
void OLED_Manager::setUpdateInterval(unsigned long interval) {
    updateInterval = interval;
}

// 繪製進度條
void OLED_Manager::drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress) {
    // 邊框
    u8g2.drawFrame(x, y, width, height);
    
    // 進度填充
    uint8_t fillWidth = (progress * (width - 2)) / 100;
    u8g2.drawBox(x + 1, y + 1, fillWidth, height - 2);
}

// 繪製啟動畫面
void OLED_Manager::drawSplashScreen(const char* title, const char* subtitle) {
    // 清除緩衝區
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_ncenB10_tr);
    if (title) {
        u8g2.drawStr((128 - u8g2.getStrWidth(title)) / 2, 15, title);
    }
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (subtitle) {
        u8g2.drawStr((128 - u8g2.getStrWidth(subtitle)) / 2, 35, subtitle);
    }
    
    // 繪製簡單的機器人圖示
    u8g2.drawCircle(64, 50, 8, U8G2_DRAW_ALL);
    u8g2.drawLine(64, 58, 64, 62);
    u8g2.drawLine(64, 62, 58, 68);
    u8g2.drawLine(64, 62, 70, 68);
    
    // 移除這兩行，因為在初始化時還沒有頁面
    // pages[currentPageIndex]->draw(u8g2);
    
    // 發送緩衝區到顯示器
    u8g2.sendBuffer();
}

// 獲取 U8G2 對象引用
U8G2_SH1106_128X64_NONAME_F_HW_I2C& OLED_Manager::getU8G2() {
    return u8g2;
}