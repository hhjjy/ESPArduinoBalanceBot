#ifndef DISPLAY_PAGE_H
#define DISPLAY_PAGE_H

#include <U8g2lib.h>

class DisplayPage {
public:
    /**
     * 繪製頁面
     * @param u8g2 U8G2 對象引用
     */
    virtual void draw(U8G2_SH1106_128X64_NONAME_F_HW_I2C& u8g2) = 0;
    
    /**
     * 獲取頁面名稱
     * @return 頁面名稱
     */
    virtual const char* getName() = 0;
    
    /**
     * 更新頁面數據
     * 在繪製之前調用
     */
    virtual void update() = 0;
    
    /**
     * 處理按鈕按下事件
     */
    virtual void handleButtonPress() {};
    
    /**
     * 虛擬析構函數
     */
    virtual ~DisplayPage() {};
};

#endif // DISPLAY_PAGE_H