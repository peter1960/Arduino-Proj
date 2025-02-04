#include <speed-box.h>

pSpeedBox::pSpeedBox(int left, int top, int width, int height, const char *boxLabel) : pBox(left, top, width, height, TFT_BLACK, TFT_BLACK, "")
{
    p_boxLabel = boxLabel;
}

void pSpeedBox::Draw(TFT_eSPI &tft)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    // FillRect(tft, ButtonOff);
    //Speed(tft, "0");
}
void pSpeedBox::Speed(TFT_eSPI &tft, String speed)
{
    char buffer[50];
    if (speed != last_speed)
    {
        tft.setTextColor(TFT_BLACK);
        tft.drawRightString(last_speed, left + width - 5, top + (height / 4), 7);
        last_speed = speed;
        tft.setTextColor(TFT_GREEN);
        tft.drawRightString(last_speed, left + width - 5, top + (height / 4), 7);
        tft.drawString(p_boxLabel, left + 2, top + 1, 2);
    }
}
