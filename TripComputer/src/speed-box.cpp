#include <speed-box.h>

pSpeedBox::pSpeedBox(int left, int top, int width, int height) : pBox(left, top, width, height, TFT_BLACK, TFT_BLACK,"")
{
}

void pSpeedBox::Draw(TFT_eSPI &tft)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    // FillRect(tft, ButtonOff);
    Speed(tft, 0);
}
void pSpeedBox::Speed(TFT_eSPI &tft, float speed)
{
    char buffer[50];
    if (speed != last_speed)
    {
        if (last_speed != -1)
        {
            sprintf(buffer, "%.0f", last_speed);
            tft.setTextColor(TFT_BLACK);
            tft.drawRightString(buffer, left + width - 5, top + (height / 4), 7);
        }
        last_speed = speed;
        sprintf(buffer, "%.0f", last_speed);
        tft.setTextColor(TFT_GREEN);
        tft.drawRightString(buffer, left + width - 5, top + (height / 4), 7);
    }
}
