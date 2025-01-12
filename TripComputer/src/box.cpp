#include <box.h>

pBox::pBox(int left, int top, int width, int height, int fill)
{
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    this->fillon = fillon;
}
pBox::pBox(int left, int top, int width, int height, int fillon, int filloff)
{
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    this->fillon = fillon;
    this->filloff = filloff;
}

void pBox::Draw(TFT_eSPI &tft)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    FillRect(tft, false);
}
void pBox::FillRect(TFT_eSPI &tft, bool On)
{
    if (On)
    {
        tft.fillRect(left + 1, top + 1, width - 2, height - 2, fillon);
    }
    else
    {
        tft.fillRect(left + 1, top + 1, width - 2, height - 2, filloff);
    }
}

void pBox::DrawText(TFT_eSPI &tft, const char *string, bool On)
{
    // tft.loadFont(AA_FONT_SMALL); // Must load the font first
    FillRect(tft, On);
    if (On)
    {
        tft.setTextColor(TFT_BLACK); // Text colour
    }
    else
    {
        tft.setTextColor(TFT_WHITE); // Text colour
    }
    tft.drawCentreString(string, left + (width / 2), top + (height / 4), 2);
}