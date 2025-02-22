#include <box.h>

/*
pBox::pBox(int left, int top, int width, int height, int fill)
{
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    this->fillon = fillon;
}
*/

pBox::pBox(int left, int top, int width, int height, int fillon, int filloff, const char *boxText)
{
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    this->fillon = fillon;
    this->filloff = filloff;
    p_boxText = boxText;
}

void pBox::TweekY(int tweek)
{
    this->yTweek = tweek;
}
void pBox::Draw(TFT_eSPI &tft)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    FillRect(tft, ButtonOff);
}

void pBox::DrawJustText(TFT_eSPI &tft, String label)
{
    FillRect(tft, ButtonOff);
    if (label != p_boxText)
    {
        Serial.println("Print Label");
        tft.setTextColor(filloff);
        tft.drawCentreString(p_boxText, left + (width / 2), top + (height / 4), 2);
        p_boxText = label;
        tft.setTextColor(TFT_WHITE);
        tft.drawCentreString(p_boxText, left + (width / 2), top + (height / 4) , 2);
    }
}

bool pBox::FillRect(TFT_eSPI &tft, TriState BoxState)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    if (lastBoxState != BoxState && BoxState == ButtonOn)
    {
#ifdef PL_DEBUG_DISPLAY

        Serial.print(p_boxText);
        Serial.println(" Box Redraw On");
#endif
        tft.fillRect(left + 1, top + 1, width - 2, height - 2, fillon);
        lastBoxState = ButtonOn;
        return true;
    }
    else if (lastBoxState != BoxState && BoxState == ButtonOff)
    {
#ifdef PL_DEBUG_DISPLAY
        Serial.print(p_boxText);
        Serial.println(" Box Redraw Off");
#endif

        tft.fillRect(left + 1, top + 1, width - 2, height - 2, filloff);
        lastBoxState = ButtonOff;
        return true;
    }
    return false;
}
void pBox::DrawText(TFT_eSPI &tft, TriState BoxState)
{
    // Serial.print("New ");
    // Serial.print(string);
    // Serial.print(" Old ");
    // Serial.println(lastText);
    /* Need to redraw if color changed or text*/
    if (FillRect(tft, BoxState))
    {
        // Serial.println("State Change");
        if (BoxState == ButtonOn)
        {
#ifdef PL_DEBUG_DISPLAY
            Serial.println("Text On");
#endif

            tft.setTextColor(TFT_BLACK); // On Text colour
        }
        else
        {
#ifdef PL_DEBUG_DISPLAY
            Serial.println("Text Off");
#endif

            tft.setTextColor(TFT_WHITE); // Off Text colour
        }
        if (yTweek < 999)
        {
            tft.drawCentreString(p_boxText, left + (width / 2), top + (height / 4) + yTweek, 2);
        }
        else
        {
            tft.drawCentreString(p_boxText, left + (width / 2), top + (height / 4), 2);
        }
    }
}
