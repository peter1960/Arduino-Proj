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
    int l = strlen(boxText);
    strncpy(p_boxText, boxText, l);
    // terminate the string
    p_boxText[l] = '\0';
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
bool pBox::FillRect(TFT_eSPI &tft, TriState BoxState)
{
    tft.drawRect(left, top, width, height, TFT_WHITE); // Draw bezel line
    if (lastBoxState != BoxState && BoxState == ButtonOn)
    {
        Serial.print(p_boxText);
        Serial.println(" Box Redraw On");

        tft.fillRect(left + 1, top + 1, width - 2, height - 2, fillon);
        lastBoxState = ButtonOn;
        return true;
    }
    else if (lastBoxState != BoxState && BoxState == ButtonOff)
    {
        Serial.print(p_boxText);
        Serial.println(" Box Redraw Off");
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
            Serial.println("Text On");
            tft.setTextColor(TFT_BLACK); // On Text colour
        }
        else
        {
            Serial.println("Text Off");
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
