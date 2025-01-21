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
pBox::pBox(int left, int top, int width, int height, int fillon, int filloff)
{
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    this->fillon = fillon;
    this->filloff = filloff;
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
    if (lastBoxState != BoxState && BoxState == ButtonOn)
    {
        Serial.println("Box Redraw On");

        tft.fillRect(left + 1, top + 1, width - 2, height - 2, fillon);
        lastBoxState = ButtonOn;
        return true;
    }
    else if (lastBoxState != BoxState && BoxState == ButtonOff)
    {
        Serial.println("Box Redraw Off");
        tft.fillRect(left + 1, top + 1, width - 2, height - 2, filloff);
        lastBoxState = ButtonOff;
        return true;
    }
    return false;
}

void pBox::DrawText(TFT_eSPI &tft, const char *string, TriState BoxState)
{
    //Serial.print("New ");
    //Serial.print(string);
    //Serial.print(" Old ");
    //Serial.println(lastText);
    /* Need to redraw if color changed or text*/
    if (FillRect(tft, BoxState) || strncmp(lastText, string, 16) != 0)
    {
        //Serial.println("State Change");
        if (BoxState == ButtonOn)
        {
            tft.setTextColor(TFT_BLACK); // On Text colour
           // Serial.print(string);
            //Serial.println(" On Text");
        }
        else
        {
            tft.setTextColor(TFT_WHITE); // Off Text colour
            //Serial.print(string);
           // Serial.println("Off Text");
        }
        if (yTweek < 999)
        {
            tft.drawCentreString(string, left + (width / 2), top + (height / 4) + yTweek, 2);
        }
        else
        {
            tft.drawCentreString(string, left + (width / 2), top + (height / 4), 2);
        }
        strncpy(lastText, string, 16);
        lastText[15] = '\0';
    }
}
