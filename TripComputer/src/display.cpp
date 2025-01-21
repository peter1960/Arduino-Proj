#include <display.h>

Display::Display()
{

    tft.setRotation(1);
    tft.init();
    tft.fillScreen(TFT_BLACK);
    // put your setup code here, to run once:
    // tft.fillRect(0, 0, 239, 126, TFT_BLACK);
    // tft.fillRect(5, 3, 230, 119, TFT_WHITE);
    value[SAT_COUNT] = 0;
    b_Wifi = new pBox(BX, BY, BWIDE, BHIGH, TFT_GREEN, TFT_RED,"Wifi");
    b_Rec = new pBox(BX + BWIDE, BY, BWIDE, BHIGH, TFT_GREEN, TFT_RED,"Rec");
    b_Lock = new pBox(BX + BWIDE + BWIDE, BY, BWIDE, BHIGH, TFT_GREEN, TFT_RED,"Lock");
    b_ECU = new pBox(BX + BWIDE + BWIDE + BWIDE, BY, BWIDE, BHIGH, TFT_GREEN, TFT_RED,"ECU");
    b_IP = new pBox(IX, IY, IWIDE, IHIGH, TFT_GREEN, TFT_BLACK,"IP");
    b_IP->TweekY(-3);
    b_GPSSpeed = new pSpeedBox(SX, SY, SWIDE, SHIGH,"g-kmh");
    b_ECUkmh = new pSpeedBox(SX, SY + SHIGH, SWIDE, SHIGH,"kmh");
    b_ECURPM = new pSpeedBox(SX, SY + SHIGH + SHIGH, SWIDE, SHIGH,"rpm");
}
void Display::plotSpeed(int value, byte ms_delay)
{
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    char buf[8];
    dtostrf(value, 4, 0, buf);
    tft.drawRightString(buf, 40, 119 - 20, 2);

    if (value < 0)
        value = 0; // Limit value to emulate needle end stops
    if (value > 35)
        value = 35;

    // Move the needle util new value reached
    while (!(value == old_analog))
    {
        if (old_analog < value)
            old_analog++;
        else
            old_analog--;

        if (ms_delay == 0)
        {
            old_analog = value; // Update immediately id delay is 0
        }
        float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
        // Calcualte tip of needle coords
        float sx = cos(sdeg * 0.0174532925);
        float sy = sin(sdeg * 0.0174532925);

        // Calculate x delta of needle start (does not start at pivot point)
        float tx = tan((sdeg + 90) * 0.0174532925);

        // Erase old needle image
        tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_WHITE);
        tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_WHITE);
        tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_WHITE);

        // Re-plot text under needle
        tft.setTextColor(TFT_BLACK);
        tft.drawCentreString("Kmh", 120, 70, 4); // // Comment out to avoid font 4

        // Store new needle end coords for next erase
        ltx = tx;
        osx = sx * 98 + 120;
        osy = sy * 98 + 140;

        // Draw the needle in the new postion, magenta makes needle a bit bolder
        // draws 3 lines to thicken needle
        tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_RED);
        tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_MAGENTA);
        tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_RED);

        // Slow needle down slightly as it approaches new postion
        if (abs(old_analog - value) < 10)
            ms_delay += ms_delay / 5;

        // Wait before next update
        delay(ms_delay);
    }
}
void Display::DisplayTime(const char *time)
{
    static String xx = "xx";
    if (xx.compareTo(time) == 0)
    {
        return;
    }
    tft.setTextColor(TFT_BLACK);
    tft.drawString(xx, 10, 135, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(time, 10, 135, 2);
    xx = time;
}
void Display::DisplayStat(int sat)
{
    value[SAT_COUNT] = sat;
}
void Display::screenLayout()
{
    Wifi(false);
    Rec(false);
    HasLock(false);
    ECUConnect(false);
    b_GPSSpeed->Draw(tft);
    b_ECUkmh->Draw(tft);
    b_ECURPM->Draw(tft);
}
void Display::Wifi(bool yes)
{
    if (yes)
    {
    b_Wifi->DrawText(tft, ButtonOn);
    }
    else
    {
    b_Wifi->DrawText(tft,  ButtonOff);
    }
}
void Display::Rec(bool yes)
{
    if (yes)
    {
        b_Rec->DrawText(tft, ButtonOn);
    }
    else
    {
        b_Rec->DrawText(tft,  ButtonOff);
    }
}
void Display::ipAdress(const char *ip)
{
    b_IP->DrawText(tft, ButtonOff);
}
void Display::HasLock(bool yes)
{
    if (yes)
    {
        b_Lock->DrawText(tft, ButtonOn);
    }
    else
    {
        b_Lock->DrawText(tft, ButtonOff);
    }
}
void Display::ECUConnect(bool yes)
{
    if (yes)
    {
        b_ECU->DrawText(tft,  ButtonOn);
    }
    else
    {
        b_ECU->DrawText(tft, ButtonOff);
    }
}
void Display::speed(float act_speed)
{
    b_GPSSpeed->Speed(tft, act_speed);
}
void Display::ecu_speed(float avg_speed)
{
    b_ECUkmh->Speed(tft, avg_speed);
}
void Display::rpm(float rpm)
{
    b_ECURPM->Speed(tft, rpm);
}


void Display::analogMeter()
{
    // Meter outline
    tft.fillRect(0, 0, 239, 126, TFT_GREY);
    tft.fillRect(5, 3, 230, 119, TFT_WHITE);

    tft.setTextColor(TFT_BLACK); // Text colour

    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
    // will use 99% range to max of 33kmh
    for (int i = 0; i < 100; i += 3)
    {
        // Long scale tick length
        int tl = 15;

        // Coodinates of tick to draw
        float sx = cos((i - SPEED_ROT) * 0.0174532925);
        float sy = sin((i - SPEED_ROT) * 0.0174532925);
        uint16_t x0 = sx * (100 + tl) + 120;
        uint16_t y0 = sy * (100 + tl) + 140;
        uint16_t x1 = sx * 100 + 120;
        uint16_t y1 = sy * 100 + 140;

        // Coordinates of next tick for zone fill
        float sx2 = cos((i + 5 - SPEED_ROT) * 0.0174532925);
        float sy2 = sin((i + 5 - SPEED_ROT) * 0.0174532925);
        int x2 = sx2 * (100 + tl) + 120;
        int y2 = sy2 * (100 + tl) + 140;
        int x3 = sx2 * 100 + 120;
        int y3 = sy2 * 100 + 140;

        // Yellow zone limits
        // if (i >= -50 && i < 0) {10
        //  tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
        //  tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
        //}

        // Green zone limits
        if (i >= 15 && i < 25)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        }

        // Orange zone limits
        if (i >= 25 && i < 33)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        }

        // Short scale tick length
        if (i % 25 != 0)
            tl = 8;

        // Recalculate coords incase tick lenght changed
        x0 = sx * (100 + tl) + 120;
        y0 = sy * (100 + tl) + 140;
        x1 = sx * 100 + 120;
        y1 = sy * 100 + 140;

        // Draw tick
        tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

        // Check if labels should be drawn, with position tweaks
        if (i % 25 == 0)
        {
            // Calculate label positions
            x0 = sx * (100 + tl + 10) + 120;
            y0 = sy * (100 + tl + 10) + 140;
            switch (i / 10)
            {
            case 0:
                tft.drawCentreString("0", x0, y0 - 12, 2);
                break;
            case 1:
                tft.drawCentreString("25", x0, y0 - 9, 2);
                break;
            case 2:
                tft.drawCentreString("50", x0, y0 - 6, 2);
                break;
            case 3:
                tft.drawCentreString("75", x0, y0 - 9, 2);
                break;
            case 4:
                tft.drawCentreString("100", x0, y0 - 12, 2);
                break;
            }
        }

        // Now draw the arc of the scale
        sx = cos((i + 5 - SPEED_ROT) * 0.0174532925);
        sy = sin((i + 5 - SPEED_ROT) * 0.0174532925);
        x0 = sx * 100 + 120;
        y0 = sy * 100 + 140;
        // Draw scale arc, don't draw the last part
        if (i < 50)
            tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }

    tft.drawString("%RH", 5 + 230 - 40, 119 - 20, 2); // Units at bottom right
    tft.drawCentreString("Kmh", 120, 70, 4);          // Comment out to avoid font 4
    tft.drawRect(5, 3, 230, 119, TFT_BLACK);          // Draw bezel line

    plotSpeed(0, 0); // Put meter needle at 0
}
// #########################################################################
//  Draw a linear meter on the screen
// #########################################################################
void Display::plotLinearSat(const char *label)
{

    tft.drawRect(BAR_SAT_X, BAR_SAT_Y, BAR_WIDTH, BAR_HEIGHT, TFT_GREY);
    tft.fillRect(BAR_SAT_X + 2, BAR_SAT_Y + BAR_OFFSET, BAR_WIDTH - 3, BAR_HEIGHT - (BAR_OFFSET * 2), TFT_WHITE);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawCentreString(label, BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + 2, 2);

    for (int i = 0; i <= BAR_SAT_POINTS; i += 1)
    {
        tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (i * 11), 6, TFT_BLACK);
        // save in reverse order so vale matches location
        plotSatLines[BAR_SAT_POINTS - i] = BAR_SAT_Y + 27 + (i * 11);
    }

    tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (0 * 11), 10, TFT_BLACK);
    tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (9 * 11), 10, TFT_BLACK);

    // plotTriangle(BAR_SAT_X, BAR_SAT_Y);

    tft.drawCentreString("---", BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + BAR_HEIGHT - (BAR_OFFSET), 2);
}

void Display::plotTriangle(int x, int y, uint32_t color)
{

    tft.fillTriangle(x + 3, y, x + 3 + 16, y, x + 3, y - 5, color);
    tft.fillTriangle(x + 3, y, x + 3 + 16, y, x + 3, y + 5, color);
}
// #########################################################################
//  Adjust 6 linear meter pointer positions
// #########################################################################
void Display::plotPointer(void)
{
    int dy = 187;
    byte pw = 16;

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    char buf[8];
    dtostrf(value[SAT_COUNT], 4, 0, buf);
    tft.drawCentreString(buf, BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + BAR_HEIGHT - (BAR_OFFSET), 2);
    if (value[SAT_COUNT] != old_value[SAT_COUNT])
    {
        plotTriangle(BAR_SAT_X, plotSatLines[old_value[SAT_COUNT]], TFT_WHITE);
        plotTriangle(BAR_SAT_X, plotSatLines[value[SAT_COUNT]], TFT_RED);
        old_value[SAT_COUNT] = value[SAT_COUNT];
    }
    // plotTriangle(BAR_SAT_X, plotSatLines[0]);

    return;
    // Move the 6 pointers one pixel towards new value
    for (int i = 0; i < 6; i++)
    {
        char buf[8];
        dtostrf(value[i], 4, 0, buf);
        tft.drawRightString(buf, i * 40 + 36 - 5, 187 - 27 + 155 - 18, 2);

        int dx = 3 + 40 * i;
        if (value[i] < 0)
        {
            value[i] = 0; // Limit value to emulate needle end stops
        }
        if (i == SAT_COUNT)
        {
            // pw = 25;
            if (value[i] > 9)
            {
                value[i] = 9;
            }
        }
        else
        {
            // pw =16;
            if (value[i] > 100)
            {
                value[i] = 100;
            }
        }
        while (!(value[i] == old_value[i]))
        {
            dy = 187 + 100 - old_value[i];
            if (old_value[i] > value[i])
            {
                tft.drawLine(dx, dy - 5, dx + pw, dy, TFT_WHITE);
                old_value[i]--;
                tft.drawLine(dx, dy + 6, dx + pw, dy + 1, TFT_RED);
            }
            else
            {
                tft.drawLine(dx, dy + 5, dx + pw, dy, TFT_WHITE);
                old_value[i]++;
                tft.drawLine(dx, dy - 6, dx + pw, dy - 1, TFT_RED);
            }
        }
    }
}
