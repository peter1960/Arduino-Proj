#ifndef BOX_H
#define BOX_H
#include <TFT_eSPI.h>
#include <box.h>

enum TriState
    {
        ButtonOn,
        ButtonOff,
        Indeterminate
    };


class pBox
{
public:
    

    pBox(int left, int top, int width, int height, int fill);
    pBox(int left, int top, int width, int height, int fillon, int filloff);
    virtual void Draw(TFT_eSPI &tft);
    void DrawText(TFT_eSPI &tft, const char *string, TriState BoxState);
    void TweekY(int);

protected:
    int width;
    int height;
    int top;
    int left;

private:
    int fillon = -1;
    int filloff = -1;
    int yTweek = 999;
    TriState lastState = Indeterminate;
    bool FillRect(TFT_eSPI &tft, TriState State);
    int LastColor = TFT_BLACK;
    char lastText[16];
};

#endif
