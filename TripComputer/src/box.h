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
    // pBox(int left, int top, int width, int height, int fill);
    pBox(int left, int top, int width, int height, int fillon, int filloff, const char *string);
    void DrawText(TFT_eSPI &tft, TriState BoxState);
    void TweekY(int);

protected:
    int width;
    int height;
    int top;
    int left;
    char p_boxText[16];
    virtual void Draw(TFT_eSPI &tft);


private:
    int fillon = -1;
    int filloff = -1;
    int yTweek = 999;
    TriState lastState = Indeterminate;
    bool FillRect(TFT_eSPI &tft, TriState State);
    // int LastColor = TFT_BLACK;
    TriState lastBoxState = Indeterminate;
    char lastText[16];
};

#endif
