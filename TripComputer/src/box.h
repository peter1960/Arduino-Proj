#include <TFT_eSPI.h>

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
    void Draw(TFT_eSPI &tft);
    void DrawText(TFT_eSPI &tft, const char *string, TriState BoxState);
    void TweekY(int);

private:
    int width;
    int height;
    int top;
    int left;
    int fillon = -1;
    int filloff = -1;
    int yTweek = 999;
    TriState lastState = Indeterminate;
    bool FillRect(TFT_eSPI &tft, TriState State);
    int LastColor = TFT_BLACK;
    char lastText[16];
};