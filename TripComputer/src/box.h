#include <TFT_eSPI.h>

class pBox
{
public:
    int width;
    int height;
    int top;
    int left;
    int fillon = -1;
    int filloff = -1;
    pBox(int left, int top, int width, int height, int fill);
    pBox(int left, int top, int width, int height, int fillon, int filloff);
    void Draw(TFT_eSPI &tft);
    void DrawText(TFT_eSPI &tft, const char *string, bool On);

private:
    void FillRect(TFT_eSPI &tft, bool On);
};