#include <TFT_eSPI.h>

#include <box.h>

class pSpeedBox : pBox
{
public:
    pSpeedBox(int left, int top, int width, int height,const char *boxLabel);
    void Draw(TFT_eSPI &tft);
    void Speed(TFT_eSPI &tft, String speed);

private:
    String last_speed = "N/A";
    char p_boxText[10];

};