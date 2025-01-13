#include <TFT_eSPI.h>

#include <box.h>

class pSpeedBox : pBox
{
public:
    pSpeedBox(int left, int top, int width, int height);
    void Draw(TFT_eSPI &tft);
    void Speed(TFT_eSPI &tft, float speed);

private:
    float last_speed = -1;
};