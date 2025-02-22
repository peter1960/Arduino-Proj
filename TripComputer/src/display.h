#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <box.h>
#include <speed-box.h>

#define TFT_GREY 0x5AEB
//* these are button dims
#define BX 240
#define BY 5
#define BWIDE 49
#define BHIGH 29
// IP Box
#define IX 0
#define IY 300
#define IWIDE 109
#define IHIGH 20

// Speed Box
#define SX 0
#define SY 1
#define SWIDE 150
#define SHIGH 80

// Status
#define SAT_WIDE 49
#define SAT_LEFT 480-SAT_WIDE
#define SAT_HIGH 29
#define SAT_TOP 45
//RES
#define RES_WIDE 49
#define RES_LEFT 480-SAT_WIDE
#define RES_HIGH 29
#define RES_TOP SAT_TOP+SAT_HIGH



// linear bar sizes
#define BAR_HEIGHT 155
#define BAR_OFFSET 19
#define BAR_WIDTH 36

// Bar Pos
#define BAR_SAT_X 0
#define BAR_SAT_Y 160
#define BAR_SAT_POINTS 9

#define SPEED_ROT 141

class Display
{
private:
    TFT_eSPI tft;
    int old_analog = -999;         // Value last displayed
    int old_digital = -999;        // Value last displayed
    float ltx = 0;                 // Saved x coord of bottom of needle
    uint16_t osx = 120, osy = 120; // Saved x & y coords
    int plotSatLines[BAR_SAT_POINTS + 1];
    int value[6] = {0, 0, 0, 0, 0, 0};
    int old_value[6] = {0, -1, -1, -1, -1, -1};
    bool m_HasLock;
    enum
    {
        SAT_COUNT = 0
    };

    pBox *b_Wifi;
    pBox *b_Rec;
    pBox *b_Lock;
    pBox *b_IP;
    pBox *b_ECU;
    pBox *b_Sats;
    pBox *b_Res;
    pSpeedBox *b_GPSSpeed;
    pSpeedBox *b_ECUkmh;
    pSpeedBox *b_ECURPM;

public:
    Display(void);
    void plotSpeed(int value, byte ms_delay);
    void analogMeter();
    void plotLinearSat(const char *label);
    void plotPointer(void);
    void plotTriangle(int x, int y, uint32_t color);
    void DisplayTime(const char *time);
    void screenLayout();
    void DisplayStat(int sat);
    void Accuracy(int distance);
    void Wifi(bool yes);
    void ipAdress(const char *ip);
    void speed(float);
    void ecu_speed(float);
    void HasLock(bool yes);
    void ECUConnect(bool yes);
    void Rec(bool yes);
    void rpm(float rpm);
    void Sats(int count);
};