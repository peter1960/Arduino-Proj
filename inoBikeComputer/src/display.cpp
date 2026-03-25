#include "pins.h"
#include "display.h"
#include <RTOS.h>
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include "rtc-time.h"

// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,BUSY=4,RES(RST)=17,DC=16
#define CS_PIN EPD_CS_PIN
#define BUSY_PIN EPD_BUSY_PIN
#define RES_PIN EPD_RES_PIN
#define DC_PIN EPD_DC_PIN

// Show boxes for screen testing
#define LAYOUT_BOXES 1

// 1.54'' EPD Module
// GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEH0154D67 200x200, SSD1681

// 2.13'' EPD Module - b/w 122x250, SSD1680
GxEPD2_BW<GxEPD2_213_BN, GxEPD2_213_BN::HEIGHT> display(GxEPD2_213_BN(/*CS=5*/ CS_PIN, /*DC=*/DC_PIN, /*RES=*/RES_PIN, /*BUSY=*/BUSY_PIN)); // DEPG0213BN 122x250, SSD1680
// GxEPD2_3C<GxEPD2_213_Z98c, GxEPD2_213_Z98c::HEIGHT> display(GxEPD2_213_Z98c(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEY0213Z98 122x250, SSD1680

// 2.9'' EPD Module
// GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // DEPG0290BS 128x296, SSD1680
// GxEPD2_3C<GxEPD2_290_C90c, GxEPD2_290_C90c::HEIGHT> display(GxEPD2_290_C90c(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEM029C90 128x296, SSD1680

// 3.7'' EPD Module
// GxEPD2_BW<GxEPD2_370_GDEY037T03, GxEPD2_370_GDEY037T03::HEIGHT> display(GxEPD2_370_GDEY037T03(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEY037T03 240x416, UC8253

// 4.2'' EPD Module
// GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // 400x300, SSD1683
// GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> display(GxEPD2_420c_GDEY042Z98(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // 400x300, SSD1683

void myTask(void *pvParameters);

void setupDisplay()
{

    // Create task
    xTaskCreate(
        myTask,    // Function
        "My Task", // Name
        2048,      // Stack size
        NULL,      // Parameters
        1,         // Priority
        NULL       // Task handle
    );
}

void show_time(const char *text)
{
    uint16_t hwx = 0;
    uint16_t hwy = 0;
    uint16_t wide = 64 + 8;
    uint16_t high = 16 + 8;
    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
#if LAYOUT_BOXES
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
#endif
        display.setCursor(hwx + 7, hwy + 19);
        display.print(text);
    } while (display.nextPage());
}
void myTask(void *pvParameters)
{
    display.init(0, true, 50, false);
    display.setRotation(1);
    // display.setFont(&FreeMonoBold9pt7b);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    // display.clearScreen(GxEPD_BLACK);
    display.clearScreen();
    show_time("00:00");
    display.hibernate();

    while (true)
    {
        DateTime now = fetchRTCTime();

        vTaskDelay(pdMS_TO_TICKS(1000));
        show_time((String(now.hour()) + ":" + String(now.minute())).c_str());
    }
}