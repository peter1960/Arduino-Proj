#include "pins.h"
#include "display.h"
#include "inputs.h"
#include <RTOS.h>
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeSansBoldOblique24pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include "rtc-time.h"

GFXcanvas1 canvas(122, 250);

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
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT>
    display(GxEPD2_213_B74(/*CS=5*/ CS_PIN, /*DC=*/DC_PIN, /*RES=*/RES_PIN, /*BUSY=*/BUSY_PIN)); // DEPG0213BN 122x250, SSD1680
// GxEPD2_3C<GxEPD2_213_Z98c, GxEPD2_213_Z98c::HEIGHT> display(GxEPD2_213_Z98c(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEY0213Z98 122x250, SSD1680

// 2.9'' EPD Module
// GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // DEPG0290BS 128x296, SSD1680
// GxEPD2_3C<GxEPD2_290_C90c, GxEPD2_290_C90c::HEIGHT> display(GxEPD2_290_C90c(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEM029C90 128x296, SSD1680

// 3.7'' EPD Module
// GxEPD2_BW<GxEPD2_370_GDEY037T03, GxEPD2_370_GDEY037T03::HEIGHT> display(GxEPD2_370_GDEY037T03(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // GDEY037T03 240x416, UC8253

// 4.2'' EPD Module
// GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // 400x300, SSD1683
// GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> display(GxEPD2_420c_GDEY042Z98(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RES_PIN, /*BUSY=*/ BUSY_PIN)); // 400x300, SSD1683

void displayTask(void *pvParameters);

void setupDisplay()
{
    // Create task
    xTaskCreate(
        displayTask,    // Function
        "Display Task", // Name
        2048,           // Stack size
        NULL,           // Parameters
        1,              // Priority
        NULL            // Task handle
    );
}

void show_distance()
{
    static int16_t lastDistance = -1;
    int16_t distance = getDistance();
    if (distance == lastDistance)
    {
        return; // No change in distance, no need to update display
    }
    char buf[10];

    uint16_t hwx = 0;
    uint16_t hwy = (8 * 12) - 1;
    uint16_t wide = 8 * 12;
    uint16_t high = 8 * 3;

    sprintf(buf, "%07.2f", (distance * WHEEL_CIRCUMFERANCE) / 1000.0f);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
        // Clear the update area
        display.writeFillRect(hwx, hwy, wide, high, GxEPD_WHITE);
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(hwx + 7, hwy + 19);
        display.print(buf);
    } while (display.nextPage());
}
void show_tripdistance()
{
    static int16_t lastTripDistance = -1;
    int16_t tripDistance = getTripDistance();
    if (tripDistance == lastTripDistance)
    {
        return; // No change in trip distance, no need to update display
    }
    char buf[10];

    uint16_t hwx = (8 * 12) - 1;
    uint16_t hwy = (8 * 12) - 1;
    uint16_t wide = 8 * 12;
    uint16_t high = 8 * 3;

    sprintf(buf, "%07.2f", (tripDistance * WHEEL_CIRCUMFERANCE) / 1000.0f);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
        // Clear the update area
        display.writeFillRect(hwx, hwy, wide, high, GxEPD_WHITE);
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(hwx + 7, hwy + 19);
        display.print(buf);
    } while (display.nextPage());
}

/**
 * Show recording status on the display
 */
void show_rec()
{
    static bool lastRecordState = false;
    bool recordState = isRecord();

    uint16_t hwx = (8 * 26) - 1;
    uint16_t hwy = 0;
    uint16_t wide = 8 * 5;
    uint16_t high = 8 * 3;
    display.setFont(&FreeSans9pt7b);
    display.setTextColor(GxEPD_BLACK);

    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
        if (!recordState)
        {
            display.fillRect(hwx, hwy, wide, high, GxEPD_WHITE);
            display.setTextColor(GxEPD_BLACK);
        }
        else
        {
            display.fillRect(hwx, hwy, wide, high, GxEPD_BLACK);
            display.setTextColor(GxEPD_WHITE);
        }
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
        display.setCursor(hwx + 5, hwy + 17);
        display.print("Rec");

    } while (display.nextPage());
}

void show_speed()
{

    uint16_t hwx = 0;
    uint16_t hwy = (8 * 4) - 1;
    uint16_t wide = 8 * 22;
    uint16_t high = 8 * 5;
    display.setFont(&FreeSansBoldOblique24pt7b);
    display.setTextColor(GxEPD_BLACK);

    char buf[10];
    float speed = (random(0, 4050) / 100.0f);
    sprintf(buf, "%5.1f", speed);

    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
        if (!isRecord())
        {
            display.fillRect(hwx, hwy, wide, high, GxEPD_WHITE);
            display.setTextColor(GxEPD_BLACK);
        }
        else
        {
            display.fillRect(hwx, hwy, wide, high, GxEPD_BLACK);
            display.setTextColor(GxEPD_WHITE);
        }
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
        display.setCursor(hwx + 5, hwy + high - 5);
        display.print(buf);

        //        {
        //            display.drawLine(hwx + 2, hwy + 2, hwx + wide - 2, hwy + high - 2, GxEPD_BLACK);
        //       }

    } while (display.nextPage());
}

void show_time()
{
    DateTime now = fetchRTCTime();

    char buf[6];
    sprintf(buf, "%02d:%02d", now.hour(), now.minute());

    uint16_t hwx = 0;
    uint16_t hwy = 0;
    uint16_t wide = 64 + 8;
    uint16_t high = 16 + 8;
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);

    display.setPartialWindow(hwx, hwy, wide, high);
    display.firstPage();
    do
    {
        // Clear the update area
        display.writeFillRect(hwx, hwy, wide, high, GxEPD_WHITE);
#if LAYOUT_BOXES
        display.drawRect(hwx, hwy, wide, high, GxEPD_BLACK);
#endif
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(hwx + 7, hwy + 19);
        display.print(buf);
    } while (display.nextPage());
}
void displayTask(void *pvParameters)
{
    uint8_t refresh_count = 0;

    display.init(0, true, 5, false);
    // display.init(115200, true, 2, false);
    display.setRotation(1);
    // display.setFont(&FreeMonoBold9pt7b);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(GxEPD_BLACK);
    // display.clearScreen(GxEPD_BLACK);
    display.setFullWindow();
    display.clearScreen();
    display.hibernate();

    while (true)
    {
        if (++refresh_count >= 600)
        {
            refresh_count = 0;
            display.setFullWindow();
            display.firstPage();
            do
            {
                display.clearScreen();
                display.fillScreen(GxEPD_WHITE);
            } while (display.nextPage());
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        show_time();
        show_rec();
        show_distance();
        show_tripdistance();
        show_speed();
        display.powerOff();
    }
}