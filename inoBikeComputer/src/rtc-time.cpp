#include "RTClib.h"
#include "rtc-time.h"
#include <time.h>
RTC_DS3231 rtc;
bool rtcInitialized = false;
void startRTC()
{
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
    }
    else
    {
        Serial.println("RTC found");
        if (rtc.lostPower())
        {
            Serial.println("RTC lost power, setting the time!");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        else
        {
            Serial.println("RTC has power");
        }
        rtcInitialized = true;
    }
}
DateTime fetchRTCTime()
{
    if (!rtcInitialized)
    {
        Serial.println("RTC not initialized, returning default time");
        return DateTime();
    }
    return rtc.now();
}