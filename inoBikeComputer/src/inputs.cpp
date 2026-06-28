#include <atomic>
#include "Arduino.h"
#include "inputs.h"
#include "pins.h"

#include <Preferences.h>
Preferences prefs;


std::atomic<bool> recordState{false};
std::atomic<int16_t> distanceState{0};
std::atomic<int16_t> tripState{0};

void IRAM_ATTR recordISR()
{

    static uint32_t lastMillis = 0;
    uint32_t now = millis();
    if (now - lastMillis > 50)
    {
        bool currentState = recordState.load();
        recordState.store(!currentState);
        lastMillis = now;
    }
}

void IRAM_ATTR resetISR()
{

    static uint32_t lastMillis = 0;
    uint32_t now = millis();
    if (now - lastMillis > 50)
    {
        resetTripDistance();
        lastMillis = now;
    }
}

void IRAM_ATTR wheelISR()
{

    static uint32_t lastMillis = 0;
    uint32_t now = millis();
    if (now - lastMillis > 50)
    {
        addTripDistance(1);
        lastMillis = now;
    }
}

bool isRecord()
{
    return recordState.load();
}

void addDistance(int16_t distance)
{
    distanceState.fetch_add(distance);
}   

int16_t getDistance()
{
    return distanceState.load();
}   

int16_t getTripDistance()
{
    return tripState.load();
}
void resetTripDistance()
{
    tripState.store(0);
}
void addTripDistance(int16_t distance)
{
    addDistance(distance);
    tripState.fetch_add(distance);
}

void loadDistance() {
    prefs.begin("app", false);
    int16_t saved = prefs.getShort("distance", 0);
    distanceState.store(saved);
    prefs.end();
}

void saveTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        //prefs.begin("app", false);
        //prefs.putShort("distance", distanceState.load());
        //prefs.end();
    }
}
