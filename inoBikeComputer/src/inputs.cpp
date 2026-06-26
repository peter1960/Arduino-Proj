#include <atomic>
#include "Arduino.h"
#include "inputs.h"
#include "pins.h"

std::atomic<bool> recordState{false};

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

bool isRecord()
{
    return recordState.load();
}
