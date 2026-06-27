#include <Arduino.h>
#include "pulses.h"
#include "inputs.h"

void setupPulses()
{

    // Create task
    xTaskCreate(
        pulseTask,    // Function
        "Pulse Task", // Name
        2048,         // Stack size
        NULL,         // Parameters
        1,            // Priority
        NULL          // Task handle
    );
    xTaskCreate(
        saveTask,              // Function
        "Store Distance Task", // Name
        2048,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority
        NULL                   // Task handle
    );
}

void pulseTask(void *pvParameters)
{
    loadDistance();
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        addTripDistance(1);
    }
}
