#include <Arduino.h>
#include "pulses.h"
#include "inputs.h"
#include "pins.h"

void setupPulses()
{

    /**
     * Create a FreeRTOS task for generating wheel sensor pulses.
     */
    xTaskCreate(
        pulseTask,    // Function
        "Pulse Task", // Name
        2048,         // Stack size
        NULL,         // Parameters
        1,            // Priority
        NULL          // Task handle
    );
    /**
     * Create a FreeRTOS task for saving distance data to persistent storage.
     */
    xTaskCreate(
        saveTask,              // Function
        "Store Distance Task", // Name
        2048,                  // Stack size
        NULL,                  // Parameters
        1,                     // Priority
        NULL                   // Task handle
    );
}
/** 
 * Pulse task function (fake wheel sensor pulse generator)
 * This task is responsible for generating pulses on the WHEEL_SENSOR_PIN to simulate wheel rotations. It runs indefinitely, generating a pulse every second.
 * The task first loads the distance from persistent storage, then enters an infinite loop where it waits for 1 second, sets the WHEEL_SENSOR_PIN to HIGH for 100 milliseconds, and then sets it back to LOW. This simulates a wheel rotation pulse.
 * The task uses vTaskDelay to yield control to other tasks and avoid blocking the CPU. The pinMode for WHEEL_SENSOR_PIN is set to OUTPUT before generating the pulse.
 */
void pulseTask(void *pvParameters)
{
    loadDistance();
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        pinMode(WHEEL_SENSOR_PIN, OUTPUT);
        digitalWrite(WHEEL_SENSOR_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(WHEEL_SENSOR_PIN, LOW);
    }
}
