#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pl_heart.h"
#include "pl_display.h"

void app_main(void)
{

    xTaskCreate(
        led_task,   // Task function
        "led_task", // Name
        2048,       // Stack size
        NULL,       // Parameter
        5,          // Priority
        NULL        // Handle (optional)
    );
    // test_epd();
}