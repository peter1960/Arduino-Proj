#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include "peter_display.h"

// #include "peter_adc.h"
#include "peter_flash.h"
#include "peter_buttons.h"
#include "peter_pulse.h"
#include "peter_wheel.h"

static const char *TAG = "SPEEDO";

void speed_producer_task(void *arg)
{
    /*
    Given:
    Diameter = 0.7 m
    Circumference = π * Diameter ≈ 3.1416 * 0.7 ≈ 2.199 m
    Pulse duration = T seconds (time for one rotation)

    Step 1: Speed in m/s
    Speed = Circumference / T
    Speed = 2.199 / T

    Step 2: Convert to km/h
    Speed_kmh = (Speed_mps * 3600) / 1000
    Speed_kmh = (2.199 / T) * 3.6
    Speed_kmh ≈ 7.9164 / T

    */
    /*
    Faking data from interupt on wheel pulse.
    Need to check time between pulses as delay here in not real
     */
    /*
        float s_speed = 0.0f;
        float delay = 198.0f;
        bool IsStopped = false;
        for (;;)
        {

            delay = getValue() * 1.0f;
            if (delay < 6500)
            {
                IsStopped = false;
                s_speed = (WHEEL_CIRCUMFERANCE / (delay / 1000.0f)) * 3.6;
                // Example: compute or read speed from sensor/network
                speed_set(s_speed, WHEEL_CIRCUMFERANCE);
            }
            else
            {
                // was stopped last time then nothing to do.
                // really for the write oddometer & trip to flash.
                if (!IsStopped)
                {
                    delay = 500;
                    speed_set(0.0, 0.0);
                    IsStopped = true;
                    flash_store(100.00, 50.00);
                }
            }
            // ESP_LOGI(TAG, "Speed: %05.2f Delay %f", s_speed, delay);

            vTaskDelay(pdMS_TO_TICKS(delay));
        }
            */
}

void app_main(void)
{
    // setup_adc1();
    //  speed_mutex_init();
    // flash_init();

    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    xTaskCreate(wheel_task, "wheel_task", 2048, NULL, 10, NULL);
    xTaskCreate(avg_task, "avg_task", 2048, NULL, 10, NULL);
    xTaskCreate(pulse_task, "pulse_task", 4096, NULL, 5, NULL);
}