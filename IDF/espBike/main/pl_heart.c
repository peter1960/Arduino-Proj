#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pl_heart.h"
#include "pins.h"
static const char *TAG = "LED_TASK";

void led_task(void *pvParameter)
{
    ESP_LOGI(TAG, "LED task started");
    gpio_set_direction(LED_HEART, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(LED_HEART, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(LED_HEART, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}