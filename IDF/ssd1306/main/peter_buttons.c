#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include "peter_buttons.h"
#include "pins.h"

static const char *TAG = "BUTTON";

//////////////////////
portMUX_TYPE g_dataLockButton = portMUX_INITIALIZER_UNLOCKED;
/*
     data below is locked on read/write
*/

void button_lock(void)
{
    portENTER_CRITICAL(&g_dataLockButton);
}

void button_unlock(void)
{
    portEXIT_CRITICAL(&g_dataLockButton);
}

void button_task(void *args)
{
    ESP_LOGI(TAG, "Initialize Button");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PAGE), // Select GPIO 4
        .mode = GPIO_MODE_INPUT,               // Set as input
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };
    gpio_config(&io_conf);
    int PageCount = 0;
    for (;;)
    {
        int level = gpio_get_level(BUTTON_PAGE);
        if (level == 0)
        {
            PageCount++;
        }
        else
        {
            PageCount = 0;
        }
        if (PageCount == 4)
        {
            ESP_LOGI(TAG, "Button Pressed!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}