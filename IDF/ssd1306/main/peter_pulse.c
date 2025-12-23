
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "pins.h"
#include "peter_pulse.h"

#define PULSE_WIDTH_MS 10  // Width of the pulse (ms)
#define MIN_PERIOD_MS 100  // Minimum period
#define MAX_PERIOD_MS 4000 // Maximum period

#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12 // 12-bit: 0..4095
#define ADC_ATTEN_CFG ADC_ATTEN_DB_12  // ~ up to 3.3V input

static const char *TAG = "ADC READ";

static inline uint32_t map_adc_to_ms(int raw12)
{
    if (raw12 < 0)
        raw12 = 0;
    if (raw12 > 4095)
        raw12 = 4095;

    // Linear map: 0 -> MIN_PERIOD_MS, 4095 -> MAX_PERIOD_MS
    // Use 64-bit math to avoid overflow
    uint64_t span = (uint64_t)(MAX_PERIOD_MS - MIN_PERIOD_MS);
    return MIN_PERIOD_MS + (uint32_t)((span * raw12) / 4095ULL);
    // To invert mapping (0 -> MAX, 4095 -> MIN), use:
    // return MAX_PERIOD_MS - (uint32_t)((span * raw12) / 4095ULL);
}

static int read_adc_avg(int samples)
{
    int sum = 0;
    for (int i = 0; i < samples; i++)
    {
        sum += adc1_get_raw(ADC_CHANNEL);
    }
    return sum / samples;
}

void pulse_task(void *arg)
{
    ESP_LOGI(TAG, "Pulse task started. Output GPIO=%d to send to %d", PULSE_OUT, WHEEL_PULSE);
    adc1_config_width(ADC_WIDTH_CFG);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_CFG);

    // Configure output GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PULSE_OUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(PULSE_OUT, 0);

    while (1)
    {
        // Read ADC and map to period
        int raw = read_adc_avg(16); // average over 16 samples to reduce noise
        uint32_t period_ms = map_adc_to_ms(raw);
        // ESP_LOGI(TAG, "Raw: %d -> Mapped: %d", raw, period_ms);

        // Generate a pulse of fixed width only if less than max
        if (period_ms < MAX_PERIOD_MS)
        {
            gpio_set_level(PULSE_OUT, 1);
            vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));
            gpio_set_level(PULSE_OUT, 0);
        }
        // Wait remainder of the period
        uint32_t remainder_ms = (period_ms > PULSE_WIDTH_MS) ? (period_ms - PULSE_WIDTH_MS) : 0;
        if (remainder_ms > 0)
        {
            vTaskDelay(pdMS_TO_TICKS(remainder_ms));
        }
        else
        {
            // If your pulse width >= period, just yield briefly to avoid starving other tasks
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/* void app_main(void)
{
    // Configure ADC1
    adc1_config_width(ADC_WIDTH_CFG);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_CFG);

    // Configure output GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << OUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(OUT_GPIO, 0);

    // Start the task (you can pin to a core if you want)
    xTaskCreatePinnedToCore(pulse_task, "pulse_task", 2048, NULL, 5, NULL, tskNO_AFFINITY);
}
 */