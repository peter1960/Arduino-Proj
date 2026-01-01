
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <esp_cpu.h>
#include "pins.h"
#include "peter_wheel.h"
#include "peter_display.h"
#include "driver/timer.h"

#define TAG "gpio_irq"
#define WHEEL_CIRCUMFERANCE 2.199
#define PULSE_WIDTH_MS 10 // Width of the pulse (ms)

/* ------------------------------------------------------------------ */
/* --- CONFIGURABLE SECTION ------------------------------------------- */
#define SAMPLE_COUNT 3 // how many samples to average
#define TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define TASK_STACK 4096 // stack depth in words (~16 kB)

#define SECONDS_IS_ZERO 3000
const TickType_t timeoutTicksQueueWait = pdMS_TO_TICKS(SECONDS_IS_ZERO);
#define timeoutSendQueue (SECONDS_IS_ZERO - 100) * 1000
/* ------------------------------------------------------------------ */
/* --- GLOBALS ------------------------------------------------------- */

/* ring buffer that holds the last N samples */
static int g_samples[SAMPLE_COUNT];
static uint8_t g_writeIdx = 0; // where the next sample will be written
static bool g_bufferFull = false;

/*
 *  avg_task:
 *      runs every second, reads a value from the sensor,
 *      updates the ring buffer and prints the average.
 */
void avg_task(void *pvParameters)
{
    (void)pvParameters; // unused

    ESP_LOGI(TAG, "Reset pulse count");
    speed_read_reset(); // clear any values

    for (;;)
    {
        /* ---- 1. Wait until next second --------------------------------- */
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* ---- 2. Get the current count ------------------------------- */
        int current = speed_read_reset();

        /* ---- 3. Store it in the ring buffer -------------------------- */
        g_samples[g_writeIdx] = current;
        g_writeIdx = (g_writeIdx + 1) % SAMPLE_COUNT;

        if (!g_bufferFull && g_writeIdx == 0)
        {
            g_bufferFull = true; // we have wrapped at least once
        }

        /* ---- 4. Compute the average --------------------------------- */
        float sum = 0.0;
        uint8_t count = g_bufferFull ? SAMPLE_COUNT : g_writeIdx;

        for (uint8_t i = 0; i < count; ++i)
        {
            sum += g_samples[i];
        }

        float avg = (count > 0) ? (float)sum / count : 0.0f;
        /* ---- 5. Output ---------------------------------------------- */
        float mps = WHEEL_CIRCUMFERANCE * avg;
        float kph = mps * 3.6;
        // Only set speed when buffer full so average is correct
        if (g_bufferFull)
        {
            speed_kph(kph);
        }
        // ESP_LOGI(TAG, "count=%d  avg over %u seconds(s): %.2f  m/s %.2f kmh %.2f", current, count, avg, mps, kph);
    }
}

// static gpio_evt_queue_t xQueueHandle = NULL;
static QueueHandle_t xQueueHandle = NULL;
static volatile uint64_t s_last_cnt = 0; // last timestamp

// ISR
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    if (gpio_num == WHEEL_PULSE)
    {
        static int64_t last_ts_us = -1;
        int64_t now_us = esp_timer_get_time();
        if (last_ts_us >= 0)
        {
            int64_t delta_us = now_us - last_ts_us;
            if (delta_us < 3500000)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(xQueueHandle, &delta_us, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        last_ts_us = now_us;
    }
}

void wheel_task(void *arg)
{

    xQueueHandle = xQueueCreate(10, sizeof(int64_t));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WHEEL_PULSE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // Configure output GPIO LED
    gpio_config_t io_conf2 = {
        .pin_bit_mask = 1ULL << PULSE_LED_OUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf2);
    gpio_set_level(PULSE_LED_OUT, 0);

    ESP_ERROR_CHECK(gpio_isr_handler_add(WHEEL_PULSE, gpio_isr_handler, (void *)WHEEL_PULSE));

    ESP_LOGI(TAG, "%d lower-edge interrupt initialized.", WHEEL_PULSE);
    uint32_t gpio_num = WHEEL_PULSE;
    int64_t delta_us;
    while (true)
    {
        // if (xQueueReceive(xQueueHandle, &gpio_num, portMAX_DELAY))
        if (xQueueReceive(xQueueHandle, &delta_us, timeoutTicksQueueWait) == pdTRUE)
        {
            float speed = (WHEEL_CIRCUMFERANCE * 3600.0f) / (delta_us / 1000.0);
            ESP_LOGI(TAG, "Speed = %.2f kph (%.3f ms, %.3f Hz)", speed, delta_us / 1000.0, delta_us > 0 ? (1e6 / (double)delta_us) : 0.0);
            // ESP_LOGI("EVENT", "Received: %d", gpio_num);
            distance_set(WHEEL_CIRCUMFERANCE);
            speed_kph(speed);
            // ESP_LOGI("GPIO", "Interrupt from GPIO %lu", gpio_num);
            ESP_LOGI("EVENT", "Pulse");
        }
        else
        {
            ESP_LOGI("EVENT", "No Pulse");
            speed_kph(0.0);
        }
        /*
        if (xQueueReceive(xQueueHandle, &evt, portMAX_DELAY))
        {
            gpio_set_level(PULSE_LED_OUT, 1);
            vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));
            gpio_set_level(PULSE_LED_OUT, 0);
        }
            */
    }
}
/*
void app_main(void)
{
    // 1) Configure the pin as input with interrupt on rising edge
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // <-- prefer pulldown for rising-edge detection
        .intr_type = GPIO_INTR_POSEDGE          // <-- rising edge
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 2) Create a queue for ISR-to-task communication
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // 3) Install the ISR service (once globally in your app)
    // Pass 0 for default flags; you can use ESP_INTR_FLAG_IRAM if your handler resides in IRAM.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // 4) Attach the ISR to GPIO5
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_INPUT_PIN, gpio_isr_handler, (void*) GPIO_INPUT_PIN));

    ESP_LOGI(TAG, "GPIO5 rising-edge interrupt initialized.");
}
*/
