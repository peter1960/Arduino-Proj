
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
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
#define SAMPLE_COUNT 5 // how many samples to average
#define TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define TASK_STACK 4096 // stack depth in words (~16 kB)

/* ------------------------------------------------------------------ */
/* --- GLOBALS ------------------------------------------------------- */

/* ring buffer that holds the last N samples */
static int g_samples[SAMPLE_COUNT];
static uint8_t g_writeIdx = 0; // where the next sample will be written
static bool g_bufferFull = false;

/* ------------------------------------------------------------------ */
/* --- SENSOR READ (replace this stub with your own logic) ---------- */

int sensor_read(void)
{
    return speed_read_reset();
}
/*
 *  avg_task:
 *      runs every second, reads a value from the sensor,
 *      updates the ring buffer and prints the average.
 */
void avg_task(void *pvParameters)
{
    (void)pvParameters; // unused

    for (;;)
    {
        /* ---- 1. Wait until next second --------------------------------- */
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* ---- 2. Get the current count ------------------------------- */
        int current = sensor_read();

        /* ---- 3. Store it in the ring buffer -------------------------- */
        g_samples[g_writeIdx] = current;
        g_writeIdx = (g_writeIdx + 1) % SAMPLE_COUNT;

        if (!g_bufferFull && g_writeIdx == 0)
        {
            g_bufferFull = true; // we have wrapped at least once
        }

        /* ---- 4. Compute the average --------------------------------- */
        int sum = 0;
        uint8_t count = g_bufferFull ? SAMPLE_COUNT : g_writeIdx;

        for (uint8_t i = 0; i < count; ++i)
        {
            sum += g_samples[i];
        }

        float avg = (count > 0) ? (float)sum / count : 0.0f;

        /* ---- 5. Output ---------------------------------------------- */
        ESP_LOGI(TAG, "cnt=%d  avg over %u sample(s): %.2f", current, count, avg);
    }
}

// static gpio_evt_queue_t xQueueHandle = NULL;
static QueueHandle_t xQueueHandle = NULL;
static volatile uint64_t s_last_cnt = 0; // last timestamp

// ISR
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xQueueHandle, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void wheel_task(void *arg)
{

    xQueueHandle = xQueueCreate(10, sizeof(uint8_t));

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
    uint32_t gpio_num = 0;
    while (true)
    {
        if (xQueueReceive(xQueueHandle, &gpio_num, portMAX_DELAY))
        {
            if (gpio_num == WHEEL_PULSE)
            {
                // ESP_LOGI("EVENT", "Received: %d", gpio_num);
                distance_set(WHEEL_CIRCUMFERANCE);
                speed_pulse();
            }
            // ESP_LOGI("GPIO", "Interrupt from GPIO %lu", gpio_num);
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