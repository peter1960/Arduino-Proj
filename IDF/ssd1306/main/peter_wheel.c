
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

typedef struct
{
    uint32_t gpio_num;
    int64_t t_us;  // current timestamp in microseconds
    int64_t dt_us; // time since previous trigger (microseconds)
} trigger_event_t;

// static gpio_evt_queue_t xQueueHandle = NULL;
static QueueHandle_t xQueueHandle = NULL;
static volatile uint64_t s_last_cnt = 0; // last timestamp

static void init_free_running_timer(void)
{
    timer_config_t config = {
        .divider = 80, // 80 MHz APB / 80 = 1 MHz
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS, // free-run
        .auto_reload = false};
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// ISR: keep it short!
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;

    uint64_t cnt = 0;
    // timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &cnt); // 1 tick = 1 us

    // Compute delta (0 for first sample)
    int64_t dt_us = (s_last_cnt == 0) ? 0 : (cnt - s_last_cnt);
    s_last_cnt = cnt;

    trigger_event_t evt = {
        .gpio_num = gpio_num,
        .t_us = cnt,
        .dt_us = dt_us};

    // Send the pin number to a queue for processing in a task
    xQueueSendFromISR(xQueueHandle, &evt, NULL);
}

void wheel_task(void *arg)
{

    trigger_event_t evt;

    xQueueHandle = xQueueCreate(10, sizeof(uint32_t));
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
    init_free_running_timer();
    // uint32_t io_num;
    while (true)
    {
        if (xQueueReceive(xQueueHandle, &evt, portMAX_DELAY))
        {
            // Process the event in task context
            /*
            ESP_LOGI(TAG, "GPIO %u: dt=%lu us, t=%lu us",
                     (unsigned)evt.gpio_num,
                     (unsigned long)evt.dt_us,
                     (unsigned long)evt.t_us);
*/
            speed_set(1, WHEEL_CIRCUMFERANCE);
            gpio_set_level(PULSE_LED_OUT, 1);
            vTaskDelay(pdMS_TO_TICKS(PULSE_WIDTH_MS));
            gpio_set_level(PULSE_LED_OUT, 0);
        }
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