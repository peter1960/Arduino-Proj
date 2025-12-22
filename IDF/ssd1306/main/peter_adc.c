#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

/*
ADC1_CHANNEL_0	GPIO36
https://embeddedexplorer.com/esp32-adc-esp-idf-tutorial/

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
static const char *TAG = "ADC READ";

#define a 10   // min of source range
#define b 4096 // max of source range
#define c 6500 // min of target range (larger number)
#define d 198  // max of target range (smaller number)

int map_value(int x)
{
    /* Clamp the input just in case */
    if (x < a)
        x = a;
    if (x > b)
        x = b;

    /* Perform the linear mapping */
    double y = c + (double)(x - a) * (d - c) / (b - a);

    /* If you really need an integer result, round or cast as desired */
    return (int)(y + 0.5); // round to nearest int
}

void setup_adc1()
{

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
}
int getValue()
{
    int xx = adc1_get_raw(ADC1_CHANNEL_0);
    int map_xx = map_value(xx);
    // ESP_LOGI(TAG, "ADC1_CHANNEL_0: %d -> %d", xx, map_xx);
    return map_xx;
}
