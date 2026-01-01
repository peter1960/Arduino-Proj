#define CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306 1
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include <driver/i2c_master.h>

#include <ssd1306.h>
#include <ssd1306_fonts.h>
#include "pins.h"
#include "peter_display.h"

static const char *TAG = "DISPLAY";

#define OLED_WIDE 128
#define OLED_HIGH 64
#define OLED_TICK_TOP OLED_HIGH - OLED_TICK_HIGH
#define OLED_TICK_LEFT OLED_WIDE - OLED_TICK_WIDE
#define OLED_TICK_BOTTOM OLED_HIGH
#define OLED_TICK_RIGHT OLED_WIDE
#define OLED_TICK_HIGH 6
#define OLED_TICK_WIDE 6

#define I2C_MASTER_NUM I2C_NUM_0  /*!&lt; I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!&lt; I2C master clock frequency */

// static SemaphoreHandle_t s_speed_mutex;

//////////////////////
portMUX_TYPE g_dataLockSpeedPulse = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_dataLockDistance = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_dataLockSpeed = portMUX_INITIALIZER_UNLOCKED;
/*
     data below is locked on read/write
*/
static int g_speed_pulses = 0;
static float g_odometer = 0.0f;
static float g_speed = 0.0f;
//////////////////////
void speed_kph(float speed)
{
    speed_lock();
    g_speed = speed;
    speed_unlock();
    // ESP_LOGI(TAG, "Speed %.2f", speed);
}

void speed_pulse()
{
    // ESP_LOGI("PULSE", "Received Pulse");

    speed_pulse_lock();
    g_speed_pulses++;
    speed_pulse_unlock();
}
int speed_read_reset()
{
    speed_pulse_lock();
    int val = g_speed_pulses;
    g_speed_pulses = 0;
    speed_pulse_unlock();
    return val;
}

void distance_set(float s_distance)
{
    distance_lock();
    g_odometer += s_distance;
    distance_unlock();
}

void speed_pulse_lock(void)
{
    portENTER_CRITICAL(&g_dataLockSpeedPulse);
}

void speed_pulse_unlock(void)
{
    portEXIT_CRITICAL(&g_dataLockSpeedPulse);
}

void speed_lock(void)
{
    portENTER_CRITICAL(&g_dataLockSpeed);
}

void speed_unlock(void)
{
    portEXIT_CRITICAL(&g_dataLockSpeed);
}

void distance_lock(void)
{
    portENTER_CRITICAL(&g_dataLockDistance);
}

void distance_unlock(void)
{
    portEXIT_CRITICAL(&g_dataLockDistance);
}

void display_task(void *args)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    ssd1306_handle_t ssd1306_dev = NULL;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    ///////////////////////////////////////
    float speed_copy = 0.0f;
    float odd_copy = 0.0f;
    float trip_copy = 0.0f;
    char buffer[20];
    //    float last_speed = -1.0;
    bool on = true;
    // ssd1306_clear_display(dev_hdl, false);
    while (true)
    {
        // ESP_LOGI(TAG, "Display updated successfully.");
        speed_lock();
        speed_copy = g_speed;
        speed_unlock();
        distance_lock();
        odd_copy = g_odometer;
        distance_unlock();
        //      if (last_speed != speed_copy)
        //      {
        //          if (last_speed > 0.0)
        //          {
        //              sprintf(buffer, "%04.1f", last_speed);
        //          }
        if (speed_copy < 100)
        {
            sprintf(buffer, "%05.2f", speed_copy);
            ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)buffer, 16, 1);
        }
        sprintf(buffer, "O %08.2f", odd_copy / 1000.0);
        ssd1306_draw_string(ssd1306_dev, 0, 33, (const uint8_t *)buffer, 16, 1);

        sprintf(buffer, "T %08.2f", trip_copy / 1000.0);
        ssd1306_draw_string(ssd1306_dev, 0, 49, (const uint8_t *)buffer, 16, 1);

        //          last_speed = speed_copy;
        //      }

        Ticker(ssd1306_dev, OLED_TICK_LEFT, OLED_TICK_TOP, OLED_TICK_RIGHT, OLED_TICK_BOTTOM, on);
        on = !on;
        ssd1306_refresh_gram(ssd1306_dev);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void Ticker(ssd1306_handle_t dev, int left, int top, int right, int bottom, bool on)
{
    ssd1306_fill_rectangle(dev, left, top, right, bottom, (on ? 1 : 0));
}
