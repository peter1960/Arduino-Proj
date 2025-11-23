#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "mpu6050.hpp"
#include "level_filter.hpp"

static const char *TAG = "MAIN_CPP";
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_PORT I2C_NUM_0

#define SAMPLE_RATE_HZ 100.0f
#define LEVEL_THRESHOLD_DEG 2.0f

#define CAL_BUTTON 0 // active low

static double now_s()
{
    return esp_timer_get_time() / 1000000.0;
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting MPU6050 C++ project");

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << CAL_BUTTON;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    MPU6050 imu(I2C_PORT, SDA_PIN, SCL_PIN);
    imu.init();
    imu.calibrate(500);

    float ax, ay, az, gx, gy, gz;

    imu.read(ax, ay, az, gx, gy, gz);
    float init_roll = atan2f(ay, az) * 180.0f / M_PI;
    float init_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    LevelFilter filter(0.98f);
    filter.reset(init_roll, init_pitch);

    double last = now_s();

    while (1)
    {
        double t = now_s();
        float dt = t - last;
        last = t;

        imu.read(ax, ay, az, gx, gy, gz);
        filter.update(ax, ay, az, gx, gy, dt);

        float roll = filter.roll();
        float pitch = filter.pitch();

        bool level =
            fabsf(roll) < LEVEL_THRESHOLD_DEG &&
            fabsf(pitch) < LEVEL_THRESHOLD_DEG;

        ESP_LOGI(TAG, "R:%+6.2f  P:%+6.2f  -> %s",
                 roll, pitch, level ? "LEVEL" : "NOT LEVEL");

        if (gpio_get_level(CAL_BUTTON) == 0)
        {
            ESP_LOGI(TAG, "Recalibrating...");
            imu.calibrate(500);

            imu.read(ax, ay, az, gx, gy, gz);
            float r = atan2f(ay, az) * 180.0f / M_PI;
            float p = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
            filter.reset(r, p);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }
}
