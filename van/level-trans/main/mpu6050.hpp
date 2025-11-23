#pragma once
#include <stdint.h>
#include "esp_err.h"

class MPU6050
{
public:
    MPU6050(gpio_num_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, int freq_hz = 400000);

    esp_err_t init();
    esp_err_t calibrate(int samples = 500);

    esp_err_t read(float &ax_g, float &ay_g, float &az_g,
                   float &gx_dps, float &gy_dps, float &gz_dps);

private:
    gpio_num_t i2c_port_;
    gpio_num_t sda_pin_;
    gpio_num_t scl_pin_;
    int freq_hz_;

    float ax_offset_ = 0;
    float ay_offset_ = 0;
    float az_offset_ = 0;
    float gx_offset_ = 0;
    float gy_offset_ = 0;
    float gz_offset_ = 0;

    esp_err_t readBytes(uint8_t reg, uint8_t *data, size_t len);
    esp_err_t writeByte(uint8_t reg, uint8_t data);
};
