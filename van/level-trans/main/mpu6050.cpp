#include "mpu6050.hpp"
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MPU6050_CPP";

#define MPU_ADDR 0x68
#define ACC_SCALE 16384.0f
#define GYRO_SCALE 131.0f

enum
{
    MPU_PWR_MGMT_1 = 0x6B,
    MPU_SMPLRT_DIV = 0x19,
    MPU_CONFIG = 0x1A,
    MPU_GYRO_CONFIG = 0x1B,
    MPU_ACCEL_CONFIG = 0x1C,
    MPU_ACCEL_XOUT_H = 0x3B,
    MPU_GYRO_XOUT_H = 0x43
};

MPU6050::MPU6050(gpio_num_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, int freq_hz)
    : i2c_port_(i2c_port), sda_pin_(sda_pin), scl_pin_(scl_pin), freq_hz_(freq_hz) {}

esp_err_t MPU6050::init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin_;
    conf.scl_io_num = scl_pin_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq_hz_;

    // ESP_ERROR_CHECK(i2c_param_config(i2c_port_, &conf));
    // ESP_ERROR_CHECK(i2c_driver_install(i2c_port_, conf.mode, 0, 0, 0));

    writeByte(MPU_PWR_MGMT_1, 0x00);
    writeByte(MPU_SMPLRT_DIV, 0x07);
    writeByte(MPU_CONFIG, 0x03);
    writeByte(MPU_GYRO_CONFIG, 0x00);
    writeByte(MPU_ACCEL_CONFIG, 0x00);

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t MPU6050::calibrate(int samples)
{
    ESP_LOGI(TAG, "Calibrating with %d samples...", samples);

    double sumAX = 0, sumAY = 0, sumAZ = 0;
    double sumGX = 0, sumGY = 0, sumGZ = 0;

    for (int i = 0; i < samples; ++i)
    {
        float ax, ay, az, gx, gy, gz;
        read(ax, ay, az, gx, gy, gz);
        sumAX += ax;
        sumAY += ay;
        sumAZ += az;
        sumGX += gx;
        sumGY += gy;
        sumGZ += gz;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ax_offset_ = sumAX / samples;
    ay_offset_ = sumAY / samples;
    az_offset_ = sumAZ / samples;
    gx_offset_ = sumGX / samples;
    gy_offset_ = sumGY / samples;
    gz_offset_ = sumGZ / samples;

    ESP_LOGI(TAG, "Calibration complete.");
    return ESP_OK;
}

esp_err_t MPU6050::read(float &ax_g, float &ay_g, float &az_g,
                        float &gx_dps, float &gy_dps, float &gz_dps)
{
    uint8_t raw[6];

    readBytes(MPU_ACCEL_XOUT_H, raw, 6);
    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];

    readBytes(MPU_GYRO_XOUT_H, raw, 6);
    int16_t gx = (raw[0] << 8) | raw[1];
    int16_t gy = (raw[2] << 8) | raw[3];
    int16_t gz = (raw[4] << 8) | raw[5];

    ax_g = ax / ACC_SCALE - ax_offset_;
    ay_g = ay / ACC_SCALE - ay_offset_;
    az_g = az / ACC_SCALE - az_offset_;
    gx_dps = gx / GYRO_SCALE - gx_offset_;
    gy_dps = gy / GYRO_SCALE - gy_offset_;
    gz_dps = gz / GYRO_SCALE - gz_offset_;

    return ESP_OK;
}

esp_err_t MPU6050::readBytes(uint8_t reg, uint8_t *data, size_t len)
{
    if (len == 0)
        return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MPU6050::writeByte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}
