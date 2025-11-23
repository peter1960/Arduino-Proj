#include "level_filter.hpp"

LevelFilter::LevelFilter(float alpha) : alpha_(alpha) {}

void LevelFilter::reset(float roll_deg, float pitch_deg)
{
    roll_ = roll_deg;
    pitch_ = pitch_deg;
}

void LevelFilter::update(float ax, float ay, float az,
                         float gx, float gy, float dt)
{
    float acc_roll = atan2f(ay, az) * 180.0f / M_PI;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    float gyro_roll = roll_ + gx * dt;
    float gyro_pitch = pitch_ + gy * dt;

    roll_ = alpha_ * gyro_roll + (1 - alpha_) * acc_roll;
    pitch_ = alpha_ * gyro_pitch + (1 - alpha_) * acc_pitch;
}
