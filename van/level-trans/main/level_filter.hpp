#pragma once
#include <math.h>

class LevelFilter
{
public:
    LevelFilter(float alpha = 0.98f);

    void reset(float roll_deg, float pitch_deg);

    void update(float ax_g, float ay_g, float az_g,
                float gx_dps, float gy_dps, float dt);

    float roll() const { return roll_; }
    float pitch() const { return pitch_; }

private:
    float alpha_;
    float roll_ = 0;
    float pitch_ = 0;
};
