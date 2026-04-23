#pragma once
#include "types.h"

class MahonyFilter {
public:
    void begin(float sample_hz, float kp = 0.5f, float ki = 0.0f);
    void update(Vec3 gyro_dps, Vec3 accel_g);
    void set_bias(Vec3 gyro_bias_dps) { bias_ = gyro_bias_dps; }
    Vec3 gravity() const;
    void reset();

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;
    float kp_ = 0.5f, ki_ = 0.0f;
    float dt_ = 0.01f;
    Vec3  bias_ = {0.0f, 0.0f, 0.0f};
};
