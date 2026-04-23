#include "filter.h"
#include <cmath>

static inline float inv_sqrt(float x) {
    return 1.0f / std::sqrt(x);
}

void MahonyFilter::begin(float sample_hz, float kp, float ki) {
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    ix_ = iy_ = iz_ = 0.0f;
    kp_ = (kp > 0.0f) ? kp : 0.5f;
    ki_ = ki;
    dt_ = 1.0f / sample_hz;
}

void MahonyFilter::reset() {
    begin(1.0f / dt_, kp_, ki_);
}

void MahonyFilter::update(Vec3 gyro_dps, Vec3 accel_g) {
    const float DEG2RAD = (float)M_PI / 180.0f;
    float gx = (gyro_dps.x - bias_.x) * DEG2RAD;
    float gy = (gyro_dps.y - bias_.y) * DEG2RAD;
    float gz = (gyro_dps.z - bias_.z) * DEG2RAD;

    float ax = accel_g.x, ay = accel_g.y, az = accel_g.z;

    float an = ax*ax + ay*ay + az*az;
    if (an > 0.0f) {
        float inv = inv_sqrt(an);
        ax *= inv; ay *= inv; az *= inv;

        float vx = 2.0f * (q1_*q3_ - q0_*q2_);
        float vy = 2.0f * (q0_*q1_ + q2_*q3_);
        float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

        float ex = -(ay*vz - az*vy);
        float ey = -(az*vx - ax*vz);
        float ez = -(ax*vy - ay*vx);

        if (ki_ > 0.0f) {
            ix_ += ki_ * ex * dt_;
            iy_ += ki_ * ey * dt_;
            iz_ += ki_ * ez * dt_;
            gx += ix_;
            gy += iy_;
            gz += iz_;
        }

        gx += kp_ * ex;
        gy += kp_ * ey;
        gz += kp_ * ez;
    }

    gx *= 0.5f * dt_;
    gy *= 0.5f * dt_;
    gz *= 0.5f * dt_;

    float qa = q0_, qb = q1_, qc = q2_;
    q0_ += -qb*gx - qc*gy - q3_*gz;
    q1_ +=  qa*gx + qc*gz - q3_*gy;
    q2_ +=  qa*gy - qb*gz + q3_*gx;
    q3_ +=  qa*gz + qb*gy - qc*gx;

    float inv = inv_sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
}

Vec3 MahonyFilter::gravity() const {
    float vx = 2.0f * (q1_*q3_ - q0_*q2_);
    float vy = 2.0f * (q0_*q1_ + q2_*q3_);
    float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;
    return {-vx, -vy, -vz};
}
