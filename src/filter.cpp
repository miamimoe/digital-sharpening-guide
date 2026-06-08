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

void MahonyFilter::nudge_to_gravity(Vec3 accel_g) {
    float mag = std::sqrt(accel_g.x*accel_g.x + accel_g.y*accel_g.y + accel_g.z*accel_g.z);
    if (mag < 1e-3f) return;                 // degenerate sample — leave orientation untouched
    float tz = accel_g.z / mag;

    // Build the shortest-arc orientation whose gravity() equals the normalized
    // accel, choosing yaw = 0 (yaw is unobservable from gravity and unused
    // downstream). Derivation: gravity() returns {-vx,-vy,-vz}, so we aim the
    // rotation arc at u = the desired v-vector (before that sign flip), i.e.
    // u = (tx, ty, -tz). The shortest arc from +Z to u then yields gravity()
    // = (tx, ty, tz) exactly. Note u_z = -tz, hence denom = 1 + u_z = 1 - tz.
    float denom = 1.0f - tz;                 // == 1 + u_z; ->0 only when accel ≈ +Z
    if (denom < 1e-6f) {
        // Singular: target points straight up the body +Z axis (device inverted).
        // q = (0,1,0,0) gives gravity() = (0,0,1), the matching pose.
        q0_ = 0.0f; q1_ = 1.0f; q2_ = 0.0f; q3_ = 0.0f;
    } else {
        q0_ = denom;
        q1_ = -accel_g.y / mag;
        q2_ =  accel_g.x / mag;
        q3_ = 0.0f;
        float inv = inv_sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
    }
    ix_ = iy_ = iz_ = 0.0f;                   // re-anchor: drop accumulated integral term
}

bool mahony::should_snap(Vec3 g_filter, Vec3 accel_g, Vec3 gyro_dps) {
    float gyro_mag = std::sqrt(gyro_dps.x*gyro_dps.x + gyro_dps.y*gyro_dps.y + gyro_dps.z*gyro_dps.z);
    if (gyro_mag >= SNAP_GYRO_DPS) return false;          // still moving — trust the filter

    float a_mag = std::sqrt(accel_g.x*accel_g.x + accel_g.y*accel_g.y + accel_g.z*accel_g.z);
    if (std::fabs(a_mag - 1.0f) >= SNAP_ACCEL_TOL_G) return false;  // not pure gravity

    float g_mag = std::sqrt(g_filter.x*g_filter.x + g_filter.y*g_filter.y + g_filter.z*g_filter.z);
    if (g_mag < 1e-3f) return false;          // degenerate filter state (a_mag already ≈ 1g here)

    float d = (g_filter.x*accel_g.x + g_filter.y*accel_g.y + g_filter.z*accel_g.z) / (g_mag * a_mag);
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    float div_deg = std::acos(d) * (180.0f / (float)M_PI);
    return div_deg >= SNAP_DIVERGENCE_DEG;
}
