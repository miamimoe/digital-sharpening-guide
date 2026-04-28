#include "zero_cal.h"
#include <cmath>

namespace zero_cal {

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps) {
    float ax = accel_g.x, ay = accel_g.y, az = accel_g.z;
    float a_sq = ax*ax + ay*ay + az*az;
    float a_mag = std::sqrt(a_sq);
    if (std::fabs(a_mag - 1.0f) >= STILL_ACCEL_MAG_TOL_G) return false;
    // Lateral check: energy outside the dominant axis must be small.
    // Catches motions that keep |a|≈1g but shift the gravity vector sideways.
    float ax_sq = ax*ax, ay_sq = ay*ay, az_sq = az*az;
    float dom = ax_sq > ay_sq ? (ax_sq > az_sq ? ax_sq : az_sq)
                              : (ay_sq > az_sq ? ay_sq : az_sq);
    float lateral_sq = a_sq - dom;
    if (lateral_sq > STILL_ACCEL_LATERAL_G * STILL_ACCEL_LATERAL_G) return false;
    float g_mag = std::sqrt(gyro_dps.x*gyro_dps.x + gyro_dps.y*gyro_dps.y + gyro_dps.z*gyro_dps.z);
    if (g_mag >= STILL_GYRO_MAG_DPS) return false;
    return true;
}

void CaptureFSM::start() {
    reset_to_warmup();
}

void CaptureFSM::reset_to_warmup() {
    phase_          = Phase::WARMUP;
    ticks_in_phase_ = 0;
    accum_          = {0.0f, 0.0f, 0.0f};
    accum_count_    = 0;
    result_         = {0.0f, 0.0f, 0.0f};
}

void CaptureFSM::update(Vec3 accel_g, Vec3 gyro_dps) {
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) return;

    if (!is_still_instant(accel_g, gyro_dps)) {
        reset_to_warmup();
        return;
    }

    ++ticks_in_phase_;

    if (phase_ == Phase::WARMUP) {
        if (ticks_in_phase_ >= WARMUP_TICKS) {
            phase_          = Phase::AVERAGING;
            ticks_in_phase_ = 0;
            accum_          = {0.0f, 0.0f, 0.0f};
            accum_count_    = 0;
        }
        return;
    }

    // AVERAGING
    accum_.x += accel_g.x;
    accum_.y += accel_g.y;
    accum_.z += accel_g.z;
    ++accum_count_;

    if (ticks_in_phase_ >= AVERAGING_TICKS) {
        float n = (float)accum_count_;
        result_ = { accum_.x / n, accum_.y / n, accum_.z / n };
        phase_  = Phase::DONE;
    }
}

int CaptureFSM::warmup_remaining() const {
    if (phase_ != Phase::WARMUP) return 0;
    return WARMUP_TICKS - ticks_in_phase_;
}

int CaptureFSM::averaging_remaining() const {
    if (phase_ != Phase::AVERAGING) return 0;
    return AVERAGING_TICKS - ticks_in_phase_;
}

Vec3 CaptureFSM::result() const {
    return result_;
}

}  // namespace zero_cal
