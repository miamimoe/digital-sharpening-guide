#include "zero_cal.h"
#include <cmath>

namespace zero_cal {

void CaptureFSM::start() {
    reset_to_warmup();
}

void CaptureFSM::reset_to_warmup() {
    phase_          = Phase::WARMUP;
    ticks_in_phase_ = 0;
    accum_          = {0.0f, 0.0f, 0.0f};
    gyro_accum_     = {0.0f, 0.0f, 0.0f};
    accum_count_    = 0;
    result_         = {0.0f, 0.0f, 0.0f};
    gyro_result_    = {0.0f, 0.0f, 0.0f};
    anchored_       = false;   // re-anchor the drift reference on the next still sample
}

void CaptureFSM::update(Vec3 accel_g, Vec3 gyro_dps) {
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) return;

    // Accel-only stillness: magnitude near 1g AND direction close to the anchor.
    float a_mag = std::sqrt(accel_g.x*accel_g.x + accel_g.y*accel_g.y + accel_g.z*accel_g.z);
    bool mag_ok = std::fabs(a_mag - 1.0f) < STILL_ACCEL_MAG_TOL_G;
    bool drift_ok = true;
    if (!anchored_) {
        accel_anchor_ = accel_g;
        anchored_     = true;
    } else {
        float dx = accel_g.x - accel_anchor_.x;
        float dy = accel_g.y - accel_anchor_.y;
        float dz = accel_g.z - accel_anchor_.z;
        drift_ok = std::sqrt(dx*dx + dy*dy + dz*dz) < STILL_DRIFT_TOL_G;
    }
    moving_ = !(mag_ok && drift_ok);
    if (moving_) {
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
    gyro_accum_.x += gyro_dps.x;
    gyro_accum_.y += gyro_dps.y;
    gyro_accum_.z += gyro_dps.z;
    ++accum_count_;

    if (ticks_in_phase_ >= AVERAGING_TICKS) {
        float n = (float)accum_count_;
        gyro_result_ = { gyro_accum_.x / n, gyro_accum_.y / n, gyro_accum_.z / n };
        Vec3 mean = { accum_.x / n, accum_.y / n, accum_.z / n };
        // Normalize (spec §4 step 6). Averaging unit vectors yields magnitude < 1;
        // downstream consumers (grav_dot_ref polarity in side.cpp) expect a unit
        // reference. bevel_angle normalizes defensively, but make the stored
        // zero canonical here.
        float mag = std::sqrt(mean.x*mean.x + mean.y*mean.y + mean.z*mean.z);
        if (mag > 1e-6f) {
            result_ = { mean.x / mag, mean.y / mag, mean.z / mag };
        } else {
            result_ = mean;  // degenerate; should never happen past the stillness gate
        }
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

Vec3 CaptureFSM::gyro_bias() const {
    return gyro_result_;
}

}  // namespace zero_cal
