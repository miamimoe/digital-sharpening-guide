#include "side.h"
#include <cmath>

void SideFSM::update(uint32_t now_ms, float accel_mag_g, float gyro_mag_dps, float grav_dot_ref) {
    // After a manual override, ignore auto-detection briefly so the user's choice
    // isn't immediately re-evaluated against the current orientation.
    if (suppress_armed_ && ((int32_t)(suppress_until_ms_ - now_ms) > 0)) {
        flipping_ = false;
        return;
    }
    suppress_armed_ = false;

    bool settled = std::fabs(accel_mag_g - 1.0f) <= SETTLE_TOL_G && gyro_mag_dps < SETTLE_GYRO_DPS;

    // Polarity that belongs to the OTHER side than the one we think we're on.
    bool flipped_pose = (side_ == Side::A) ? (grav_dot_ref < -FLIP_POLARITY_MIN)
                                           : (grav_dot_ref >  FLIP_POLARITY_MIN);

    if (settled && flipped_pose) {
        if (!flipping_) {
            flipping_        = true;
            flip_started_ms_ = now_ms;
        } else if (now_ms - flip_started_ms_ >= SETTLE_REQUIRED_MS) {
            side_           = (side_ == Side::A) ? Side::B : Side::A;
            switch_pending_ = true;
            flipping_       = false;
        }
    } else {
        // Not at rest, or still in the current side's orientation — keep waiting.
        flipping_ = false;
    }
}

void SideFSM::manual_toggle(uint32_t now_ms) {
    side_              = (side_ == Side::A) ? Side::B : Side::A;
    switch_pending_    = true;
    suppress_until_ms_ = now_ms + SUPPRESS_MS;
    suppress_armed_    = true;
    flipping_          = false;
}

bool SideFSM::consume_switch() {
    bool v = switch_pending_;
    switch_pending_ = false;
    return v;
}

void SideFSM::reset() {
    side_              = Side::A;
    switch_pending_    = false;
    flipping_          = false;
    flip_started_ms_   = 0;
    suppress_until_ms_ = 0;
    suppress_armed_    = false;
}
