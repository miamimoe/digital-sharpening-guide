#include "side.h"
#include <cmath>

void SideFSM::update(uint32_t now_ms, float accel_mag_g, float gyro_mag_dps, float grav_dot_ref) {
    float dev = std::fabs(accel_mag_g - 1.0f);

    if (phase_ == STABLE) {
        if (dev > SPIKE_DEVIATION_G) {
            // Only enter WAITING_SETTLE if not currently suppressed.
            bool in_suppress = suppress_armed_ && ((int32_t)(suppress_until_ms_ - now_ms) > 0);
            if (!in_suppress) {
                // Clear expired suppress flag.
                suppress_armed_    = false;
                phase_             = WAITING_SETTLE;
                phase_entered_ms_  = now_ms;
                settling_          = false;
            }
        }
        return;
    }

    // WAITING_SETTLE
    if (now_ms - phase_entered_ms_ > POST_SPIKE_TIMEOUT_MS) {
        phase_    = STABLE;
        settling_ = false;
        return;
    }

    // A genuine settle needs both the accel magnitude back near 1g AND the gyro
    // quiet — a slow handling rotation can momentarily read ~1g while the device
    // is still turning, which must not be mistaken for "laid flat on the blade".
    if (dev <= SETTLE_TOL_G && gyro_mag_dps < SETTLE_GYRO_DPS) {
        if (!settling_) {
            settling_          = true;
            settle_started_ms_ = now_ms;
        }
        if (now_ms - settle_started_ms_ >= SETTLE_REQUIRED_MS) {
            // Settle achieved. Check for flip using side-aware orientation.
            bool flipped = (side_ == Side::A) ? (grav_dot_ref < 0.0f)
                                              : (grav_dot_ref > 0.0f);
            if (flipped) {
                side_           = (side_ == Side::A) ? Side::B : Side::A;
                switch_pending_ = true;
            }
            phase_    = STABLE;
            settling_ = false;
        }
    } else {
        settling_ = false;
    }
}

void SideFSM::manual_toggle(uint32_t now_ms) {
    side_              = (side_ == Side::A) ? Side::B : Side::A;
    switch_pending_    = true;
    suppress_until_ms_ = now_ms + SUPPRESS_MS;
    suppress_armed_    = true;
    // Abort any in-progress WAITING_SETTLE to prevent a stale peel from
    // triggering a spurious second switch against the just-toggled side.
    phase_             = STABLE;
    settling_          = false;
}

bool SideFSM::consume_switch() {
    bool v = switch_pending_;
    switch_pending_ = false;
    return v;
}

void SideFSM::reset() {
    phase_              = STABLE;
    phase_entered_ms_   = 0;
    settle_started_ms_  = 0;
    settling_           = false;
    suppress_until_ms_  = 0;
    suppress_armed_     = false;
    switch_pending_     = false;
    side_               = Side::A;
}
