#include "stroke.h"

void StrokeFSM::update(uint32_t now_ms, bool in_tolerance, float lat_accel_g) {
    if (in_tolerance) {
        have_in_tol_    = true;
        last_in_tol_ms_ = now_ms;
    }
    // Only count while on-angle; a brief dip out of green mid-pass is tolerated.
    bool on_angle = in_tolerance ||
                    (have_in_tol_ && (now_ms - last_in_tol_ms_) < ON_ANGLE_GRACE_MS);
    if (!on_angle) {
        armed_ = false;
        return;
    }

    if (lat_accel_g < PEAK_LOW_G) {
        armed_ = true;                                   // settled between passes — re-arm
    } else if (lat_accel_g >= PEAK_HIGH_G && armed_ &&
               (now_ms - last_count_ms_) >= MIN_INTERVAL_MS) {
        count_++;                                        // rising edge of a pass
        armed_         = false;
        last_count_ms_ = now_ms;
    }
}

void StrokeFSM::reset() {
    armed_          = true;
    have_in_tol_    = false;
    last_count_ms_  = 0;
    last_in_tol_ms_ = 0;
    count_          = 0;
}
