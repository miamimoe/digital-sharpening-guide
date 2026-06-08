#pragma once
#include <cstdint>

// Counts sharpening passes from the back-and-forth MOTION while the blade is held
// on-angle (green), rather than counting on-angle-then-off periods. A pass shows
// up as a peak in horizontal linear acceleration (gravity removed); each peak,
// rising-edge detected with hysteresis + a refractory interval, counts one stroke.
//
// Thresholds are hardware-tunable — calibrate against a hand-counted real session.
class StrokeFSM {
public:
    static constexpr float    PEAK_HIGH_G       = 0.18f;  // rising-edge level that counts a pass
    static constexpr float    PEAK_LOW_G        = 0.10f;  // re-arm below this (hysteresis)
    static constexpr uint32_t MIN_INTERVAL_MS   = 350;    // merge intra-pass humps; ~max 3 passes/s
    static constexpr uint32_t ON_ANGLE_GRACE_MS = 600;    // tolerate a brief off-angle dip mid-pass

    // lat_accel_g: horizontal linear-acceleration magnitude (|accel - gravity|,
    // projected onto the plane perpendicular to gravity) — the stroke motion.
    void      update(uint32_t now_ms, bool in_tolerance, float lat_accel_g);
    uint32_t  stroke_count() const { return count_; }
    void      reset();

private:
    bool     armed_          = true;
    bool     have_in_tol_    = false;
    uint32_t last_count_ms_  = 0;
    uint32_t last_in_tol_ms_ = 0;
    uint32_t count_          = 0;
};
