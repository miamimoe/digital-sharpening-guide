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

    // Per-session thresholds. reset() does not touch these — a side toggle must
    // not throw away a just-learned stroke. Factory defaults live in the
    // PEAK_* constants above.
    void  set_thresholds(float high_g, float low_g);
    void  reset_thresholds();
    float peak_high() const { return high_g_; }
    float peak_low()  const { return low_g_; }

private:
    bool     armed_          = true;
    bool     have_in_tol_    = false;
    uint32_t last_count_ms_  = 0;
    uint32_t last_in_tol_ms_ = 0;
    uint32_t count_          = 0;
    float    high_g_         = PEAK_HIGH_G;
    float    low_g_          = PEAK_LOW_G;
};

// Records motion peaks during STROKE_LEARN so A can lock thresholds to this
// user's stroke instead of the factory 0.18 / 0.10 g guess.
class StrokeLearner {
public:
    static constexpr int      TARGET_PEAKS     = 10;
    static constexpr int      MIN_PEAKS        = 3;
    static constexpr int      MAX_PEAKS        = 16;
    static constexpr float    DETECT_HIGH_G    = 0.08f;
    static constexpr float    DETECT_LOW_G     = 0.04f;
    static constexpr uint32_t MIN_INTERVAL_MS  = 350;

    void reset();
    void update(uint32_t now_ms, float lat_accel_g);
    int  peaks() const { return n_; }
    // Writes learned (or factory) thresholds onto fsm. True if learned.
    bool commit(StrokeFSM& fsm) const;

private:
    float    peaks_[MAX_PEAKS] = {};
    int      n_                = 0;
    bool     armed_            = true;
    float    peak_lat_         = 0.0f;
    uint32_t last_peak_ms_     = 0;
};
