#pragma once
#include "types.h"

namespace zero_cal {

// Stillness thresholds. The spec's original 0.01g / 0.5 dps proved unachievable
// on real MPU6886 hardware: a genuinely still device reads ~0.012g of accel
// magnitude error (scale/offset) and up to ~2.7 dps of raw gyro (bias+noise),
// so the warm-up never completed and the capture hung. Loosened to measured
// at-rest noise with margin — still far tighter than a real sharpening motion
// (>0.1g, tens of dps) so motion is still rejected. Bring-up tunable.
constexpr float STILL_ACCEL_MAG_TOL_G     = 0.05f;
constexpr float STILL_GYRO_MAG_DPS        = 8.0f;

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps);

// Derived from the real loop period (kLoopTickMs) so durations stay correct if
// the loop rate ever changes: 500 ms warm-up, 1000 ms averaging (spec §4).
constexpr int WARMUP_TICKS    = 500  / (int)kLoopTickMs;   // 25 ticks @ 50 Hz
constexpr int AVERAGING_TICKS = 1000 / (int)kLoopTickMs;   // 50 ticks @ 50 Hz

enum class Phase : uint8_t { IDLE, WARMUP, AVERAGING, DONE };

class CaptureFSM {
public:
    void  start();
    void  update(Vec3 accel_g, Vec3 gyro_dps);
    bool  done()      const { return phase_ == Phase::DONE; }
    Phase phase()     const { return phase_; }
    // Ticks left in WARMUP. Returns 0 in any other phase (including DONE/IDLE)
    // — callers should consult phase() if they need to disambiguate "not in
    // warmup yet" vs. "warmup finished".
    int   warmup_remaining()    const;
    // Ticks left in AVERAGING. Same caveat as warmup_remaining().
    int   averaging_remaining() const;
    Vec3  result()    const;

private:
    void reset_to_warmup();

    Phase phase_           = Phase::IDLE;
    int   ticks_in_phase_  = 0;
    Vec3  accum_           = {0.0f, 0.0f, 0.0f};
    int   accum_count_     = 0;
    Vec3  result_          = {0.0f, 0.0f, 0.0f};
};

}  // namespace zero_cal
