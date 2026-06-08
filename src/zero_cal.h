#pragma once
#include "types.h"

namespace zero_cal {

// Stillness thresholds. These must tolerate a HAND-HELD capture: during zero-cal
// the user holds the knife flat on the stone, so there's always hand tremor on
// top of raw-gyro bias. The original tight values (0.01g/0.5dps, then 0.05g/8dps)
// rejected that tremor and hung on "KEEP STILL". Loosened to hand-held-steady
// levels — still well below an actual sharpening stroke (>0.3g, 30+ dps). If the
// gate still can't pass (heavy bias/vibration), the user can force-capture (tap B).
constexpr float STILL_ACCEL_MAG_TOL_G     = 0.10f;
constexpr float STILL_GYRO_MAG_DPS        = 20.0f;

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
    // Mean gyro (dps) over the averaging window = the gyro bias, since the gate
    // guarantees the device was still. Use to refresh the filter's bias estimate.
    Vec3  gyro_bias() const;

private:
    void reset_to_warmup();

    Phase phase_           = Phase::IDLE;
    int   ticks_in_phase_  = 0;
    Vec3  accum_           = {0.0f, 0.0f, 0.0f};
    Vec3  gyro_accum_      = {0.0f, 0.0f, 0.0f};
    int   accum_count_     = 0;
    Vec3  result_          = {0.0f, 0.0f, 0.0f};
    Vec3  gyro_result_     = {0.0f, 0.0f, 0.0f};
};

}  // namespace zero_cal
