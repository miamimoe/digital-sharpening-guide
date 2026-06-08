#pragma once
#include "types.h"

namespace zero_cal {

// Stillness is gated on the ACCELEROMETER only, for a HAND-HELD capture (the user
// holds the knife flat on the stone). Two checks:
//   - magnitude: |‖a‖ − 1g| < MAG_TOL  (rejects gross translation)
//   - direction drift: |a − anchor| < DRIFT_TOL  (rejects rotation/reposition)
// Raw gyro is deliberately NOT a gate: it measures angular VELOCITY, which spikes
// on hand tremor and stalled the capture on "KEEP STILL". Hand tremor oscillates
// AROUND a point, so the accel direction stays near its anchor and passes; only a
// real reposition drifts it out. Gyro is still averaged to estimate the bias.
constexpr float STILL_ACCEL_MAG_TOL_G = 0.12f;
constexpr float STILL_DRIFT_TOL_G     = 0.12f;   // ~7° of accel-direction drift

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
    // True if the most recent update() saw motion (gate failed) — for the UI cue.
    bool  moving()    const { return moving_; }

private:
    void reset_to_warmup();

    Phase phase_           = Phase::IDLE;
    int   ticks_in_phase_  = 0;
    Vec3  accum_           = {0.0f, 0.0f, 0.0f};
    Vec3  gyro_accum_      = {0.0f, 0.0f, 0.0f};
    int   accum_count_     = 0;
    Vec3  result_          = {0.0f, 0.0f, 0.0f};
    Vec3  gyro_result_     = {0.0f, 0.0f, 0.0f};
    Vec3  accel_anchor_    = {0.0f, 0.0f, 0.0f};
    bool  anchored_        = false;
    bool  moving_          = false;
};

}  // namespace zero_cal
