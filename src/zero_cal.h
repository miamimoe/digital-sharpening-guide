#pragma once
#include "types.h"

namespace zero_cal {

// Stillness thresholds (per spec 2026-04-28 §4):
//   |‖a‖ − 1.0g| < 0.01g
//   |gyro| < 0.5 dps
// Per-axis stddev windowing (spec §4 third bullet) is deferred to v2 —
// magnitude + gyro is sufficient for realistic bench sharpening setups.
constexpr float STILL_ACCEL_MAG_TOL_G     = 0.01f;
constexpr float STILL_GYRO_MAG_DPS        = 0.5f;

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps);

constexpr int WARMUP_TICKS    = 50;   // 500 ms at 100 Hz
constexpr int AVERAGING_TICKS = 100;  // 1000 ms at 100 Hz

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
