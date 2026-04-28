#pragma once
#include "types.h"

namespace zero_cal {

// Stillness thresholds (per spec 2026-04-28 §4):
//   |‖a‖ − 1.0g| < 0.01g
//   |gyro| < 0.5 dps
// Per-axis stddev gate (spec §4 third bullet) is deferred to v2 — the magnitude
// + gyro pair is sufficient for realistic bench sharpening setups. Revisit if
// hardware bring-up shows false-positive captures.
constexpr float STILL_ACCEL_MAG_TOL_G = 0.01f;
constexpr float STILL_GYRO_MAG_DPS    = 0.5f;

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps);

}  // namespace zero_cal
