#pragma once
#include "types.h"

class MahonyFilter {
public:
    void begin(float sample_hz, float kp = 0.5f, float ki = 0.0f);
    void update(Vec3 gyro_dps, Vec3 accel_g);
    void set_bias(Vec3 gyro_bias_dps) { bias_ = gyro_bias_dps; }
    Vec3 gravity() const;
    void reset();

    // Re-anchor the orientation so gravity() immediately equals the (normalized)
    // measured accel, discarding the unobservable yaw. Used to shortcut Mahony's
    // ~2 s convergence after a fast motion (side flip / ACTIVE entry) once the
    // device is verified still — the raw accelerometer IS gravity when stationary.
    // No-op if the accel sample has near-zero magnitude.
    void nudge_to_gravity(Vec3 accel_g);

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;
    float kp_ = 0.5f, ki_ = 0.0f;
    float dt_ = 0.01f;
    Vec3  bias_ = {0.0f, 0.0f, 0.0f};
};

namespace mahony {
    // Thresholds for the snap-to-raw recovery. All bring-up tunable — see
    // docs/superpowers/bringup/2026-04-23-hardware-bringup.md.
    constexpr float SNAP_GYRO_DPS       = 3.0f;   // "still": only re-anchor when truly set down,
                                                  // not during in-hand handling (avoids angle twitch)
    constexpr float SNAP_ACCEL_TOL_G    = 0.10f;  // accel must be within this of 1g (pure gravity)
    constexpr float SNAP_DIVERGENCE_DEG = 8.0f;   // only snap when filter is this far off raw
    constexpr uint8_t SNAP_COOLDOWN_TICKS = 20;   // ~400ms at 50Hz: don't re-snap every tick

    // True when the filter's gravity estimate has drifted from a trustworthy raw
    // gravity reading while the device is held still — the cue to nudge_to_gravity().
    bool should_snap(Vec3 g_filter, Vec3 accel_g, Vec3 gyro_dps);
}
