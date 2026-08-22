#pragma once
#include "types.h"

class MahonyFilter {
public:
    void begin(float sample_hz, float kp = 0.5f, float ki = 0.0f);
    void update(Vec3 gyro_dps, Vec3 accel_g);
    void set_bias(Vec3 gyro_bias_dps) { bias_ = gyro_bias_dps; }
    Vec3 gravity() const;
    void reset();

    // Low-pass the accelerometer BEFORE it is used as the gravity reference.
    //
    // Gravity is a DC quantity; sharpening-stroke acceleration is oscillatory and
    // very nearly zero-mean over a stroke cycle. Averaging the accel therefore
    // removes the stroke disturbance while leaving gravity intact. The gyro path
    // is deliberately untouched — in a complementary filter the gyro carries fast
    // re-orientation and the accel supplies only the slow anchor, so smoothing the
    // anchor costs no responsiveness.
    //
    // tau_s = 0 disables it (raw accel — the pre-0.3.0 behaviour), which is what
    // makes an A/B comparison on one device possible.
    void set_accel_tau(float tau_s) { a_tau_ = (tau_s > 0.0f) ? tau_s : 0.0f; a_lp_valid_ = false; }
    // The smoothed gravity reference actually fed to the correction step, for
    // diagnostics. Equals the last raw accel when smoothing is off.
    Vec3 accel_reference() const { return a_lp_valid_ ? a_lp_ : Vec3{0.0f, 0.0f, 0.0f}; }

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

    // Accel-reference low-pass state (see set_accel_tau).
    float a_tau_      = 0.0f;                  // 0 = disabled
    Vec3  a_lp_       = {0.0f, 0.0f, 0.0f};
    bool  a_lp_valid_ = false;
};

namespace mahony {
    // Thresholds for the snap-to-raw recovery. All bring-up tunable — see
    // docs/superpowers/bringup/2026-04-23-hardware-bringup.md.
    constexpr float SNAP_GYRO_DPS       = 3.0f;   // "still": only re-anchor when truly set down,
                                                  // not during in-hand handling (avoids angle twitch)
    constexpr float SNAP_ACCEL_TOL_G    = 0.10f;  // accel must be within this of 1g (pure gravity)
    constexpr float SNAP_DIVERGENCE_DEG = 8.0f;   // only snap when filter is this far off raw
    constexpr uint8_t SNAP_COOLDOWN_TICKS = 20;   // ~400ms at 50Hz: don't re-snap every tick

    // The accelerometer is only a valid gravity reference when |a| is near 1g.
    // During a stroke (linear acceleration) it isn't — gate the Mahony correction
    // on this so the angle doesn't spike at stroke turnarounds.
    //
    // CAUTION — this test is nearly blind to HORIZONTAL acceleration, which is the
    // direction a sharpening stroke actually pushes. Perpendicular components add
    // in quadrature, so 0.18 g of sweep (the level StrokeFSM counts as a pass)
    // tilts the measured vector 10.2 deg while moving |a| by only 0.016 g — a
    // ninth of what is needed to reject it. It does not reject a horizontal
    // disturbance until ~0.57 g. Treat it as a coarse outlier reject for gross
    // vertical events (drops, lifts), NOT as stroke rejection; the accel-reference
    // low-pass in MahonyFilter::set_accel_tau is what actually removes the stroke.
    constexpr float ACCEL_TRUST_TOL_G   = 0.15f;

    // Time constant of that accel-reference low-pass, in seconds. Chosen to sit
    // well below a 1-3 Hz stroke: at tau = 0.7 s the corner is ~0.23 Hz, which
    // attenuates a 1 Hz disturbance ~4x and 2 Hz ~9x before the Mahony correction
    // (itself a 1/kp = 1.25 s low-pass) sees it at all.
    constexpr float ACCEL_LP_TAU_S      = 0.7f;

    // True when the filter's gravity estimate has drifted from a trustworthy raw
    // gravity reading while the device is held still — the cue to nudge_to_gravity().
    bool should_snap(Vec3 g_filter, Vec3 accel_g, Vec3 gyro_dps);
}
