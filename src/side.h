#pragma once
#include <cstdint>
#include "types.h"

// Side detection by gravity polarity. The device's back faces the opposite blade
// flat after a flip, so gravity (vs the side-A zero) reverses sign. We don't try
// to catch the "peel" event — we simply notice when the device has come to rest
// in the orientation belonging to the OTHER side and switch then. This is robust
// to a slow hand-swap (peel, switch hands, reposition, reattach) that the old
// spike+5s-timeout scheme would miss.
class SideFSM {
public:
    static constexpr float    SETTLE_TOL_G       = 0.1f;   // accel within this of 1g => "at rest"
    static constexpr float    SETTLE_GYRO_DPS    = 15.0f;  // and gyro below this
    static constexpr float    FLIP_POLARITY_MIN  = 0.3f;   // grav_dot_ref must clearly indicate the other side
    static constexpr uint32_t SETTLE_REQUIRED_MS = 500;    // held in the flipped pose this long => switch
    static constexpr uint32_t SUPPRESS_MS        = 2000;   // ignore auto-detect after a manual toggle

    // grav_dot_ref = dot(gravity_now, g_zero_A): positive on side A, negative on side B.
    void  update(uint32_t now_ms, float accel_mag_g, float gyro_mag_dps, float grav_dot_ref);
    void  manual_toggle(uint32_t now_ms);
    bool  consume_switch();
    Side  current_side() const { return side_; }
    void  reset();

    // For session-restore only. Bypasses any FSM state; caller is responsible
    // for ensuring this is only called during wake-from-sleep.
    void restore_side(Side s) { side_ = s; }

private:
    Side     side_              = Side::A;
    bool     switch_pending_    = false;
    bool     flipping_          = false;  // accumulating time settled in the opposite-side pose
    uint32_t flip_started_ms_   = 0;
    uint32_t suppress_until_ms_ = 0;
    bool     suppress_armed_    = false;
};
