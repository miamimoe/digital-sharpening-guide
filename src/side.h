#pragma once
#include <cstdint>
#include "types.h"

class SideFSM {
public:
    static constexpr float    SPIKE_DEVIATION_G     = 0.5f;
    static constexpr float    SETTLE_TOL_G          = 0.1f;
    static constexpr uint32_t SETTLE_REQUIRED_MS    = 500;
    static constexpr uint32_t POST_SPIKE_TIMEOUT_MS = 5000;
    static constexpr uint32_t SUPPRESS_MS           = 2000;

    void  update(uint32_t now_ms, float accel_mag_g, float grav_dot_ref);
    void  manual_toggle(uint32_t now_ms);
    bool  consume_switch();
    Side  current_side() const { return side_; }
    void  reset();

private:
    enum Phase : uint8_t { STABLE, WAITING_SETTLE };

    Phase    phase_              = STABLE;
    uint32_t phase_entered_ms_   = 0;
    uint32_t settle_started_ms_  = 0;
    bool     settling_           = false;
    uint32_t suppress_until_ms_  = 0;
    bool     suppress_armed_     = false;
    bool     switch_pending_     = false;
    Side     side_               = Side::A;
};
