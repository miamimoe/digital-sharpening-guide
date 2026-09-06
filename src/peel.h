#pragma once
#include <cstdint>

// Detects a peel-off-the-stone (airborne) followed by a settle. A sharpening
// stroke stays near 1 g and must not fire this. Used to ask SAME / OTHER
// because gravity does not reverse when the user flips the knife, not the stick.
class PeelFSM {
public:
    static constexpr float    AIR_ACCEL_TOL_G     = 0.35f;
    static constexpr float    AIR_GYRO_DPS        = 80.0f;
    static constexpr uint32_t AIR_REQUIRED_MS     = 200;
    static constexpr float    SETTLE_TOL_G        = 0.12f;
    static constexpr float    SETTLE_GYRO_DPS     = 20.0f;
    static constexpr uint32_t SETTLE_REQUIRED_MS  = 300;
    static constexpr uint32_t SUPPRESS_MS         = 1500;

    void update(uint32_t now_ms, float accel_mag_g, float gyro_mag_dps);
    bool consume_settled();
    void reset();
    void suppress(uint32_t now_ms);

private:
    enum class Phase : uint8_t { IDLE, AIRBORNE, WAIT_SETTLE };

    Phase    phase_             = Phase::IDLE;
    uint32_t phase_started_ms_  = 0;
    uint32_t suppress_until_ms_ = 0;
    bool     settled_pending_   = false;
};
