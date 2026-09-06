#include "peel.h"
#include <cmath>

void PeelFSM::update(uint32_t now_ms, float accel_mag_g, float gyro_mag_dps) {
    if ((int32_t)(suppress_until_ms_ - now_ms) > 0) {
        phase_ = Phase::IDLE;
        return;
    }

    const bool airborne = std::fabs(accel_mag_g - 1.0f) > AIR_ACCEL_TOL_G
                       || gyro_mag_dps > AIR_GYRO_DPS;
    const bool settled  = std::fabs(accel_mag_g - 1.0f) <= SETTLE_TOL_G
                       && gyro_mag_dps < SETTLE_GYRO_DPS;

    switch (phase_) {
        case Phase::IDLE:
            if (airborne) {
                phase_            = Phase::AIRBORNE;
                phase_started_ms_ = now_ms;
            }
            break;
        case Phase::AIRBORNE:
            if (airborne) {
                break;   // still in the air — keep accumulating
            }
            if (now_ms - phase_started_ms_ >= AIR_REQUIRED_MS) {
                phase_            = Phase::WAIT_SETTLE;
                phase_started_ms_ = now_ms;
            } else {
                phase_ = Phase::IDLE;   // a twitch, not a peel
            }
            break;
        case Phase::WAIT_SETTLE:
            if (settled) {
                if (now_ms - phase_started_ms_ >= SETTLE_REQUIRED_MS) {
                    settled_pending_ = true;
                    phase_           = Phase::IDLE;
                    suppress(now_ms);
                }
            } else if (airborne) {
                phase_            = Phase::AIRBORNE;
                phase_started_ms_ = now_ms;
            } else {
                // In between — still waiting to sit down. Keep the settle clock
                // from starting until we actually look settled.
                phase_started_ms_ = now_ms;
            }
            break;
    }
}

bool PeelFSM::consume_settled() {
    bool v = settled_pending_;
    settled_pending_ = false;
    return v;
}

void PeelFSM::reset() {
    phase_             = Phase::IDLE;
    phase_started_ms_  = 0;
    suppress_until_ms_ = 0;
    settled_pending_   = false;
}

void PeelFSM::suppress(uint32_t now_ms) {
    suppress_until_ms_ = now_ms + SUPPRESS_MS;
    phase_             = Phase::IDLE;
}
