#pragma once
#include <cstdint>
#include "types.h"

namespace power {
    struct IdleConfig {
        uint32_t dim_ms;     // 0 = never dim
        uint32_t sleep_ms;   // 0 = never sleep
    };

    IdleConfig config_for(State s);

    void begin();

    // Returns true if the caller should begin the sleep sequence.
    bool check_idle(uint32_t now_ms, State current,
                    uint32_t last_activity_ms,
                    uint32_t last_stroke_ms);

    void update_backlight(uint32_t now_ms, State current,
                          uint32_t last_activity_ms, uint32_t last_stroke_ms);

    [[noreturn]] void enter_deep_sleep();

    // ---- Battery indicator -------------------------------------------------
    struct BatteryView {
        int  bars;       // 0..4 filled bars; -1 = level unknown (PMIC read failed)
        bool charging;
    };

    // Quantize a 0..100 percentage into 0..4 bars with a +-3 % hysteresis band
    // around each boundary so a reading hovering on 40 % does not flicker the
    // icon. prev_bars is the currently shown count (-1 if none yet). Negative pct
    // (M5Unified's error codes) returns -1.
    int battery_bars(int pct, int prev_bars);

    // Polls the PMIC at most once every BATTERY_POLL_MS (an I2C transaction on
    // the bus shared with the IMU on the Plus, so it must not run per tick) and
    // returns the cached view. Safe to call every tick.
    constexpr uint32_t BATTERY_POLL_MS = 10000;
    BatteryView battery_sample(uint32_t now_ms);
}
