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

    // Gate a PMIC percentage on the raw VBAT millivolts behind it. The AXP192's
    // battery ADC samples at 25 Hz and is only enabled inside M5.begin(), so the
    // first poll after boot often reads 0 mV — which M5Unified's level formula
    // clamps to a "valid" 0 % instead of an error, painting an empty battery on
    // a full cell. No connected LiPo can sit below BATTERY_MV_VALID_MIN, so such
    // a reading means "ADC not ready" (or M5PM1's no-battery/undetermined codes)
    // and comes back as -1 (unknown). A plausible VBAT passes pct through as-is.
    constexpr int BATTERY_MV_VALID_MIN = 2500;
    int battery_level_gate(int pct, int batt_mv);

    // Polls the PMIC at most once every BATTERY_POLL_MS (an I2C transaction on
    // the bus shared with the IMU on the Plus, so it must not run per tick) and
    // returns the cached view. Safe to call every tick. Until the first gated
    // reading is seen — and only within the boot warm-up window — it retries at
    // BATTERY_RETRY_MS so a not-ready ADC shows the unknown dash for well under
    // a second instead of caching an empty icon for a full poll period.
    constexpr uint32_t BATTERY_POLL_MS   = 10000;
    constexpr uint32_t BATTERY_RETRY_MS  = 500;
    constexpr uint32_t BATTERY_WARMUP_MS = 5000;
    BatteryView battery_sample(uint32_t now_ms);
}
