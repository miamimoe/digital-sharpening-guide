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

    // Standby is two-stage. The device first deep-sleeps (one power-key click
    // wakes it and an in-progress session resumes). If nobody wakes it within
    // OFF_AFTER_SLEEP_S, a timer wake runs finish_power_off_if_timer_wake(),
    // which cuts power at the PMIC. Deep sleep alone is NOT off on the Plus /
    // Plus2: the PMIC rails and the regulator stay up, and a ~120 mAh cell left
    // "asleep" on a shelf is flat within days. Nothing on a dark screen tells
    // the user which of the two states it is in, so the firmware must not rely
    // on them holding the power key for a real power-off.
    // Override with -D SG_OFF_AFTER_SLEEP_S=<n> to bench-test the hand-over.
#ifdef SG_OFF_AFTER_SLEEP_S
    constexpr uint32_t OFF_AFTER_SLEEP_S = SG_OFF_AFTER_SLEEP_S;
#else
    constexpr uint32_t OFF_AFTER_SLEEP_S = 30 * 60;
#endif

    [[noreturn]] void enter_deep_sleep();

    // Call first thing in setup(), BEFORE M5.begin(). Returns normally on any
    // boot that is not the standby timer; on the timer wake it powers the
    // device off and does not return.
    void finish_power_off_if_timer_wake();

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

    // ---- Battery level from voltage (AXP192 boards) --------------------------
    // M5Unified maps VBAT to percent in a straight line (about 3.38 V = 0 %,
    // 4.07 V = 100 %). A LiPo is nothing like linear: it sits on a 3.7-3.85 V
    // plateau for most of its capacity, so the linear map shows "half" when the
    // cell is nearly empty, and the terminal voltage it reads is also pushed up
    // by the charger and pulled down by the load. Measured on a Plus, 24 Nov
    // 2026: 3.78 V while charging at 89 mA, 3.72 V a few seconds after the
    // charger was switched off, which the library called 60 % and 52 % for a
    // cell that was really near 20 %.
    //
    // battery_ocv_mv removes the charge/load offset (current_ma > 0 charging,
    // < 0 discharging, as M5.Power.getBatteryCurrent() reports it);
    // lipo_percent then reads the resting-voltage curve.
    constexpr int BATTERY_R_MOHM = 600;   // cell + path resistance, measured as above
    int battery_ocv_mv(int vbat_mv, int current_ma);
    int lipo_percent(int ocv_mv);         // 0..100

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
