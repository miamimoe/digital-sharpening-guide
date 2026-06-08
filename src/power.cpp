#include "power.h"
#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include <esp_sleep.h>
#endif

namespace power {

IdleConfig config_for(State s) {
    switch (s) {
        case State::BOOT:
        case State::BIAS_CAL:
        case State::ZERO_CAL:
        case State::FAULT:
        case State::RESUME_PROMPT:
        case State::SLEEP:
            return {0, 0};
        case State::REZERO:         return {0, 0};            // never sleep mid-capture
        case State::SET_TARGET:     return { 90000, 120000};
        case State::SET_TOLERANCE:  return { 60000,  90000};
        case State::ACTIVE:         return {180000, 300000}; // strokes-based
        case State::SUMMARY:        return { 60000,  90000};
    }
    __builtin_unreachable();
}

void begin() {}

bool check_idle(uint32_t now_ms, State current,
                uint32_t last_activity_ms, uint32_t last_stroke_ms)
{
    auto cfg = config_for(current);
    if (cfg.sleep_ms == 0) return false;

    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    return (now_ms - reference) >= cfg.sleep_ms;
}

void update_backlight(uint32_t now_ms, State current,
                      uint32_t last_activity_ms, uint32_t last_stroke_ms)
{
    auto cfg = config_for(current);
    // ACTIVE baseline is a mild discount from full — bright enough to read the
    // color cue clearly under typical indoor light, while saving some battery.
    // Other states stay at 100% for readable text/numbers.
    uint8_t baseline_pct = (current == State::ACTIVE) ? 80 : 100;
    // Dim level for idle — lower than baseline but still faintly visible so
    // the user sees the device hasn't slept yet.
    uint8_t dim_pct = (current == State::ACTIVE) ? 30 : 15;

    if (cfg.dim_ms == 0) {
        ui::set_backlight(baseline_pct);
        return;
    }
    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    uint32_t idle = now_ms - reference;
    uint8_t pct = (idle >= cfg.dim_ms) ? dim_pct : baseline_pct;
    ui::set_backlight(pct);
}

#ifndef UNIT_TEST
[[noreturn]] void enter_deep_sleep() {
    M5.Display.setBrightness(0);
    // Use M5Unified's board-aware deep sleep: it calls Display.sleep() (cutting
    // the backlight rail correctly for this board) and configures the power-key
    // wake source (esp_sleep_enable_ext0_wakeup on GPIO35 for M5StickC Plus).
    // Hand-rolling the EXT1 mask was both backlight- and wake-source-fragile.
    M5.Power.deepSleep();
    while (true) {}  // deepSleep() does not return
}
#else
[[noreturn]] void enter_deep_sleep() {
    while (true) {}
}
#endif

} // namespace power
