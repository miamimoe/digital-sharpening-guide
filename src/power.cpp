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
        case State::FAULT:
        case State::RESUME_PROMPT:
        case State::SLEEP:
            return {0, 0};
        case State::SET_TARGET:     return { 90000, 120000};
        case State::SET_TOLERANCE:  return { 60000,  90000};
        case State::ACTIVE:         return {180000, 300000}; // strokes-based
        case State::SUMMARY:        return { 60000,  90000};
    }
    return {0, 0};
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
    // ACTIVE uses 50% baseline — color is the primary signal and is easily
    // read peripherally at half-brightness. Other states use 100% for
    // readable text/numbers.
    uint8_t baseline_pct = (current == State::ACTIVE) ? 50 : 100;

    if (cfg.dim_ms == 0) {
        ui::set_backlight(baseline_pct);
        return;
    }
    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    uint32_t idle = now_ms - reference;
    uint8_t pct = (idle >= cfg.dim_ms) ? 10 : baseline_pct;
    ui::set_backlight(pct);
}

#ifndef UNIT_TEST
[[noreturn]] void enter_deep_sleep() {
    // Cut the display fully — Display::sleep() routes through the board HAL to
    // disable both the backlight LED AND the ST7789's internal power. This is
    // more reliable than setBrightness(0) which only PWMs to zero but leaves
    // the AXP192 LDO3 supplying quiescent current.
    M5.Display.sleep();
    M5.Display.setBrightness(0);

    // Wake from power key: M5StickC Plus routes the AXP192 PEK through GPIO35.
    const uint64_t WAKE_MASK = (1ULL << 35);
    esp_sleep_enable_ext1_wakeup(WAKE_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
    while (true) {}
}
#else
[[noreturn]] void enter_deep_sleep() {
    while (true) {}
}
#endif

} // namespace power
