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
    if (cfg.dim_ms == 0) {
        ui::set_backlight(100);
        return;
    }
    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    uint32_t idle = now_ms - reference;
    uint8_t pct = (idle >= cfg.dim_ms) ? 10 : 100;
    ui::set_backlight(pct);
}

#ifndef UNIT_TEST
[[noreturn]] void enter_deep_sleep() {
    // Cut backlight via M5Unified's power abstraction. On M5StickC Plus this
    // routes to AXP192 LDO3 which otherwise stays on during deep sleep.
    M5.Display.setBrightness(0);
    M5.Power.setExtOutput(false);

    // Wake from power key: on M5StickC Plus the AXP192 PEK is routed to GPIO35.
    // If board revision differs and button isn't recognized after deep sleep,
    // bring-up checklist item 9 tracks the fallback (trying GPIO37 for BtnA).
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
