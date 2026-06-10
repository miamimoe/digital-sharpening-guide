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
        case State::RESUME_PROMPT:
        case State::SLEEP:
            return {0, 0};
        case State::ZERO_CAL:       return { 60000, 120000};
        // An in-progress REZERO capture never sleeps: app.cpp refreshes
        // last_activity_ms_ while the capture FSM is progressing or the user is
        // handling the device, so only an abandoned static screen idles out.
        case State::REZERO:         return { 60000, 120000};
        case State::FAULT:          return { 60000, 300000};
        case State::SET_TARGET:     return { 90000, 120000};
        case State::SET_TOLERANCE:  return { 60000,  90000};
        case State::ACTIVE:         return {180000, 300000}; // strokes-or-activity-based
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

    // ACTIVE keys on the LATER of strokes and other activity (buttons, motion).
    uint32_t reference = (current == State::ACTIVE)
        ? (last_stroke_ms > last_activity_ms ? last_stroke_ms : last_activity_ms)
        : last_activity_ms;
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
    // Same reference as check_idle: ACTIVE uses the later of strokes and activity.
    uint32_t reference = (current == State::ACTIVE)
        ? (last_stroke_ms > last_activity_ms ? last_stroke_ms : last_activity_ms)
        : last_activity_ms;
    uint32_t idle = now_ms - reference;
    uint8_t pct = (idle >= cfg.dim_ms) ? dim_pct : baseline_pct;
    ui::set_backlight(pct);
}

#ifndef UNIT_TEST
[[noreturn]] void enter_deep_sleep() {
    M5.Display.setBrightness(0);
    // M5Unified's deepSleep() only arms its _wakeupPin, which is UNSET on the
    // AXP192 StickC Plus (set only for the Plus2) — on this board it enters deep
    // sleep with no wake source, and the sole escape is the AXP hardware dance
    // (hold 6 s to cut power, then ~2 s press to boot). Arm EXT0 on GPIO35 (the
    // AXP192 IRQ line, active-low) ourselves: a short power-key press asserts
    // the IRQ and wakes the chip. PEK IRQs are enabled by AXP192 power-on
    // default. Clear all pending IRQ flags first (write-1-to-clear), otherwise
    // the line may already be low and the device would wake instantly.
    M5.Power.Axp192.writeRegister8(0x44, 0xFF);
    M5.Power.Axp192.writeRegister8(0x45, 0xFF);
    M5.Power.Axp192.writeRegister8(0x46, 0xFF);
    M5.Power.Axp192.writeRegister8(0x47, 0xFF);
    M5.Power.Axp192.writeRegister8(0x4D, 0xFF);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
    M5.Power.deepSleep();  // Display.sleep() + esp_deep_sleep_start(); EXT0 stays armed
    while (true) {}  // deepSleep() does not return
}
#else
[[noreturn]] void enter_deep_sleep() {
    while (true) {}
}
#endif

} // namespace power
