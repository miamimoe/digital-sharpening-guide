#include "power.h"
#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include <esp_sleep.h>
#include "board.h"
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
    switch (board::variant()) {
        case board::Variant::PLUS:
            // M5Unified's deepSleep() does NOT arm a wake source on the AXP192
            // Plus, so do it ourselves: EXT0 on GPIO35 (the AXP192 IRQ line,
            // active-low) — a short power-key press asserts it and wakes the chip.
            // Clear all pending AXP IRQ flags first (write-1-to-clear), else the
            // line may already be low and the device would wake instantly.
            // SG_BOARD_PLUS guard: M5.Power.Axp192 is only a member of Power_Class
            // when M5Unified is compiled for AXP192-bearing targets; gating here
            // prevents a type error when this translation unit is compiled for S3.
#if defined(SG_BOARD_PLUS)
            M5.Power.Axp192.writeRegister8(0x44, 0xFF);
            M5.Power.Axp192.writeRegister8(0x45, 0xFF);
            M5.Power.Axp192.writeRegister8(0x46, 0xFF);
            M5.Power.Axp192.writeRegister8(0x47, 0xFF);
            M5.Power.Axp192.writeRegister8(0x4D, 0xFF);
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
#endif
            M5.Power.deepSleep();  // EXT0 armed above → wakes on power key, RTC session preserved
            break;
        case board::Variant::PLUS2:
            // The Plus2 has no AXP192, but M5Unified sets its internal _wakeupPin to
            // the power button (GPIO35) for this board, so deepSleep() arms EXT0
            // itself. Power key wakes it; the RTC-RAM session is preserved.
            M5.Power.deepSleep();
            break;
        case board::Variant::S3:
            // The StickS3 power button is owned by the M5PM1 PMIC (read over I2C),
            // NOT an RTC-capable GPIO — so deep sleep would arm NO wake source and
            // the device could never wake (M5Unified 0.2.14 leaves _wakeupPin unset
            // on this board). Use a clean PMIC power-off instead: a power-key press
            // re-powers the device. Trade-off vs. Plus/Plus2: the S3 cold-boots on
            // wake, so the in-progress session in RTC RAM is NOT resumed. That is an
            // inherent StickS3 limitation, and better than a wake-less deep sleep.
            M5.Power.powerOff();
            break;
    }
    while (true) {}  // neither deepSleep() nor powerOff() returns
}
#else
[[noreturn]] void enter_deep_sleep() {
    while (true) {}
}
#endif

} // namespace power
