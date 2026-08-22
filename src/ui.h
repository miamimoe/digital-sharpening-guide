#pragma once
#include "types.h"

namespace ui {
    struct ActiveView {
        ColorState color;
        Side       current_side;
        uint32_t   strokes_A;
        uint32_t   strokes_B;
        bool       buzzer_flash;      // true = draw BUZZER ON/OFF overlay
        bool       buzzer_flash_on;
        float      angle_deg;         // live sharpening angle (secondary readout)
        bool       steady;            // hold the shown degree against sub-degree chatter
    };

    void begin();
    void clear();
    void draw_boot();
    void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset);
    void draw_set_tolerance(Tolerance tol, bool steady);
    void draw_active(const ActiveView& v);
    void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b,
                      uint32_t duration_s, uint8_t green_pct);
    // Past sessions, newest first. Shows nothing but a hint when empty.
    void draw_history(const SessionRecord* recs, int count);
    void draw_fault(FaultCode code);
    void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining);

    // VERIFY: accuracy check against an external reference.
    void draw_verify_prompt();                 // "lay flat, press A"
    void draw_verify_capture(int remaining_ms, bool moving);
    void draw_verify_reading(float deg);       // live angle at 0.1 deg resolution

    // step: 1 (side A) or 2 (side B). retry: true if last attempt failed stillness gate.
    void draw_zero_cal_prompt(int step, bool retry);
    // remaining_ms: time left in current capture window (warmup + averaging combined).
    // moving: device is not still this tick (gate failing) — show a motion warning
    // so a frozen-looking countdown is understood as "you're moving it".
    void draw_zero_cal_progress(int remaining_ms, bool moving);

    void set_backlight(uint8_t percent); // 0..100
}
