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
    };

    void begin();
    void clear();
    void draw_boot();
    void draw_bias_cal(int seconds_remaining);
    void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset);
    void draw_set_tolerance(Tolerance tol);
    void draw_active(const ActiveView& v);
    void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b, uint32_t duration_s);
    void draw_fault(FaultCode code);
    void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining);

    // step: 1 (side A) or 2 (side B). retry: true if last attempt failed stillness gate.
    void draw_zero_cal_prompt(int step, bool retry);
    // remaining_ms: time left in current capture window (warmup or averaging combined).
    void draw_zero_cal_progress(int remaining_ms);

    void set_backlight(uint8_t percent); // 0..100
}
