#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include <cstring>
#include <cstdio>
#include <cmath>

// Landscape layout: M5StickC Plus panel is 135x240 native; setRotation(1) gives
// a 240(W) x 135(H) canvas, which is how the device is held in use.
namespace {
    constexpr int SCR_W = 240;
    constexpr int SCR_H = 135;

    constexpr uint16_t COL_GREEN = 0x07E0;
    constexpr uint16_t COL_RED   = 0xF800;
    constexpr uint16_t COL_BLUE  = 0x001F;
    constexpr uint16_t COL_BLACK = 0x0000;
    constexpr uint16_t COL_WHITE = 0xFFFF;

    ui::ActiveView s_last{};
    bool           s_last_valid = false;
    char           s_last_angle[12] = "";
    bool           s_last_angle_valid = false;

    // Throttle ZERO_CAL countdown to ~10 Hz (only repaint when tenths digit or
    // the moving-state changes).
    int  s_last_zc_tenths       = -1;
    bool s_last_zc_tenths_valid = false;
    bool s_last_zc_moving        = false;

    uint16_t color_for(ColorState c) {
        switch (c) {
            case ColorState::GREEN: return COL_GREEN;
            case ColorState::BLUE:  return COL_BLUE;
            case ColorState::RED:   return COL_RED;
        }
        __builtin_unreachable();
    }

    // Default GLCD font advances 6 px/char wide, 8 px tall, scaled by text size.
    int text_w(const char* s, int size) { return (int)std::strlen(s) * 6 * size; }

    void draw_centered(const char* s, int y, int size, uint16_t fg, uint16_t bg) {
        M5.Display.setTextColor(fg, bg);
        M5.Display.setTextSize(size);
        int x = (SCR_W - text_w(s, size)) / 2;
        if (x < 0) x = 0;
        M5.Display.setCursor(x, y);
        M5.Display.print(s);
    }

    // Center a string horizontally within the column [x0, x0 + region_w).
    void draw_centered_in(const char* s, int x0, int region_w, int y, int size,
                          uint16_t fg, uint16_t bg) {
        M5.Display.setTextColor(fg, bg);
        M5.Display.setTextSize(size);
        int x = x0 + (region_w - text_w(s, size)) / 2;
        if (x < x0) x = x0;
        M5.Display.setCursor(x, y);
        M5.Display.print(s);
    }
}

namespace ui {

void begin() {
    M5.Display.setRotation(1);   // landscape, 240x135
    M5.Display.setTextWrap(false);
    clear();
}

void clear() {
    M5.Display.fillScreen(COL_BLACK);
    s_last_valid = false;
    s_last_angle_valid = false;
    s_last_zc_tenths_valid = false;
}

void draw_boot() {
    clear();
    draw_centered("SHARPENING", 38, 2, COL_WHITE, COL_BLACK);
    draw_centered("GUIDE",      64, 2, COL_WHITE, COL_BLACK);
    draw_centered("v0.1.0",     100, 1, COL_WHITE, COL_BLACK);
}

void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset) {
    clear();
    draw_centered("SET TARGET", 8, 1, COL_WHITE, COL_BLACK);
    char buf[12];
    if (in_preset_mode && preset == PresetSelection::CANCEL) {
        draw_centered("CANCEL", 48, 3, COL_WHITE, COL_BLACK);
    } else {
        if (in_preset_mode) std::snprintf(buf, sizeof buf, "%d", (int)preset_degrees(preset));
        else                std::snprintf(buf, sizeof buf, "%.1f", live_angle_deg);
        draw_centered(buf, 42, 5, COL_WHITE, COL_BLACK);
    }
    draw_centered(in_preset_mode ? "A:Pick   B:Next" : "A:Confirm   B:Presets",
                  118, 1, COL_WHITE, COL_BLACK);
}

void draw_set_tolerance(Tolerance tol) {
    clear();
    draw_centered("TOLERANCE", 8, 1, COL_WHITE, COL_BLACK);
    const char* label = "NORMAL +-3";
    switch (tol) {
        case Tolerance::TIGHT:  label = "TIGHT +-2";  break;
        case Tolerance::NORMAL: label = "NORMAL +-3"; break;
        case Tolerance::EASY:   label = "EASY +-5";   break;
    }
    draw_centered(label, 52, 3, COL_WHITE, COL_BLACK);
    draw_centered("A:Confirm   B:Change", 118, 1, COL_WHITE, COL_BLACK);
}

void draw_active(const ActiveView& v) {
    // Two equal columns: ANGLE (left) | STROKE (right). Both rendered at the same
    // large text size so neither reads as secondary.
    constexpr int DIV_X   = 120;          // column divider / right-column origin
    constexpr int LABEL_Y = 24;           // small column headers
    constexpr int NUM_Y   = 52;           // big numbers (size 5 -> 40 px tall)
    constexpr int SUB_Y   = 102;          // other-side stroke count, under STROKE
    constexpr int NUM_SZ  = 5;
    const uint16_t bg = color_for(v.color);

    bool color_changed = !s_last_valid || s_last.color != v.color;
    if (color_changed) {
        M5.Display.fillScreen(bg);
        // Legend strip across the top.
        M5.Display.fillRect(8,   4, 12, 12, COL_BLUE);
        M5.Display.fillRect(78,  4, 12, 12, COL_GREEN);
        M5.Display.fillRect(146, 4, 12, 12, COL_RED);
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(24,  6); M5.Display.print("LOW");
        M5.Display.setCursor(94,  6); M5.Display.print("OK");
        M5.Display.setCursor(162, 6); M5.Display.print("HIGH");
        // Column divider + static headers.
        M5.Display.fillRect(DIV_X - 1, 22, 2, SCR_H - 22, COL_WHITE);
        draw_centered_in("ANGLE",  0,     DIV_X,         LABEL_Y, 2, COL_WHITE, bg);
        draw_centered_in("STROKE", DIV_X, SCR_W - DIV_X, LABEL_Y, 2, COL_WHITE, bg);
    }

    bool counts_changed =
        s_last.current_side != v.current_side ||
        s_last.strokes_A != v.strokes_A ||
        s_last.strokes_B != v.strokes_B;
    // The buzzer overlay sits over the lower band; when it clears (or a color
    // change wiped the screen) we must repaint the numbers it covered.
    bool flash_ended = s_last_valid && s_last.buzzer_flash && !v.buzzer_flash;

    // Right column: current-side stroke count (big) + other-side count (small).
    if (color_changed || counts_changed || flash_ended) {
        uint32_t big = (v.current_side == Side::A) ? v.strokes_A : v.strokes_B;
        uint32_t sm  = (v.current_side == Side::A) ? v.strokes_B : v.strokes_A;
        char other_label = (v.current_side == Side::A) ? 'B' : 'A';
        char buf[12];
        std::snprintf(buf, sizeof buf, "%u", (unsigned)big);
        M5.Display.fillRect(DIV_X + 1, NUM_Y, SCR_W - DIV_X - 1, 8 * NUM_SZ, bg);
        draw_centered_in(buf, DIV_X, SCR_W - DIV_X, NUM_Y, NUM_SZ, COL_WHITE, bg);
        char sbuf[16];
        std::snprintf(sbuf, sizeof sbuf, "%c:%u", other_label, (unsigned)sm);
        M5.Display.fillRect(DIV_X + 1, SUB_Y, SCR_W - DIV_X - 1, 16, bg);
        draw_centered_in(sbuf, DIV_X, SCR_W - DIV_X, SUB_Y, 2, COL_WHITE, bg);
    }

    // Left column: live angle as a whole number. Rounding to an integer also
    // means we only repaint when the displayed degree actually changes, so the
    // value no longer flickers on sub-degree jitter.
    char abuf[12];
    std::snprintf(abuf, sizeof abuf, "%ld", std::lround(v.angle_deg));
    if (color_changed || flash_ended
        || !s_last_angle_valid || std::strcmp(abuf, s_last_angle) != 0) {
        M5.Display.fillRect(0, NUM_Y, DIV_X - 1, 8 * NUM_SZ, bg);
        draw_centered_in(abuf, 0, DIV_X, NUM_Y, NUM_SZ, COL_WHITE, bg);
        std::strncpy(s_last_angle, abuf, sizeof s_last_angle - 1);
        s_last_angle[sizeof s_last_angle - 1] = '\0';
        s_last_angle_valid = true;
    }

    // Draw the buzzer-flash overlay only when it newly appears or the area under
    // it was just repainted (color change wiped the screen; a counts repaint
    // covers the right-column sub-label band the overlay overlaps).
    if (v.buzzer_flash && (!s_last_valid || !s_last.buzzer_flash
                           || color_changed || counts_changed)) {
        M5.Display.fillRect(30, 96, 180, 30, COL_BLACK);
        const char* msg = v.buzzer_flash_on ? "BUZZER ON" : "BUZZER OFF";
        draw_centered(msg, 102, 2, COL_WHITE, COL_BLACK);
    }

    s_last       = v;
    s_last_valid = true;
}

void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b, uint32_t duration_s) {
    clear();
    draw_centered("SESSION", 4, 2, COL_WHITE, COL_BLACK);
    const char* t = (tol == Tolerance::TIGHT) ? "T2" : (tol == Tolerance::NORMAL) ? "N3" : "E5";
    char buf[48];
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    std::snprintf(buf, sizeof buf, "Target: %d", (int)target_deg);
    M5.Display.setCursor(12, 32);  M5.Display.print(buf);
    std::snprintf(buf, sizeof buf, "Tol: %s   A:%u  B:%u", t, (unsigned)a, (unsigned)b);
    M5.Display.setCursor(12, 56);  M5.Display.print(buf);
    std::snprintf(buf, sizeof buf, "Time %02u:%02u", (unsigned)(duration_s/60), (unsigned)(duration_s%60));
    M5.Display.setCursor(12, 80);  M5.Display.print(buf);
    draw_centered("A:New   B:Sleep", 118, 1, COL_WHITE, COL_BLACK);
}

void draw_fault(FaultCode code) {
    clear();
    draw_centered("IMU FAULT", 24, 3, COL_RED, COL_BLACK);
    char buf[8];
    std::snprintf(buf, sizeof buf, "E%02u", (unsigned)code);
    draw_centered(buf, 62, 3, COL_RED, COL_BLACK);
    draw_centered("Power-cycle to retry", 112, 1, COL_WHITE, COL_BLACK);
}

void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining) {
    clear();
    draw_centered("RESUME?", 8, 3, COL_WHITE, COL_BLACK);
    const char* t = (tol == Tolerance::TIGHT) ? "2" : (tol == Tolerance::NORMAL) ? "3" : "5";
    char buf[40];
    std::snprintf(buf, sizeof buf, "Tgt:%d  Tol:+-%s", (int)target_deg, t);
    draw_centered(buf, 48, 2, COL_WHITE, COL_BLACK);
    std::snprintf(buf, sizeof buf, "A:%u   B:%u", (unsigned)a, (unsigned)b);
    draw_centered(buf, 72, 2, COL_WHITE, COL_BLACK);
    std::snprintf(buf, sizeof buf, "%d", seconds_remaining);
    draw_centered(buf, 94, 2, COL_WHITE, COL_BLACK);
    draw_centered("A:Resume   B:New", 120, 1, COL_WHITE, COL_BLACK);
}

void draw_zero_cal_prompt(int step, bool retry) {
    s_last_zc_tenths_valid = false;
    M5.Display.fillScreen(COL_BLACK);
    char hdr[16];
    std::snprintf(hdr, sizeof hdr, "ZERO CAL  %d/2", step);
    draw_centered(hdr, 8, 2, COL_WHITE, COL_BLACK);
    draw_centered(step == 1 ? "Lay flat on stone" : "Raise to your angle", 40, 1, COL_WHITE, COL_BLACK);
    draw_centered("Press A, hold still", 60, 1, COL_WHITE, COL_BLACK);
    if (retry) {
        draw_centered("HOLD STILL", 92, 3, COL_RED, COL_BLACK);
    }
}

void draw_zero_cal_progress(int remaining_ms, bool moving) {
    int tenths = remaining_ms / 100;
    if (s_last_zc_tenths_valid && tenths == s_last_zc_tenths && moving == s_last_zc_moving) return;
    s_last_zc_tenths       = tenths;
    s_last_zc_tenths_valid = true;
    s_last_zc_moving       = moving;

    M5.Display.fillScreen(COL_BLACK);
    if (moving) {
        // The capture can't progress while the device is moving — say so loudly
        // instead of showing a frozen countdown, and offer the force-capture.
        draw_centered("KEEP STILL", 18, 3, COL_RED, COL_BLACK);
        draw_centered("set it down", 58, 1, COL_WHITE, COL_BLACK);
        draw_centered("or tap B to capture", 84, 1, COL_WHITE, COL_BLACK);
    } else {
        draw_centered("Hold still", 30, 2, COL_WHITE, COL_BLACK);
        char buf[16];
        std::snprintf(buf, sizeof buf, "%d.%ds", tenths / 10, tenths % 10);
        draw_centered(buf, 70, 4, COL_WHITE, COL_BLACK);
    }
}

void set_backlight(uint8_t percent) {
    // Brightness is an AXP192 I2C register write on this board (bus shared with
    // the MPU6886), so same-value writes must be skipped, not repeated every tick.
    static uint8_t s_last_pct = 255;   // impossible sentinel (range is 0..100)
    if (percent > 100) percent = 100;
    if (percent == s_last_pct) return;
    s_last_pct = percent;
    M5.Display.setBrightness((uint8_t)(percent * 255u / 100u));
}

} // namespace ui

#else
// Native stubs for tests.
namespace ui {
    void begin() {}
    void clear() {}
    void draw_boot() {}
    void draw_set_target(float, bool, PresetSelection) {}
    void draw_set_tolerance(Tolerance) {}
    void draw_active(const ActiveView&) {}
    void draw_summary(float, Tolerance, uint32_t, uint32_t, uint32_t) {}
    void draw_fault(FaultCode) {}
    void draw_resume_prompt(float, Tolerance, uint32_t, uint32_t, int) {}
    void draw_zero_cal_prompt(int, bool) {}
    void draw_zero_cal_progress(int, bool) {}
    void set_backlight(uint8_t) {}
}
#endif
