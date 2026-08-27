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

    // Last VERIFY reading rendered, so the 0.1 deg readout repaints only on
    // change and the static header is drawn once per entry.
    char           s_last_verify[12]   = "";
    bool           s_last_verify_valid = false;

    // Currently-shown angle, held against sub-degree chatter in steady mode
    // (see draw_active). Distinct from s_last_angle, which is the rendered text
    // used for dirty-region suppression.
    float          s_shown_deg   = 0.0f;
    bool           s_shown_valid = false;

    // Battery icon (top-right corner). Repaints only when the corner was wiped
    // by a full-screen fill or when the shown level / charging state changes.
    uint16_t s_bg          = COL_BLACK;   // background under the icon (set by wipe)
    bool     s_batt_wiped  = true;
    int      s_batt_bars   = -2;          // impossible sentinel; -1 = unknown
    bool     s_batt_chg    = false;

    // Every full-screen fill goes through here so the battery icon knows it
    // has to repaint. Do not call M5.Display.fillScreen() directly.
    void wipe(uint16_t bg) {
        M5.Display.fillScreen(bg);
        s_bg         = bg;
        s_batt_wiped = true;
    }

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
    wipe(COL_BLACK);
    s_last_valid = false;
    s_last_angle_valid = false;
    s_last_zc_tenths_valid = false;
    s_shown_valid = false;
    s_last_verify_valid = false;
}

void draw_boot() {
    clear();
    draw_centered("SHARPENING", 38, 2, COL_WHITE, COL_BLACK);
    draw_centered("GUIDE",      64, 2, COL_WHITE, COL_BLACK);
    draw_centered("v1.0.2", 100, 1, COL_WHITE, COL_BLACK);
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

void draw_set_tolerance(Tolerance tol, bool steady) {
    clear();
    draw_centered("TOLERANCE", 8, 1, COL_WHITE, COL_BLACK);
    const char* label = "NORMAL +-3";
    switch (tol) {
        case Tolerance::TIGHT:  label = "TIGHT +-2";  break;
        case Tolerance::NORMAL: label = "NORMAL +-3"; break;
        case Tolerance::EASY:   label = "EASY +-5";   break;
    }
    draw_centered(label, 46, 3, COL_WHITE, COL_BLACK);
    // Beta A/B switch. Lives here because this screen already repaints on input
    // and is the last stop before a session starts.
    draw_centered(steady ? "STEADY: ON" : "STEADY: OFF", 84, 2, COL_WHITE, COL_BLACK);
    draw_centered("A:Confirm  B:Chg  B-hold:Steady", 118, 1, COL_WHITE, COL_BLACK);
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
        wipe(bg);
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
    // The number repaints below do not cover the whole overlay rect (left-column
    // rows 96-126 under x<120 would stay black), so erase it first. A color
    // change already wiped the full screen.
    if (flash_ended && !color_changed) {
        M5.Display.fillRect(30, 96, 180, 30, bg);
    }

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

    // Left column: live angle as a whole number. Rounding alone is not enough —
    // a value hovering near x.5 flips between two integers forever. In steady mode
    // the shown degree only moves once the live angle clears it by more than half
    // a degree plus a margin, which leaves a ~0.3 deg deadband and stops the
    // chatter without adding any lag to a real change.
    if (v.steady) {
        if (!s_shown_valid || std::fabs(v.angle_deg - s_shown_deg) > 0.65f) {
            s_shown_deg  = (float)std::lround(v.angle_deg);
            s_shown_valid = true;
        }
    } else {
        s_shown_deg   = v.angle_deg;
        s_shown_valid = true;
    }
    char abuf[12];
    std::snprintf(abuf, sizeof abuf, "%ld", std::lround(s_shown_deg));
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

void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b,
                  uint32_t duration_s, uint8_t green_pct) {
    clear();
    draw_centered("SESSION", 4, 2, COL_WHITE, COL_BLACK);
    const char* t = (tol == Tolerance::TIGHT) ? "T2" : (tol == Tolerance::NORMAL) ? "N3" : "E5";
    char buf[48];
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    std::snprintf(buf, sizeof buf, "Target: %d  %s", (int)target_deg, t);
    M5.Display.setCursor(12, 30);  M5.Display.print(buf);
    std::snprintf(buf, sizeof buf, "A:%u  B:%u", (unsigned)a, (unsigned)b);
    M5.Display.setCursor(12, 52);  M5.Display.print(buf);
    // Time on-angle, in green, because it is the score that should improve.
    M5.Display.setTextColor(COL_GREEN, COL_BLACK);
    std::snprintf(buf, sizeof buf, "On-angle %u%%", (unsigned)green_pct);
    M5.Display.setCursor(12, 74);  M5.Display.print(buf);
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(1);
    std::snprintf(buf, sizeof buf, "Time %02u:%02u", (unsigned)(duration_s/60), (unsigned)(duration_s%60));
    M5.Display.setCursor(12, 98);  M5.Display.print(buf);
    draw_centered("A:New  B:Sleep  B-hold:Past", 118, 1, COL_WHITE, COL_BLACK);
}

void draw_verify_prompt() {
    clear();
    draw_centered("ACCURACY CHECK", 8, 2, COL_WHITE, COL_BLACK);
    draw_centered("Lay it flat on the stone", 42, 1, COL_WHITE, COL_BLACK);
    draw_centered("(or any flat surface),", 56, 1, COL_WHITE, COL_BLACK);
    draw_centered("press A, hold still.", 70, 1, COL_WHITE, COL_BLACK);
    draw_centered("Then tilt to a known angle.", 90, 1, COL_WHITE, COL_BLACK);
    draw_centered("A:Start   B:Back", 118, 1, COL_WHITE, COL_BLACK);
}

void draw_verify_capture(int remaining_ms, bool moving) {
    // Same 10 Hz throttle as the zero-cal countdown: repaint only when the tenths
    // digit or the moving state actually changes.
    int tenths = remaining_ms / 100;
    if (tenths < 0) tenths = 0;
    if (s_last_zc_tenths_valid && tenths == s_last_zc_tenths && moving == s_last_zc_moving) return;
    s_last_zc_tenths       = tenths;
    s_last_zc_tenths_valid = true;
    s_last_zc_moving       = moving;

    wipe(COL_BLACK);
    draw_centered("ACCURACY CHECK", 8, 2, COL_WHITE, COL_BLACK);
    char buf[16];
    std::snprintf(buf, sizeof buf, "%d.%d", tenths / 10, tenths % 10);
    draw_centered(buf, 48, 4, COL_WHITE, COL_BLACK);
    draw_centered(moving ? "KEEP STILL" : "Hold still...", 100, 1,
                  moving ? COL_RED : COL_WHITE, COL_BLACK);
}

void draw_verify_reading(float deg) {
    // 0.1 deg here, deliberately unlike the whole-degree ACTIVE readout: this is
    // the one screen where you WANT the extra digit, because you are comparing
    // against an external reference rather than trying to hold a value steady.
    char buf[12];
    std::snprintf(buf, sizeof buf, "%.1f", (double)deg);
    if (s_last_verify_valid && std::strcmp(buf, s_last_verify) == 0) return;

    if (!s_last_verify_valid) {
        wipe(COL_BLACK);
        draw_centered("ACCURACY CHECK", 8, 2, COL_WHITE, COL_BLACK);
        draw_centered("Compare with your reference", 104, 1, COL_WHITE, COL_BLACK);
        draw_centered("A-hold or B: done", 120, 1, COL_WHITE, COL_BLACK);
    }
    M5.Display.fillRect(0, 40, SCR_W, 8 * 6, COL_BLACK);
    draw_centered(buf, 46, 6, COL_WHITE, COL_BLACK);
    std::strncpy(s_last_verify, buf, sizeof s_last_verify - 1);
    s_last_verify[sizeof s_last_verify - 1] = '\0';
    s_last_verify_valid = true;
}

void draw_history(const SessionRecord* recs, int count) {
    clear();
    draw_centered("PAST SESSIONS", 4, 2, COL_WHITE, COL_BLACK);
    if (count <= 0) {
        draw_centered("Nothing yet -", 52, 1, COL_WHITE, COL_BLACK);
        draw_centered("finish a session first.", 66, 1, COL_WHITE, COL_BLACK);
        draw_centered("Any button: back", 118, 1, COL_WHITE, COL_BLACK);
        return;
    }
    // Newest first, one row each. Columns: angle, on-angle %, strokes, time.
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setCursor(8, 26);
    M5.Display.print("ANGLE  ON-ANGLE  STROKES   TIME");
    char buf[48];
    for (int i = 0; i < count && i < kSessionHistoryMax; i++) {
        const SessionRecord& r = recs[i];
        const int y = 40 + i * 15;
        std::snprintf(buf, sizeof buf, "%4.1f%c    %3u%%     %5u   %02u:%02u",
                      (double)r.target_deg_x10 / 10.0, 0xF8 /* degree glyph */,
                      (unsigned)r.green_pct,
                      (unsigned)(r.strokes_a + r.strokes_b),
                      (unsigned)(r.duration_s / 60), (unsigned)(r.duration_s % 60));
        // Newest row highlighted so "this one is the session you just finished"
        // reads without counting rows.
        M5.Display.setTextColor(i == 0 ? COL_GREEN : COL_WHITE, COL_BLACK);
        M5.Display.setCursor(8, y);
        M5.Display.print(buf);
    }
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    draw_centered("Any button: back", 122, 1, COL_WHITE, COL_BLACK);
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
    wipe(COL_BLACK);
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

    wipe(COL_BLACK);
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

void draw_battery(int bars, bool charging) {
    if (!s_batt_wiped && bars == s_batt_bars && charging == s_batt_chg) return;
    s_batt_wiped = false;
    s_batt_bars  = bars;
    s_batt_chg   = charging;

    // Geometry: 20x10 body at the top-right, 2x4 nub, four 3-px bars inside,
    // charging "+" to its left. The erase rect starts at x = PLUS_X - 1 = 205:
    // the widest header ("ACCURACY CHECK", size 2) ends at x = 204, so keep
    // PLUS_X >= 206 or that header's last glyph gets clipped.
    constexpr int BX = 214, BY = 3, BW = 20, BH = 10;
    constexpr int NUB_W = 2, NUB_H = 4;
    constexpr int PLUS_X = 206, PLUS_Y = 5;   // charging "+" glyph, 5x5

    // Erase the whole corner (glyph + body + nub) with the current background.
    M5.Display.fillRect(PLUS_X - 1, BY - 1, (BX + BW + NUB_W) - (PLUS_X - 1) + 1, BH + 2, s_bg);
    M5.Display.drawRect(BX, BY, BW, BH, COL_WHITE);
    M5.Display.fillRect(BX + BW, BY + (BH - NUB_H) / 2, NUB_W, NUB_H, COL_WHITE);

    if (bars < 0) {
        // Unknown level: a single dash in the middle rather than an empty body,
        // so "no reading" is not mistaken for "flat".
        M5.Display.fillRect(BX + 7, BY + BH / 2 - 1, 6, 2, COL_WHITE);
    } else {
        for (int i = 0; i < bars && i < 4; i++) {
            M5.Display.fillRect(BX + 2 + i * 4, BY + 2, 3, BH - 4, COL_WHITE);
        }
    }
    if (charging) {
        M5.Display.fillRect(PLUS_X + 2, PLUS_Y,     1, 5, COL_WHITE);
        M5.Display.fillRect(PLUS_X,     PLUS_Y + 2, 5, 1, COL_WHITE);
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
    void draw_set_tolerance(Tolerance, bool) {}
    void draw_active(const ActiveView&) {}
    void draw_summary(float, Tolerance, uint32_t, uint32_t, uint32_t, uint8_t) {}
    void draw_history(const SessionRecord*, int) {}
    void draw_verify_prompt() {}
    void draw_verify_capture(int, bool) {}
    void draw_verify_reading(float) {}
    void draw_fault(FaultCode) {}
    void draw_resume_prompt(float, Tolerance, uint32_t, uint32_t, int) {}
    void draw_zero_cal_prompt(int, bool) {}
    void draw_zero_cal_progress(int, bool) {}
    void draw_battery(int, bool) {}
    void set_backlight(uint8_t) {}
}
#endif
