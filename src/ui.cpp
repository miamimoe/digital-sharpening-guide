#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace {
    constexpr uint16_t COL_GREEN = 0x07E0;
    constexpr uint16_t COL_RED   = 0xF800;
    constexpr uint16_t COL_BLUE  = 0x001F;
    constexpr uint16_t COL_BLACK = 0x0000;
    constexpr uint16_t COL_WHITE = 0xFFFF;

    ui::ActiveView s_last{};
    bool           s_last_valid = false;

    uint16_t color_for(ColorState c) {
        switch (c) {
            case ColorState::GREEN: return COL_GREEN;
            case ColorState::BLUE:  return COL_BLUE;
            case ColorState::RED:   return COL_RED;
        }
        return COL_BLACK;
    }
}

namespace ui {

void begin() {
    M5.Display.setRotation(0);
    M5.Display.setTextWrap(false);
    clear();
}

void clear() {
    M5.Display.fillScreen(COL_BLACK);
    s_last_valid = false;
}

void draw_boot() {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 60);
    M5.Display.print("SHARPENING");
    M5.Display.setCursor(30, 85);
    M5.Display.print("GUIDE");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(45, 160);
    M5.Display.print("v0.1.0");
}

void draw_bias_cal(int seconds_remaining) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 50);
    M5.Display.print("Hold still");
    M5.Display.setCursor(10, 80);
    M5.Display.print("Calibrating");
    M5.Display.setTextSize(4);
    M5.Display.setCursor(45, 130);
    M5.Display.printf("%d", seconds_remaining);
}

void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.print("SET TARGET");
    M5.Display.setTextSize(5);
    M5.Display.setCursor(20, 50);
    if (in_preset_mode) {
        if (preset == PresetSelection::CANCEL) {
            M5.Display.setTextSize(3);
            M5.Display.setCursor(10, 70);
            M5.Display.print("CANCEL");
        } else {
            M5.Display.printf("%02d", (int)preset_degrees(preset));
            M5.Display.print((char)176); // degree symbol (Latin-1 0xB0)
        }
    } else {
        M5.Display.printf("%4.1f", live_angle_deg);
    }
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200);
    M5.Display.print(in_preset_mode ? "A:Pick  B:Next" : "A:Confirm  B:Presets");
}

void draw_set_tolerance(Tolerance tol) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.print("TOLERANCE");
    const char* label = "NORMAL +-2";
    switch (tol) {
        case Tolerance::TIGHT:  label = "TIGHT  +-1"; break;
        case Tolerance::NORMAL: label = "NORMAL +-2"; break;
        case Tolerance::EASY:   label = "EASY   +-3"; break;
    }
    M5.Display.setTextSize(3);
    M5.Display.setCursor(10, 90);
    M5.Display.print(label);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200);
    M5.Display.print("A:Confirm  B:Change");
}

void draw_active(const ActiveView& v) {
    if (!s_last_valid || s_last.color != v.color) {
        M5.Display.fillScreen(color_for(v.color));
        // Legend strip.
        M5.Display.fillRect(5,  5, 14, 14, COL_BLUE);
        M5.Display.fillRect(50, 5, 14, 14, COL_GREEN);
        M5.Display.fillRect(95, 5, 14, 14, COL_RED);
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(22, 9);  M5.Display.print("LOW");
        M5.Display.setCursor(67, 9);  M5.Display.print("OK");
        M5.Display.setCursor(112, 9); M5.Display.print("HIGH");
    }

    bool counts_changed =
        !s_last_valid ||
        s_last.current_side != v.current_side ||
        s_last.strokes_A != v.strokes_A ||
        s_last.strokes_B != v.strokes_B;
    if (counts_changed) {
        M5.Display.fillRect(0, 90, 135, 100, color_for(v.color));
        uint32_t big = (v.current_side == Side::A) ? v.strokes_A : v.strokes_B;
        uint32_t sm  = (v.current_side == Side::A) ? v.strokes_B : v.strokes_A;
        char other_label = (v.current_side == Side::A) ? 'B' : 'A';
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(8);
        M5.Display.setCursor(30, 100);
        M5.Display.printf("%u", (unsigned)big);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(90, 215);
        M5.Display.printf("%c:%u", other_label, (unsigned)sm);
    }

    if (v.buzzer_flash) {
        M5.Display.fillRect(10, 150, 115, 40, COL_BLACK);
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(20, 160);
        M5.Display.print(v.buzzer_flash_on ? "BUZZER ON" : "BUZZER OFF");
    }

    s_last       = v;
    s_last_valid = true;
}

void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b, uint32_t duration_s) {
    clear();
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(30, 5); M5.Display.print("SESSION");
    M5.Display.setTextSize(2);
    M5.Display.setCursor(5, 30);  M5.Display.printf("Target:%2d", (int)target_deg);
    const char* t = (tol == Tolerance::TIGHT) ? "T1" : (tol == Tolerance::NORMAL) ? "N2" : "E3";
    M5.Display.setCursor(5, 55);  M5.Display.printf("Tol:%s", t);
    M5.Display.setCursor(5, 85);  M5.Display.printf("A:%u", (unsigned)a);
    M5.Display.setCursor(5, 110); M5.Display.printf("B:%u", (unsigned)b);
    M5.Display.setCursor(5, 140); M5.Display.printf("%02u:%02u",
                                                     (unsigned)(duration_s/60),
                                                     (unsigned)(duration_s%60));
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200); M5.Display.print("A:New  B:Sleep");
}

void draw_fault(FaultCode code) {
    clear();
    M5.Display.setTextColor(COL_RED);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(5, 40);  M5.Display.print("IMU FAULT");
    M5.Display.setTextSize(3);
    M5.Display.setCursor(35, 90); M5.Display.printf("E%02u", (unsigned)code);
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 170); M5.Display.print("Power-cycle");
    M5.Display.setCursor(10, 185); M5.Display.print("to retry");
}

void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining) {
    clear();
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(20, 10); M5.Display.print("RESUME?");
    M5.Display.setTextSize(1);
    const char* t = (tol == Tolerance::TIGHT) ? "1" : (tol == Tolerance::NORMAL) ? "2" : "3";
    M5.Display.setCursor(5, 50);  M5.Display.printf("Tgt:%2d  Tol:+-%s", (int)target_deg, t);
    M5.Display.setCursor(5, 75);  M5.Display.printf("A:%u   B:%u", (unsigned)a, (unsigned)b);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(55, 110); M5.Display.printf("%d", seconds_remaining);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200); M5.Display.print("A:Resume  B:New");
}

void draw_zero_cal_prompt(int step, bool retry) {
    auto& d = M5.Display;
    d.fillScreen(COL_BLACK);
    d.setTextColor(COL_WHITE, COL_BLACK);
    d.setTextSize(2);
    d.setCursor(8, 8);
    d.printf("ZERO CAL  %d/2", step);

    d.setCursor(8, 50);
    if (step == 1) d.print("Lay knife flat");
    else           d.print("Flip; lay flat");

    d.setCursor(8, 90);
    d.print("Press A");

    d.setCursor(8, 130);
    d.print("Hold still");

    if (retry) {
        d.setTextColor(COL_RED, COL_BLACK);
        d.setCursor(8, 180);
        d.setTextSize(3);
        d.print("HOLD STILL");
    }
}

void draw_zero_cal_progress(int remaining_ms) {
    auto& d = M5.Display;
    d.fillScreen(COL_BLACK);
    d.setTextColor(COL_WHITE, COL_BLACK);
    d.setTextSize(3);
    d.setCursor(8, 60);
    d.print("Hold still");
    d.setTextSize(4);
    d.setCursor(8, 120);
    d.printf("%d.%ds", remaining_ms / 1000, (remaining_ms % 1000) / 100);
}

void set_backlight(uint8_t percent) {
    if (percent > 100) percent = 100;
    M5.Display.setBrightness((uint8_t)(percent * 255u / 100u));
}

} // namespace ui

#else
// Native stubs for tests.
namespace ui {
    void begin() {}
    void clear() {}
    void draw_boot() {}
    void draw_bias_cal(int) {}
    void draw_set_target(float, bool, PresetSelection) {}
    void draw_set_tolerance(Tolerance) {}
    void draw_active(const ActiveView&) {}
    void draw_summary(float, Tolerance, uint32_t, uint32_t, uint32_t) {}
    void draw_fault(FaultCode) {}
    void draw_resume_prompt(float, Tolerance, uint32_t, uint32_t, int) {}
    void draw_zero_cal_prompt(int, bool) {}
    void draw_zero_cal_progress(int) {}
    void set_backlight(uint8_t) {}
}
#endif
