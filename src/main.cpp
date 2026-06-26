#ifndef UNIT_TEST
#include <Arduino.h>
#include <M5Unified.h>
#include <esp_sleep.h>

#include "app.h"
#include "board.h"
#include "imu.h"
#include "input.h"
#include "settings.h"
#include "session.h"
#include "ui.h"
#include "feedback.h"
#include "power.h"

static App       g_app;
static InputFSM  g_input;

static uint32_t  g_next_tick_ms       = 0;
static constexpr uint32_t TICK_PERIOD_MS = kLoopTickMs; // 50 Hz — ample for human motion (1-5 Hz)
// A single imu::read() returning false just means "no fresh IMU sample this tick"
// (MPU6886 data-ready clear at 50 Hz) — not a fault. Only a sustained run of
// failures (0.5 s) indicates a genuinely unresponsive IMU.
static constexpr uint32_t IMU_FAULT_TICKS = 25;

void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;
    M5.begin(cfg);
    setCpuFrequencyMhz(80);  // 80 MHz is ample for 50 Hz loop; saves ~25 mA

    // Safety: refuse to run if this binary was flashed onto a different board than
    // it was built for (e.g. the S3 build onto a Plus2). Each board has a distinct
    // sleep/wake + IMU path, so running the wrong one would misbehave silently.
    {
        m5::board_t expected;
        switch (board::variant()) {
            case board::Variant::PLUS:  expected = m5::board_t::board_M5StickCPlus;  break;
            case board::Variant::PLUS2: expected = m5::board_t::board_M5StickCPlus2; break;
            case board::Variant::S3:    expected = m5::board_t::board_M5StickS3;     break;
        }
        if (M5.getBoard() != expected) {
            M5.Display.setRotation(1);
            M5.Display.fillScreen(TFT_RED);
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(6, 10);
            M5.Display.print("WRONG FIRMWARE");
            M5.Display.setTextSize(1);
            M5.Display.setCursor(6, 40);
            M5.Display.print("for this device.");
            M5.Display.setCursor(6, 56);
            M5.Display.print("Flash the build that");
            M5.Display.setCursor(6, 68);
            M5.Display.print("matches your stick.");
            while (true) { delay(1000); }
        }
    }

    // Consume any pending power-key press that woke us, so it doesn't surface as a
    // BtnPWR click on the first M5.update() and bounce us straight back to sleep.
    // getKeyState() is AXP192/AXP2101-only; the Plus2/S3 power buttons are handled
    // through M5.BtnPWR, so only the AXP192 Plus needs this.
    if (board::has_axp192()) {
        M5.Power.getKeyState();
    }

    ui::begin();
    feedback::begin();
    power::begin();
    settings::begin();
    session::begin();

    // Detect wake-with-session: only route to RESUME_PROMPT if we actually
    // deep-slept AND the session-state in RTC RAM is marked active.
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool had_session_in_rtc = (cause != ESP_SLEEP_WAKEUP_UNDEFINED) && session::has_session();

    FaultCode fc = imu::begin();

    g_app.begin(had_session_in_rtc);

    // Push initial fault if IMU failed.
    if (fc != FaultCode::NONE) {
        App::Tick t{millis(), InputEvent::NONE, {0,0,-1}, {0,0,0}, fc};
        g_app.on_tick(t);
    }

    g_next_tick_ms = millis();
}

void loop() {
    uint32_t now = millis();
    if ((int32_t)(now - g_next_tick_ms) < 0) {
        delay(1);
        return;
    }
    // Catch-up clamp: if the scheduler fell more than 5 periods behind (e.g. a
    // long blocking call), resync instead of replaying ticks back-to-back —
    // burst catch-up ticks would over-integrate the Mahony filter, which
    // assumes a fixed 20 ms dt per update.
    if ((int32_t)(now - g_next_tick_ms) > (int32_t)(5 * TICK_PERIOD_MS)) {
        g_next_tick_ms = now;
    }
    g_next_tick_ms += TICK_PERIOD_MS;

    M5.update();
    // Power-key short press = "off" (deep sleep). The matching "on" is the EXT0
    // wake on GPIO35 armed in power::enter_deep_sleep(). An in-progress session
    // is in RTC RAM, so waking lands on the RESUME? prompt. The AXP192's 6 s
    // hard power-off remains available as the hardware escape hatch.
    if (M5.BtnPWR.wasClicked()) {
        power::enter_deep_sleep();
    }
    bool a_pressed = M5.BtnA.isPressed();
    bool b_pressed = M5.BtnB.isPressed();
    InputEvent ev = g_input.update(now, a_pressed, b_pressed);

    // imu::read() always populates accel/gyro with the most recent (at most one
    // tick stale) sample; its false return means "no fresh data this tick", which
    // is benign and frequent. Only fault after IMU_FAULT_TICKS consecutive misses.
    static uint32_t imu_read_fails = 0;
    Vec3 accel = {0,0,-1}, gyro = {0,0,0};
    FaultCode fault = FaultCode::NONE;
    if (!imu::read(accel, gyro)) {
        if (++imu_read_fails >= IMU_FAULT_TICKS) fault = FaultCode::E02_SELF_TEST_FAILED;
    } else {
        imu_read_fails = 0;
    }

    App::Tick tick{now, ev, accel, gyro, fault};
    g_app.on_tick(tick);

    // Sleep check uses App's real last-activity / last-stroke timestamps.
    if (power::check_idle(now, g_app.current(),
                          g_app.last_activity_ms(),
                          g_app.last_stroke_ms())) {
        power::enter_deep_sleep();
    }
    power::update_backlight(now, g_app.current(),
                            g_app.last_activity_ms(),
                            g_app.last_stroke_ms());

    // If state entered SLEEP explicitly (e.g., B in SUMMARY), trigger sleep now.
    if (g_app.current() == State::SLEEP) {
        power::enter_deep_sleep();
    }
}
#endif
