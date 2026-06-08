#ifndef UNIT_TEST
#include <Arduino.h>
#include <M5Unified.h>
#include <esp_sleep.h>

#include "app.h"
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

    // First-boot gyro bias capture: run BEFORE handing control to App so the
    // filter has a valid bias from the start. App::handle_bias_cal still
    // draws the 10s countdown and clears first_boot on completion.
    if (fc == FaultCode::NONE && settings::is_first_boot()) {
        ui::draw_bias_cal(10);
        Vec3 bias;
        if (imu::capture_gyro_bias(bias)) {
            settings::save_gyro_bias(bias);
            settings::clear_first_boot();
            // App::begin below will skip BIAS_CAL since first_boot is cleared.
        }
        // If capture fails (device never stilled for 60s), leave first_boot
        // set so the state machine's BIAS_CAL screen retries on this run.
    }

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
    g_next_tick_ms += TICK_PERIOD_MS;

    M5.update();
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
