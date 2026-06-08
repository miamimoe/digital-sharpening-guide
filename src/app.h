#pragma once
#include "types.h"
#include "stroke.h"
#include "side.h"
#include "filter.h"
#include "zero_cal.h"

class App {
public:
    struct Tick {
        uint32_t    now_ms;
        InputEvent  input;
        Vec3        accel_g;
        Vec3        gyro_dps;
        FaultCode   imu_fault;          // NONE on a normal tick
    };

    void      begin(bool had_session_in_rtc_ram);
    void      on_tick(const Tick& t);
    State     current() const { return state_; }

    // Test-only and main-loop accessors
    float     target_deg()       const { return target_deg_; }
    Tolerance tolerance()        const { return tol_; }
    bool      buzzer_on()        const { return buzzer_on_; }
    uint32_t  strokes_a()        const { return strokes_a_; }
    uint32_t  strokes_b()        const { return strokes_b_; }
    Side      current_side()     const { return side_fsm_.current_side(); }
    uint32_t  last_activity_ms() const { return last_activity_ms_; }
    uint32_t  last_stroke_ms()   const { return last_stroke_ms_; }
    ZeroCalSubstate  zero_cal_substate() const { return zc_substate_; }
    Vec3             g_zero_a()          const { return g_zero_A_; }
    Vec3             g_zero_b()          const { return g_zero_B_; }

private:
    void transition(State to, uint32_t now_ms);
    void save_session_();   // snapshot current ACTIVE session into RTC RAM
    void refresh_gyro_bias_(Vec3 bias);  // apply + persist a freshly-measured gyro bias
    void handle_boot            (const Tick& t);
    void handle_bias_cal        (const Tick& t);
    void handle_zero_cal        (const Tick& t);
    void handle_set_target      (const Tick& t);
    void handle_set_tolerance   (const Tick& t);
    void handle_active          (const Tick& t);
    void handle_rezero          (const Tick& t);
    void handle_summary         (const Tick& t);
    void handle_resume_prompt   (const Tick& t);

    State            state_                = State::BOOT;
    uint32_t         state_entered_ms_     = 0;
    uint32_t         last_activity_ms_     = 0;
    uint32_t         last_stroke_ms_       = 0;

    float            target_deg_           = 17.0f;
    Tolerance        tol_                  = Tolerance::NORMAL;
    bool             buzzer_on_            = false;
    Vec3             g_zero_A_             = {0.0f, 0.0f, 0.0f};
    Vec3             g_zero_B_             = {0.0f, 0.0f, 0.0f};

    // ZERO_CAL substate machinery
    // Which side a REZERO capture is refreshing (snapshot of the active side on entry).
    Side             rezero_side_          = Side::A;

    ZeroCalSubstate  zc_substate_          = ZeroCalSubstate::PROMPT_A;
    // Which prompt substate is currently painted, so the (static) prompt screen
    // is redrawn only on change instead of full-screen every 50 Hz tick.
    ZeroCalSubstate  zc_rendered_          = ZeroCalSubstate::DONE;
    zero_cal::CaptureFSM zc_fsm_;

    // Counts down ACTIVE ticks during which snap-to-raw recovery is suppressed
    // after a snap fires (see handle_active).
    uint8_t          snap_cooldown_        = 0;

    // Last whole-second value rendered for a countdown screen (BIAS_CAL,
    // RESUME_PROMPT) so they repaint once per second instead of every tick.
    int              last_countdown_sec_   = -1;

    uint32_t         strokes_a_            = 0;
    uint32_t         strokes_b_            = 0;
    uint32_t         session_started_ms_   = 0;

    bool             in_preset_mode_       = false;
    PresetSelection  preset_selection_     = PresetSelection::P12;

    uint32_t         buzzer_flash_until_   = 0;
    bool             buzzer_flash_showing_ = false;

    // Track prior color so the buzzer beeps on the edge GREEN -> non-GREEN,
    // not every tick while out of tolerance.
    ColorState       prev_color_           = ColorState::GREEN;

    MahonyFilter filter_;
    StrokeFSM    stroke_fsm_;
    SideFSM      side_fsm_;
    FaultCode    fault_code_           = FaultCode::NONE;
};
