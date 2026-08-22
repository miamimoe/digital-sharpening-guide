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
    bool      steady_on()        const { return steady_; }
    Vec3      gyro_bias()        const { return filter_.bias(); }
    uint32_t  strokes_a()        const { return strokes_a_; }
    uint32_t  strokes_b()        const { return strokes_b_; }
    uint8_t   green_pct()        const {
        return active_ticks_ ? (uint8_t)((green_ticks_ * 100u) / active_ticks_) : 0u;
    }
    Side      current_side()     const { return side_fsm_.current_side(); }
    uint32_t  last_activity_ms() const { return last_activity_ms_; }
    uint32_t  last_stroke_ms()   const { return last_stroke_ms_; }
    ZeroCalSubstate  zero_cal_substate() const { return zc_substate_; }
    Vec3             g_flat()            const { return g_flat_; }
    float            verify_reading_deg() const { return verify_reading_deg_; }
    Vec3             edge_axis()         const { return edge_axis_; }

private:
    void transition(State to, uint32_t now_ms);
    void save_session_();   // snapshot current ACTIVE session into RTC RAM
    void refresh_gyro_bias_(Vec3 bias);  // apply + persist a freshly-measured gyro bias
    void handle_boot            (const Tick& t);
    void handle_zero_cal        (const Tick& t);
    void handle_set_target      (const Tick& t);
    void handle_set_tolerance   (const Tick& t);
    void handle_active          (const Tick& t);
    void handle_rezero          (const Tick& t);
    void handle_verify          (const Tick& t);
    void handle_summary         (const Tick& t);
    void handle_history         (const Tick& t);
    void handle_resume_prompt   (const Tick& t);

    State            state_                = State::BOOT;
    uint32_t         state_entered_ms_     = 0;
    uint32_t         last_activity_ms_     = 0;
    uint32_t         last_stroke_ms_       = 0;

    float            target_deg_           = 17.0f;
    Tolerance        tol_                  = Tolerance::NORMAL;
    bool             buzzer_on_            = false;
    // Steady mode (beta A/B switch): accel-reference smoothing in the filter plus
    // display and colour hysteresis. Toggled by B-hold on the TOLERANCE screen.
    bool             steady_               = true;

    void apply_steady_();   // push steady_ into the filter's accel time constant
    // Nudge the gyro bias toward the measured rate whenever the device is
    // genuinely still, so thermal drift does not accumulate across a session.
    void refresh_bias_if_still_(Vec3 accel_g, Vec3 gyro_dps);
    Vec3             g_flat_               = {0.0f, 0.0f, 0.0f};  // flat-on-stone reference
    Vec3             edge_axis_            = {0.0f, 0.0f, 0.0f};  // cutting-edge / hinge axis

    // ZERO_CAL substate machinery
    ZeroCalSubstate  zc_substate_          = ZeroCalSubstate::PROMPT_FLAT;
    // Which prompt substate is currently painted, so the (static) prompt screen
    // is redrawn only on change instead of full-screen every 50 Hz tick.
    ZeroCalSubstate  zc_rendered_          = ZeroCalSubstate::DONE;
    zero_cal::CaptureFSM zc_fsm_;

    // Counts down ACTIVE ticks during which snap-to-raw recovery is suppressed
    // after a snap fires (see handle_active).
    uint8_t          snap_cooldown_        = 0;

    // VERIFY: a flat reference captured just for the accuracy check, kept
    // separate from g_flat_ so checking never disturbs a live session's zero.
    Vec3             verify_ref_           = {0.0f, 0.0f, 0.0f};
    bool             verify_captured_      = false;
    float            verify_reading_deg_   = 0.0f;   // last VERIFY readout

    // Consecutive ticks the device has looked still, for the bias refresh.
    uint16_t         bias_still_ticks_     = 0;
    // Accel direction at still-window start (fixed, never slides) and the bias
    // to revert to if the window turns out to be a slow rotation.
    Vec3             bias_anchor_          = {0.0f, 0.0f, 0.0f};
    Vec3             bias_at_window_start_ = {0.0f, 0.0f, 0.0f};

    // Last whole-second value rendered for a countdown screen (RESUME_PROMPT)
    // so it repaints once per second instead of every tick.
    int              last_countdown_sec_   = -1;

    uint32_t         strokes_a_            = 0;
    uint32_t         strokes_b_            = 0;

    // Share of ACTIVE ticks spent in tolerance. Strokes measure effort;
    // this measures technique, and it is what should improve over time.
    uint32_t         green_ticks_          = 0;
    uint32_t         active_ticks_         = 0;
    uint32_t         session_started_ms_   = 0;

    bool             in_preset_mode_       = false;
    PresetSelection  preset_selection_     = PresetSelection::P12;

    uint32_t         buzzer_flash_until_   = 0;
    bool             buzzer_flash_showing_ = false;

    // Track prior color so the buzzer beeps on the edge GREEN -> non-GREEN,
    // not every tick while out of tolerance.
    ColorState       prev_color_           = ColorState::GREEN;
    // False until ACTIVE has classified at least once, so colour hysteresis is
    // not applied against a meaningless prior state (see transition()).
    bool             color_valid_          = false;

    MahonyFilter filter_;
    StrokeFSM    stroke_fsm_;
    SideFSM      side_fsm_;
    FaultCode    fault_code_           = FaultCode::NONE;
};
