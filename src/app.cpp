#include "app.h"
#include "angle.h"
#include "ui.h"
#include "feedback.h"
#include "settings.h"
#include "session.h"
#include <cmath>

// Only ever {0,0,0} (default) or a normalized result from CaptureFSM.
static inline bool is_zero_vec(Vec3 v) {
    return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
}

static PresetSelection next_preset(PresetSelection p) {
    switch (p) {
        case PresetSelection::P12:    return PresetSelection::P15;
        case PresetSelection::P15:    return PresetSelection::P17;
        case PresetSelection::P17:    return PresetSelection::P20;
        case PresetSelection::P20:    return PresetSelection::P22;
        case PresetSelection::P22:    return PresetSelection::CANCEL;
        case PresetSelection::CANCEL: return PresetSelection::P12;
    }
    __builtin_unreachable();
}

static Tolerance next_tolerance(Tolerance t) {
    switch (t) {
        case Tolerance::TIGHT:  return Tolerance::NORMAL;
        case Tolerance::NORMAL: return Tolerance::EASY;
        case Tolerance::EASY:   return Tolerance::TIGHT;
    }
    __builtin_unreachable();
}

void App::begin(bool had_session_in_rtc_ram) {
    filter_.begin(50.0f);
    filter_.set_bias(settings::load_gyro_bias());
    buzzer_on_ = settings::load_buzzer();
    tol_       = settings::load_tolerance();

    // Wake-from-sleep path. RTC RAM only survives deep sleep (battery pull
    // clears it), so a present session implies wake — skip BOOT splash and
    // resume immediately. The defensive guard below routes incomplete sessions
    // (missing g_zero_A or g_zero_B) into ZERO_CAL instead of RESUME_PROMPT.
    if (had_session_in_rtc_ram && session::has_session()) {
        const auto& s = session::state();
        target_deg_         = s.target_deg;
        tol_                = s.tolerance;
        g_zero_A_           = s.g_zero_A;
        g_zero_B_           = s.g_zero_B;
        strokes_a_          = s.strokes_A;
        strokes_b_          = s.strokes_B;
        session_started_ms_ = s.session_started_ms;
        side_fsm_.restore_side(s.current_side);
        if (is_zero_vec(g_zero_A_) || is_zero_vec(g_zero_B_)) {
            transition(State::ZERO_CAL, 0);
            zc_substate_ = is_zero_vec(g_zero_A_) ? ZeroCalSubstate::PROMPT_A
                                                  : ZeroCalSubstate::PROMPT_B;
            return;
        }
        transition(State::RESUME_PROMPT, 0);
        return;
    }
    transition(State::BOOT, 0);
}

void App::transition(State to, uint32_t now_ms) {
    state_            = to;
    state_entered_ms_ = now_ms;
    last_activity_ms_ = now_ms;
    last_stroke_ms_   = now_ms;

    switch (to) {
        case State::BOOT:          ui::draw_boot(); break;
        case State::BIAS_CAL:      last_countdown_sec_ = 10; ui::draw_bias_cal(10); break;
        case State::SET_TARGET:
            in_preset_mode_   = false;
            preset_selection_ = PresetSelection::P12;
            ui::draw_set_target(target_deg_, in_preset_mode_, preset_selection_);
            break;
        case State::SET_TOLERANCE: ui::draw_set_tolerance(tol_); break;
        case State::ZERO_CAL:      zc_rendered_ = ZeroCalSubstate::DONE; break;
        case State::ACTIVE: {
            stroke_fsm_.reset();
            // Do NOT reset side_fsm_ here: the resume path (RESUME_PROMPT->ACTIVE)
            // must keep the side restored in begin(). Fresh sessions reset the
            // side explicitly in handle_zero_cal before transitioning here.
            if (session_started_ms_ == 0) session_started_ms_ = now_ms;
            save_session_();
            break;
        }
        case State::SUMMARY: {
            uint32_t dur_s = (session_started_ms_ != 0 && now_ms >= session_started_ms_)
                             ? (now_ms - session_started_ms_) / 1000 : 0;
            ui::draw_summary(target_deg_, tol_, strokes_a_, strokes_b_, dur_s);
            break;
        }
        case State::FAULT:   ui::draw_fault(fault_code_); feedback::fault_led(); break;
        case State::RESUME_PROMPT:
            last_countdown_sec_ = 5;
            ui::draw_resume_prompt(target_deg_, tol_, strokes_a_, strokes_b_, 5);
            break;
        case State::SLEEP:   break;
    }
}

void App::save_session_() {
    SessionState ss;
    ss.target_deg         = target_deg_;
    ss.tolerance          = tol_;
    ss.g_zero_A           = g_zero_A_;
    ss.g_zero_B           = g_zero_B_;
    ss.strokes_A          = strokes_a_;
    ss.strokes_B          = strokes_b_;
    ss.current_side       = side_fsm_.current_side();
    ss.session_started_ms = session_started_ms_;
    session::mark_active(ss);
}

void App::handle_boot(const Tick& t) {
    if (t.now_ms - state_entered_ms_ >= 2000) {
        // Resume-on-wake is decided in App::begin() (which has the real wake
        // cause). A session left in RTC RAM by a crash/soft-reset must NOT route
        // here to RESUME_PROMPT — begin() did not restore App state in that case,
        // so resuming would run ACTIVE with zeroed references. Cold boot only
        // chooses first-boot calibration vs. a fresh target.
        if (settings::is_first_boot()) {
            transition(State::BIAS_CAL, t.now_ms);
        } else {
            transition(State::SET_TARGET, t.now_ms);
        }
    }
}

void App::handle_bias_cal(const Tick& t) {
    float mag = std::sqrt(t.accel_g.x*t.accel_g.x + t.accel_g.y*t.accel_g.y + t.accel_g.z*t.accel_g.z);
    if (std::fabs(mag - 1.0f) > 0.15f) {
        state_entered_ms_ = t.now_ms; // restart countdown on motion
    }
    int remaining = 10 - (int)((t.now_ms - state_entered_ms_) / 1000);
    if (remaining < 0) remaining = 0;
    if (remaining != last_countdown_sec_) {
        last_countdown_sec_ = remaining;
        ui::draw_bias_cal(remaining);
    }
    if (t.now_ms - state_entered_ms_ >= 10000) {
        // Real bias capture ran in main.cpp before handing off to App; reload it.
        filter_.set_bias(settings::load_gyro_bias());
        settings::clear_first_boot();
        transition(State::SET_TARGET, t.now_ms);
    }
}

void App::handle_zero_cal(const Tick& t) {
    InputEvent input = t.input;

    // Long-press A aborts back to SET_TARGET (consistent with other screens).
    if (input == InputEvent::A_LONG) {
        transition(State::SET_TARGET, t.now_ms);
        return;
    }

    auto on_capture_done = [&](Vec3& dest, ZeroCalSubstate next) {
        dest         = zc_fsm_.result();
        zc_substate_ = next;
    };

    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_A:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_substate_ = ZeroCalSubstate::CAPTURE_A;
            }
            break;

        case ZeroCalSubstate::CAPTURE_A:
            zc_fsm_.update(t.accel_g, t.gyro_dps);
            if (zc_fsm_.done()) {
                on_capture_done(g_zero_A_, ZeroCalSubstate::PROMPT_B);
            }
            break;

        case ZeroCalSubstate::PROMPT_B:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_substate_ = ZeroCalSubstate::CAPTURE_B;
            }
            break;

        case ZeroCalSubstate::CAPTURE_B:
            zc_fsm_.update(t.accel_g, t.gyro_dps);
            if (zc_fsm_.done()) {
                on_capture_done(g_zero_B_, ZeroCalSubstate::DONE);
                // transition(ACTIVE) builds SessionState and calls mark_active itself.
                session_started_ms_ = t.now_ms;
                side_fsm_.reset();   // fresh session begins on side A
                transition(State::ACTIVE, t.now_ms);
            }
            break;

        case ZeroCalSubstate::DONE:
            // Should not be reached — DONE triggers the transition above.
            break;
    }

    // Single smooth countdown across warmup+averaging. During warmup,
    // averaging_remaining() is 0 but the whole averaging window is still ahead,
    // so add it explicitly to avoid the timer jumping back up at phase change.
    int ticks_remaining = zc_fsm_.warmup_remaining() + zc_fsm_.averaging_remaining();
    if (zc_fsm_.phase() == zero_cal::Phase::WARMUP) {
        ticks_remaining += zero_cal::AVERAGING_TICKS;
    }
    int total_capture_ms_remaining = ticks_remaining * (int)kLoopTickMs;

    bool retry = false;  // v1: no retry cue. Add later if hardware testing shows users miss the signal.

    // Prompt screens are static — redraw only on substate change (avoid a 50 Hz
    // full-screen fillScreen flicker). Progress self-throttles in ui.cpp.
    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_A:
            if (zc_rendered_ != zc_substate_) ui::draw_zero_cal_prompt(1, retry);
            break;
        case ZeroCalSubstate::CAPTURE_A: ui::draw_zero_cal_progress(total_capture_ms_remaining); break;
        case ZeroCalSubstate::PROMPT_B:
            if (zc_rendered_ != zc_substate_) ui::draw_zero_cal_prompt(2, retry);
            break;
        case ZeroCalSubstate::CAPTURE_B: ui::draw_zero_cal_progress(total_capture_ms_remaining); break;
        case ZeroCalSubstate::DONE:      break;
    }
    zc_rendered_ = zc_substate_;
}

void App::handle_set_target(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        if (in_preset_mode_) {
            if (preset_selection_ == PresetSelection::CANCEL) {
                in_preset_mode_ = false;
                last_activity_ms_ = t.now_ms;
            } else {
                target_deg_ = preset_degrees(preset_selection_);
                transition(State::SET_TOLERANCE, t.now_ms);
            }
        } else {
            // Freehand path removed (world-horizontal assumption gone).
            // target_deg_ keeps its current value (default 17.0f or last preset).
            transition(State::SET_TOLERANCE, t.now_ms);
        }
    } else if (t.input == InputEvent::B_SHORT) {
        if (!in_preset_mode_) {
            in_preset_mode_   = true;
            preset_selection_ = PresetSelection::P12;
        } else {
            preset_selection_ = next_preset(preset_selection_);
        }
        last_activity_ms_ = t.now_ms;
    }
    // Repaint if the input changed what's shown but we stayed on this screen
    // (entered/cycled presets, or CANCEL returned to live view).
    if (state_ == State::SET_TARGET && t.input != InputEvent::NONE) {
        ui::draw_set_target(target_deg_, in_preset_mode_, preset_selection_);
    }
}

void App::handle_set_tolerance(const Tick& t) {
    if (t.input == InputEvent::B_SHORT) {
        tol_ = next_tolerance(tol_);
        last_activity_ms_ = t.now_ms;
        ui::draw_set_tolerance(tol_);
    } else if (t.input == InputEvent::A_SHORT) {
        settings::save_tolerance(tol_);
        // confirm tolerance, persist, advance into ZERO_CAL.
        zc_substate_ = ZeroCalSubstate::PROMPT_A;
        transition(State::ZERO_CAL, t.now_ms);
    }
}

void App::handle_active(const Tick& t) {
    filter_.update(t.gyro_dps, t.accel_g);
    // Snap-to-raw recovery: when the device is verified still but the filter's
    // gravity estimate still lags (e.g. just after a side flip, or on the first
    // ACTIVE tick before Mahony has converged), re-anchor to the raw accel so the
    // color/angle is correct immediately instead of after ~2 s of convergence.
    // Cooldown prevents re-snapping every tick if the filter ever settles with a
    // residual above SNAP_DIVERGENCE_DEG (possible only if kp is tuned very low).
    if (snap_cooldown_ > 0) {
        --snap_cooldown_;
    } else if (mahony::should_snap(filter_.gravity(), t.accel_g, t.gyro_dps)) {
        filter_.nudge_to_gravity(t.accel_g);
        snap_cooldown_ = mahony::SNAP_COOLDOWN_TICKS;
    }
    Vec3 g_now = filter_.gravity();

    Vec3 g_zero_active = (side_fsm_.current_side() == Side::A) ? g_zero_A_ : g_zero_B_;
    AngleResult ar = compute_angle(g_zero_active, g_now);
    ColorState  col = classify(ar.degrees, target_deg_, tolerance_degrees(tol_));

    bool in_tol = (col == ColorState::GREEN);
    uint32_t before = stroke_fsm_.stroke_count();
    stroke_fsm_.update(t.now_ms, in_tol);
    if (stroke_fsm_.stroke_count() > before) {
        if (side_fsm_.current_side() == Side::A) strokes_a_++;
        else                                      strokes_b_++;
        last_stroke_ms_ = t.now_ms;
        save_session_();   // keep RTC RAM current so idle-sleep preserves counts
    }

    float accel_mag = std::sqrt(t.accel_g.x*t.accel_g.x + t.accel_g.y*t.accel_g.y + t.accel_g.z*t.accel_g.z);
    float gyro_mag  = std::sqrt(t.gyro_dps.x*t.gyro_dps.x + t.gyro_dps.y*t.gyro_dps.y + t.gyro_dps.z*t.gyro_dps.z);
    float grav_dot_ref = g_now.x*g_zero_A_.x + g_now.y*g_zero_A_.y + g_now.z*g_zero_A_.z;
    side_fsm_.update(t.now_ms, accel_mag, gyro_mag, grav_dot_ref);
    if (side_fsm_.consume_switch()) {
        stroke_fsm_.reset();
        // Peeling and reattaching the device is unambiguous activity —
        // reset the idle timer so we don't sleep right after a side switch.
        last_activity_ms_ = t.now_ms;
        save_session_();   // persist the new active side to RTC RAM
    }

    if (t.input == InputEvent::A_LONG) {
        transition(State::SUMMARY, t.now_ms);
        return;
    }
    if (t.input == InputEvent::B_SHORT) {
        side_fsm_.manual_toggle(t.now_ms);
        side_fsm_.consume_switch();
        stroke_fsm_.reset();
        last_activity_ms_ = t.now_ms;
        save_session_();   // persist the manually-toggled side to RTC RAM
    }
    if (t.input == InputEvent::B_LONG) {
        buzzer_on_ = !buzzer_on_;
        settings::save_buzzer(buzzer_on_);
        buzzer_flash_until_   = t.now_ms + 800;
        buzzer_flash_showing_ = true;
        last_activity_ms_     = t.now_ms;
    } else if (buzzer_flash_showing_ && t.now_ms > buzzer_flash_until_) {
        buzzer_flash_showing_ = false;
    }

    feedback::set_color(col);
    // Beep only on the edge GREEN -> non-GREEN, not every tick.
    if (buzzer_on_ && col != ColorState::GREEN && prev_color_ == ColorState::GREEN) {
        feedback::beep_out_of_tolerance();
    }
    prev_color_ = col;

    ui::ActiveView v{ col,
                      side_fsm_.current_side(),
                      strokes_a_, strokes_b_,
                      buzzer_flash_showing_, buzzer_on_,
                      ar.degrees };
    ui::draw_active(v);
}

void App::handle_summary(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        strokes_a_ = strokes_b_ = 0;
        session_started_ms_ = 0;
        session::clear();
        transition(State::SET_TARGET, t.now_ms);
    } else if (t.input == InputEvent::B_SHORT) {
        transition(State::SLEEP, t.now_ms);
    }
}

void App::handle_resume_prompt(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        transition(State::ACTIVE, t.now_ms);
    } else if (t.input == InputEvent::B_SHORT) {
        session::clear();
        strokes_a_ = strokes_b_ = 0;
        session_started_ms_ = 0;
        transition(State::SET_TARGET, t.now_ms);
    } else if (t.now_ms - state_entered_ms_ >= 5000) {
        session::clear();
        strokes_a_ = strokes_b_ = 0;
        session_started_ms_ = 0;
        transition(State::SET_TARGET, t.now_ms);
    }
    if (state_ == State::RESUME_PROMPT) {
        int remaining = 5 - (int)((t.now_ms - state_entered_ms_) / 1000);
        if (remaining < 0) remaining = 0;
        if (remaining != last_countdown_sec_) {
            last_countdown_sec_ = remaining;
            ui::draw_resume_prompt(target_deg_, tol_, strokes_a_, strokes_b_, remaining);
        }
    }
}

void App::on_tick(const Tick& t) {
    if (t.imu_fault != FaultCode::NONE && state_ != State::FAULT) {
        fault_code_ = t.imu_fault;
        transition(State::FAULT, t.now_ms);
        return;
    }

    switch (state_) {
        case State::BOOT:          handle_boot(t); break;
        case State::BIAS_CAL:      handle_bias_cal(t); break;
        case State::ZERO_CAL:      handle_zero_cal(t); break;
        case State::SET_TARGET:    handle_set_target(t); break;
        case State::SET_TOLERANCE: handle_set_tolerance(t); break;
        case State::ACTIVE:        handle_active(t); break;
        case State::SUMMARY:       handle_summary(t); break;
        case State::RESUME_PROMPT: handle_resume_prompt(t); break;
        case State::FAULT:         break;
        case State::SLEEP:         break;
    }
}
