#include "app.h"
#include "angle.h"
#include "ui.h"
#include "feedback.h"
#include "settings.h"
#include "session.h"
#include <cmath>

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
        case State::BIAS_CAL:      ui::draw_bias_cal(10); break;
        case State::SET_TARGET:
            in_preset_mode_   = false;
            preset_selection_ = PresetSelection::P12;
            break;
        case State::SET_TOLERANCE: break;
        case State::ZERO_CAL:      break;
        case State::ACTIVE: {
            stroke_fsm_.reset();
            side_fsm_.reset();
            if (session_started_ms_ == 0) session_started_ms_ = now_ms;
            SessionState ss;
            ss.target_deg = target_deg_;
            ss.tolerance  = tol_;
            ss.g_zero_A   = g_zero_A_;
            ss.g_zero_B   = g_zero_B_;
            ss.strokes_A  = strokes_a_;
            ss.strokes_B  = strokes_b_;
            ss.current_side = side_fsm_.current_side();
            ss.session_started_ms = session_started_ms_;
            session::mark_active(ss);
            break;
        }
        case State::SUMMARY: break;
        case State::FAULT:   ui::draw_fault(fault_code_); feedback::fault_led(); break;
        case State::RESUME_PROMPT: break;
        case State::SLEEP:   break;
    }
}

void App::handle_boot(const Tick& t) {
    if (t.now_ms - state_entered_ms_ >= 2000) {
        if (settings::is_first_boot()) {
            transition(State::BIAS_CAL, t.now_ms);
        } else if (session::has_session()) {
            transition(State::RESUME_PROMPT, t.now_ms);
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
    if (t.now_ms - state_entered_ms_ >= 10000) {
        // Real bias capture ran in main.cpp before handing off to App; reload it.
        filter_.set_bias(settings::load_gyro_bias());
        settings::clear_first_boot();
        transition(State::SET_TARGET, t.now_ms);
    }
}

// STUB: Task 8 will implement the full ZERO_CAL handler.
void App::handle_zero_cal(const Tick& /*t*/) {
    // placeholder — Task 8 fills this in
}

void App::handle_set_target(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        if (in_preset_mode_) {
            if (preset_selection_ == PresetSelection::CANCEL) {
                in_preset_mode_ = false;
                last_activity_ms_ = t.now_ms;
            } else {
                target_deg_ = preset_degrees(preset_selection_);
                // Synthesize a g_ref consistent with the chosen angle.
                float r = target_deg_ * (float)M_PI / 180.0f;
                g_ref_ = {std::cos(r), 0.0f, -std::sin(r)};
                transition(State::SET_TOLERANCE, t.now_ms);
            }
        } else {
            // Live capture from filter.
            g_ref_ = filter_.gravity();
            float sinv = -g_ref_.z;
            if (sinv >  1.0f) sinv =  1.0f;
            if (sinv < -1.0f) sinv = -1.0f;
            target_deg_ = std::asin(sinv) * (180.0f / (float)M_PI);
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
}

void App::handle_set_tolerance(const Tick& t) {
    if (t.input == InputEvent::B_SHORT) {
        tol_ = next_tolerance(tol_);
        last_activity_ms_ = t.now_ms;
    } else if (t.input == InputEvent::A_SHORT) {
        settings::save_tolerance(tol_);
        transition(State::ACTIVE, t.now_ms);
    }
}

void App::handle_active(const Tick& t) {
    filter_.update(t.gyro_dps, t.accel_g);
    Vec3 g_now = filter_.gravity();

    AngleResult ar = compute_angle(g_ref_, g_now);
    // TEMP (Task 9 will fix): target_deg passed as 0.0f because g_ref_ is still
    // synthesized at the target angle, so ar.degrees is deviation from target.
    // Note: ar.degrees comes from acos() so it's always >= 0; the low boundary
    // (-tol) is structurally unreachable at this call site.
    ColorState  col = classify(ar.degrees, 0.0f, tolerance_degrees(tol_), ar.direction_sign);

    bool in_tol = (col == ColorState::GREEN);
    uint32_t before = stroke_fsm_.stroke_count();
    stroke_fsm_.update(t.now_ms, in_tol);
    if (stroke_fsm_.stroke_count() > before) {
        if (side_fsm_.current_side() == Side::A) strokes_a_++;
        else                                      strokes_b_++;
        last_stroke_ms_ = t.now_ms;
    }

    float accel_mag = std::sqrt(t.accel_g.x*t.accel_g.x + t.accel_g.y*t.accel_g.y + t.accel_g.z*t.accel_g.z);
    float grav_dot_ref = g_now.x*g_ref_.x + g_now.y*g_ref_.y + g_now.z*g_ref_.z;
    side_fsm_.update(t.now_ms, accel_mag, grav_dot_ref);
    if (side_fsm_.consume_switch()) {
        stroke_fsm_.reset();
        // Peeling and reattaching the device is unambiguous activity —
        // reset the idle timer so we don't sleep right after a side switch.
        last_activity_ms_ = t.now_ms;
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
                      buzzer_flash_showing_, buzzer_on_ };
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
