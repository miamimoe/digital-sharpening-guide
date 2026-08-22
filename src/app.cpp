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

// Unit vector, or {0,0,0} for a degenerate (near-zero) input.
static inline Vec3 normalized(Vec3 v) {
    float m = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (m < 1e-3f) return {0.0f, 0.0f, 0.0f};
    return {v.x/m, v.y/m, v.z/m};
}

static PresetSelection next_preset(PresetSelection p) {
    switch (p) {
        case PresetSelection::P12:    return PresetSelection::P15;
        case PresetSelection::P15:    return PresetSelection::P17;
        case PresetSelection::P17:    return PresetSelection::P20;
        case PresetSelection::P20:    return PresetSelection::P22;
        case PresetSelection::P22:    return PresetSelection::P25;
        case PresetSelection::P25:    return PresetSelection::P28;
        case PresetSelection::P28:    return PresetSelection::CANCEL;
        case PresetSelection::CANCEL: return PresetSelection::P12;
    }
    __builtin_unreachable();
}

void App::apply_steady_() {
    filter_.set_accel_tau(steady_ ? mahony::ACCEL_LP_TAU_S : 0.0f);
}

void App::refresh_bias_if_still_(Vec3 accel_g, Vec3 gyro_dps) {
    const float gyro_mag = std::sqrt(gyro_dps.x*gyro_dps.x +
                                     gyro_dps.y*gyro_dps.y +
                                     gyro_dps.z*gyro_dps.z);
    const float a_mag    = std::sqrt(accel_g.x*accel_g.x +
                                     accel_g.y*accel_g.y +
                                     accel_g.z*accel_g.z);

    // Both conditions must hold: near-zero rate AND pure gravity. Rate alone is
    // not enough — a slow steady rotation also reads low, and absorbing that as
    // bias would make the filter permanently wrong in the direction of the turn.
    const bool looks_still = gyro_mag < mahony::BIAS_STILL_GYRO_DPS
                          && std::fabs(a_mag - 1.0f) < mahony::BIAS_STILL_ACCEL_TOL;
    if (!looks_still) {
        bias_still_ticks_ = 0;
        return;
    }

    // Rate and magnitude alone cannot catch a rotation slower than the gyro
    // threshold — a sub-2 dps tilt keeps |a| at 1 g and would be learned as
    // bias. So the accel direction is ANCHORED at window start and never slides:
    // a genuine rest stays within noise of the anchor forever, while any slow
    // tilt walks away from it. When that trips, the bias is REVERTED to its
    // value at window start, so whatever the rotation taught before tripping is
    // taken back. (A yaw rotation is invisible to the accel by nature — but yaw
    // does not move gravity, and the bevel is computed from gravity alone.)
    bool in_learn_zone = true;
    if (bias_still_ticks_ == 0) {
        bias_anchor_          = accel_g;
        bias_at_window_start_ = filter_.bias();
    } else {
        const float dx = accel_g.x - bias_anchor_.x;
        const float dy = accel_g.y - bias_anchor_.y;
        const float dz = accel_g.z - bias_anchor_.z;
        const float drift = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (drift > mahony::BIAS_STILL_DRIFT_TOL_G) {
            filter_.set_bias(bias_at_window_start_);   // that was rotation, not rest
            bias_still_ticks_ = 0;
            return;
        }
        // Middle zone: still inside the trip threshold but visibly walking away
        // from the anchor. Could be the front edge of a slow rotation — hold
        // (keep the window, learn nothing) until it either trips or settles.
        in_learn_zone = drift < mahony::BIAS_STILL_DRIFT_TOL_G * 0.5f;
    }
    if (bias_still_ticks_ < 0xFFFF) ++bias_still_ticks_;
    if (bias_still_ticks_ <= mahony::BIAS_STILL_TICKS) {
        return;                      // not yet convinced it is actually at rest
    }
    if (!in_learn_zone) return;

    // Commit point: during a long genuine rest, periodically accept the learned
    // bias as the new revert baseline, so one noise spike tripping the anchor
    // cannot throw away minutes of legitimate convergence.
    if ((bias_still_ticks_ & 0x7F) == 0) bias_at_window_start_ = filter_.bias();

    // Held still long enough: whatever the gyro still reads is bias, not motion.
    Vec3 b = filter_.bias();
    b.x += mahony::BIAS_EMA_ALPHA * (gyro_dps.x - b.x);
    b.y += mahony::BIAS_EMA_ALPHA * (gyro_dps.y - b.y);
    b.z += mahony::BIAS_EMA_ALPHA * (gyro_dps.z - b.z);
    filter_.set_bias(b);
    // Deliberately NOT persisted to NVS here. A refresh is only as good as the
    // stillness detection that produced it; writing every session would let one
    // bad estimate outlive the session that made it. Zero-cal remains the only
    // thing that persists a bias.
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
    // Mahony gains. With per-session gyro-bias refresh (zero-cal) and online bias
    // correction (ki>0) doing the bias work, kp can be modest — high kp would only
    // pull the estimate toward dynamic stroke acceleration. kp=0.8, ki=0.02 is the
    // bring-up starting point (validate against a real stroke).
    filter_.begin(50.0f, 0.8f, 0.02f);
    filter_.set_bias(settings::load_gyro_bias());
    buzzer_on_ = settings::load_buzzer();
    tol_       = settings::load_tolerance();
    steady_    = settings::load_steady();
    apply_steady_();

    // Wake-from-sleep path. RTC RAM only survives deep sleep (battery pull
    // clears it), so a present session implies wake — skip BOOT splash and
    // resume immediately. The defensive guard below routes an incomplete session
    // (no flat reference captured) into ZERO_CAL instead of RESUME_PROMPT.
    if (had_session_in_rtc_ram && session::has_session()) {
        const auto& s = session::state();
        target_deg_         = s.target_deg;
        tol_                = s.tolerance;
        g_flat_             = s.g_flat;
        edge_axis_          = s.edge_axis;
        strokes_a_          = s.strokes_A;
        strokes_b_          = s.strokes_B;
        active_ticks_       = s.active_ticks;
        green_ticks_        = s.green_ticks;
        session_started_ms_ = s.session_started_ms;
        side_fsm_.restore_side(s.current_side);
        if (is_zero_vec(g_flat_)) {
            transition(State::ZERO_CAL, 0);
            zc_substate_ = ZeroCalSubstate::PROMPT_FLAT;
            return;
        }
        transition(State::RESUME_PROMPT, 0);
        return;
    }
    transition(State::BOOT, 0);
}

void App::transition(State to, uint32_t now_ms) {
    const State from  = state_;
    state_            = to;
    state_entered_ms_ = now_ms;
    last_activity_ms_ = now_ms;
    last_stroke_ms_   = now_ms;

    // prev_color_ carries no meaning until ACTIVE has classified once. Without
    // this, the first classification would inherit the initial GREEN and get the
    // widened (sticky) green band — reading in-tolerance while genuinely outside
    // it, and staying there. The first classification must be unbiased.
    if (to == State::ACTIVE) color_valid_ = false;

    switch (to) {
        case State::BOOT:          ui::draw_boot(); break;
        case State::SET_TARGET:
            in_preset_mode_   = false;
            preset_selection_ = PresetSelection::P12;
            ui::draw_set_target(target_deg_, in_preset_mode_, preset_selection_);
            break;
        case State::SET_TOLERANCE: ui::draw_set_tolerance(tol_, steady_); break;
        case State::ZERO_CAL:
            // Invalidate the ACTIVE dirty-region cache here rather than relying on
            // every predecessor screen having cleared it on its way out.
            ui::clear();
            zc_rendered_ = ZeroCalSubstate::DONE;
            break;
        case State::REZERO:
            // Repaint from scratch: clearing also invalidates the zero-cal
            // progress throttle cache, so entering REZERO never shows a stale
            // ACTIVE frame behind the progress screen.
            ui::clear();
            zc_fsm_.start();
            break;
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
            // Persist the finished session exactly once, on the ACTIVE -> SUMMARY
            // edge — not per stroke (NVS wear), and NOT on re-entry from HISTORY,
            // which would prepend a duplicate record per history visit. Skip
            // sessions with no ACTIVE time; they carry nothing worth keeping.
            if (from == State::ACTIVE && active_ticks_ > 0) {
                SessionRecord r;
                r.target_deg_x10 = (uint16_t)(target_deg_ * 10.0f + 0.5f);
                r.tolerance      = (uint8_t)tol_;
                r.green_pct      = green_pct();
                r.strokes_a      = (uint16_t)strokes_a_;
                r.strokes_b      = (uint16_t)strokes_b_;
                r.duration_s     = (uint16_t)((dur_s > 65535u) ? 65535u : dur_s);
                settings::push_session_record(r);
            }
            ui::draw_summary(target_deg_, tol_, strokes_a_, strokes_b_, dur_s, green_pct());
            break;
        }
        case State::VERIFY:
            ui::clear();
            verify_captured_ = false;
            zc_fsm_ = zero_cal::CaptureFSM{};   // IDLE until the user presses A
            ui::draw_verify_prompt();
            break;
        case State::HISTORY: {
            SessionRecord recs[kSessionHistoryMax];
            int n = settings::load_session_history(recs, kSessionHistoryMax);
            ui::draw_history(recs, n);
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

void App::refresh_gyro_bias_(Vec3 bias) {
    // Per-session gyro-bias refresh from the zero-cal still window. Tracks
    // turn-on/thermal drift a once-ever capture couldn't, so the Mahony filter
    // starts each session with an accurate bias.
    filter_.set_bias(bias);
    settings::save_gyro_bias(bias);
}

void App::save_session_() {
    SessionState ss;
    ss.target_deg         = target_deg_;
    ss.tolerance          = tol_;
    ss.g_flat             = g_flat_;
    ss.edge_axis          = edge_axis_;
    ss.strokes_A          = strokes_a_;
    ss.strokes_B          = strokes_b_;
    ss.active_ticks       = active_ticks_;
    ss.green_ticks        = green_ticks_;
    ss.current_side       = side_fsm_.current_side();
    ss.session_started_ms = session_started_ms_;
    session::mark_active(ss);
}

void App::handle_boot(const Tick& t) {
    if (t.now_ms - state_entered_ms_ >= 2000) {
        // Resume-on-wake is decided in App::begin() (which has the real wake
        // cause). A session left in RTC RAM by a crash/soft-reset must NOT route
        // here to RESUME_PROMPT — begin() did not restore App state in that case,
        // so resuming would run ACTIVE with zeroed references.
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

    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_FLAT:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_substate_ = ZeroCalSubstate::CAPTURE_FLAT;
            }
            break;

        case ZeroCalSubstate::CAPTURE_FLAT: {
            // B = force-capture escape hatch (stillness gate can't pass).
            bool done = false;
            if (input == InputEvent::B_SHORT) {
                Vec3 forced = normalized(t.accel_g);
                if (!is_zero_vec(forced)) { g_flat_ = forced; done = true; }
            } else {
                zc_fsm_.update(t.accel_g, t.gyro_dps);
                if (zc_fsm_.done()) {
                    g_flat_ = zc_fsm_.result();
                    refresh_gyro_bias_(zc_fsm_.gyro_bias());
                    done = true;
                }
            }
            if (done) zc_substate_ = ZeroCalSubstate::PROMPT_RAISE;
            break;
        }

        case ZeroCalSubstate::PROMPT_RAISE:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_substate_ = ZeroCalSubstate::CAPTURE_RAISE;
            }
            break;

        case ZeroCalSubstate::CAPTURE_RAISE: {
            bool done = false;
            Vec3 raised = {0.0f, 0.0f, 0.0f};
            if (input == InputEvent::B_SHORT) {
                raised = normalized(t.accel_g);
                if (!is_zero_vec(raised)) done = true;
            } else {
                zc_fsm_.update(t.accel_g, t.gyro_dps);
                if (zc_fsm_.done()) { raised = zc_fsm_.result(); done = true; }
            }
            if (done) {
                // {0,0,0} if the raise was too small — bevel_angle then falls back
                // to the total-tilt method, so this degrades gracefully.
                edge_axis_ = compute_edge_axis(g_flat_, raised);
                zc_substate_ = ZeroCalSubstate::DONE;
                session_started_ms_ = t.now_ms;
                side_fsm_.reset();   // fresh session begins on side A
                transition(State::ACTIVE, t.now_ms);
            }
            break;
        }

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

    // Currently-moving cue for the progress screen, so a stalled countdown reads
    // as "you're moving it" rather than a frozen device.
    bool moving = zc_fsm_.moving();

    // Keep the idle clock honest so an in-progress capture never dims/sleeps:
    // button input, device handling (moving), or an actively-progressing capture
    // all count as activity. Only an untouched static prompt screen idles out.
    zero_cal::Phase ph = zc_fsm_.phase();
    if (input == InputEvent::A_SHORT || input == InputEvent::B_SHORT
        || moving
        || ph == zero_cal::Phase::WARMUP || ph == zero_cal::Phase::AVERAGING) {
        last_activity_ms_ = t.now_ms;
    }

    bool retry = false;  // v1: no retry cue. Add later if hardware testing shows users miss the signal.

    // Prompt screens are static — redraw only on substate change (avoid a 50 Hz
    // full-screen fillScreen flicker). Progress self-throttles in ui.cpp.
    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_FLAT:
            if (zc_rendered_ != zc_substate_) ui::draw_zero_cal_prompt(1, retry);
            break;
        case ZeroCalSubstate::CAPTURE_FLAT:  ui::draw_zero_cal_progress(total_capture_ms_remaining, moving); break;
        case ZeroCalSubstate::PROMPT_RAISE:
            if (zc_rendered_ != zc_substate_) ui::draw_zero_cal_prompt(2, retry);
            break;
        case ZeroCalSubstate::CAPTURE_RAISE: ui::draw_zero_cal_progress(total_capture_ms_remaining, moving); break;
        case ZeroCalSubstate::DONE:          break;
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
    } else if (t.input == InputEvent::A_LONG) {
        // Accuracy check — "is this thing actually right?". Lives here because it
        // is a pre-session question, and it is a long-press so nobody lands in it
        // by accident on the way to sharpening.
        transition(State::VERIFY, t.now_ms);
        return;
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
        ui::draw_set_tolerance(tol_, steady_);
    } else if (t.input == InputEvent::B_LONG) {
        // Beta A/B switch: flip steady mode without reflashing, so one device can
        // be compared against itself in the same session.
        steady_ = !steady_;
        settings::save_steady(steady_);
        apply_steady_();
        last_activity_ms_ = t.now_ms;
        ui::draw_set_tolerance(tol_, steady_);
    } else if (t.input == InputEvent::A_SHORT) {
        settings::save_tolerance(tol_);
        // confirm tolerance, persist, advance into ZERO_CAL.
        zc_substate_ = ZeroCalSubstate::PROMPT_FLAT;
        transition(State::ZERO_CAL, t.now_ms);
    }
}

void App::handle_active(const Tick& t) {
    filter_.update(t.gyro_dps, t.accel_g);
    refresh_bias_if_still_(t.accel_g, t.gyro_dps);
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

    // Skew-corrected bevel about the captured edge axis. One reference (g_flat_,
    // edge_axis_) serves both blade faces — the flipped face is folded internally.
    float bevel = bevel_angle(g_flat_, edge_axis_, g_now);
    ColorState col = classify(bevel, target_deg_, tolerance_degrees(tol_),
                              prev_color_,
                              (steady_ && color_valid_) ? CLASSIFY_HYSTERESIS_DEG : 0.0f);
    color_valid_ = true;

    ++active_ticks_;
    if (col == ColorState::GREEN) ++green_ticks_;

    // Horizontal linear acceleration = the stroke motion (gravity removed, then
    // the component in the stone plane). g_now is a unit vector, so accel - g_now
    // is the linear part in g; project out the vertical to isolate the sweep.
    Vec3 la = { t.accel_g.x - g_now.x, t.accel_g.y - g_now.y, t.accel_g.z - g_now.z };
    float la_v = la.x*g_now.x + la.y*g_now.y + la.z*g_now.z;
    Vec3 la_h = { la.x - la_v*g_now.x, la.y - la_v*g_now.y, la.z - la_v*g_now.z };
    float lat = std::sqrt(la_h.x*la_h.x + la_h.y*la_h.y + la_h.z*la_h.z);

    // Sustained stroke motion counts as activity even when no stroke is counted
    // (e.g. the user never reaches green), so the device can't deep-sleep mid-use.
    if (lat >= StrokeFSM::PEAK_LOW_G) last_activity_ms_ = t.now_ms;

    bool in_tol = (col == ColorState::GREEN);
    uint32_t before = stroke_fsm_.stroke_count();
    stroke_fsm_.update(t.now_ms, in_tol, lat);
    if (stroke_fsm_.stroke_count() > before) {
        if (side_fsm_.current_side() == Side::A) strokes_a_++;
        else                                      strokes_b_++;
        last_stroke_ms_ = t.now_ms;
        save_session_();   // keep RTC RAM current so idle-sleep preserves counts
    }

    // Automatic gravity-polarity side detection is intentionally NOT run here.
    // In real use the device sits screen-up on BOTH blade faces (you flip the
    // knife, not the device), so gravity does not reverse between sides and the
    // polarity signal can't distinguish them — and worse, it would override the
    // user's manual choice. Side is controlled manually by B short-press below,
    // which is authoritative and sticks. (SideFSM::update remains available and
    // unit-tested for a future mount where the polarity does flip.)

    if (t.input == InputEvent::A_LONG) {
        transition(State::SUMMARY, t.now_ms);
        return;
    }
    if (t.input == InputEvent::A_SHORT) {
        // Re-capture the current side's zero in place (e.g. after re-mounting or
        // a bad side-B capture), then return to ACTIVE with the fresh reference.
        transition(State::REZERO, t.now_ms);
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
        // Audible confirmation when enabling — also a self-test that the buzzer
        // works (silence on disable confirms "off").
        if (buzzer_on_) feedback::beep_confirm();
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
                      bevel, steady_ };
    ui::draw_active(v);
}

void App::handle_rezero(const Tick& t) {
    // Abort (B short or long-press A) returns to ACTIVE leaving the zero unchanged.
    if (t.input == InputEvent::B_SHORT || t.input == InputEvent::A_LONG) {
        ui::clear();   // invalidate the ACTIVE dirty-region cache for a clean repaint
        transition(State::ACTIVE, t.now_ms);
        return;
    }
    zc_fsm_.update(t.accel_g, t.gyro_dps);
    if (zc_fsm_.done()) {
        g_flat_ = zc_fsm_.result();   // refresh the flat reference in place
        refresh_gyro_bias_(zc_fsm_.gyro_bias());
        ui::clear();
        transition(State::ACTIVE, t.now_ms);
        return;
    }
    int ticks_remaining = zc_fsm_.warmup_remaining() + zc_fsm_.averaging_remaining();
    if (zc_fsm_.phase() == zero_cal::Phase::WARMUP) ticks_remaining += zero_cal::AVERAGING_TICKS;
    ui::draw_zero_cal_progress(ticks_remaining * (int)kLoopTickMs, zc_fsm_.moving());

    // Keep the idle clock honest: an in-progress capture (or a user handling the
    // device) must never dim/sleep out from under the re-zero.
    zero_cal::Phase ph = zc_fsm_.phase();
    if (zc_fsm_.moving()
        || ph == zero_cal::Phase::WARMUP || ph == zero_cal::Phase::AVERAGING) {
        last_activity_ms_ = t.now_ms;
    }
}

void App::handle_summary(const Tick& t) {
    if (t.input == InputEvent::B_LONG) {
        transition(State::HISTORY, t.now_ms);
        return;
    }
    if (t.input == InputEvent::A_SHORT) {
        strokes_a_ = strokes_b_ = 0;
        green_ticks_ = active_ticks_ = 0;
        session_started_ms_ = 0;
        session::clear();
        transition(State::SET_TARGET, t.now_ms);
    } else if (t.input == InputEvent::B_SHORT) {
        // B:Sleep ends the session for real — the user already closed it via
        // A-long -> SUMMARY, so waking back into RESUME? would be wrong.
        session::clear();
        strokes_a_ = strokes_b_ = 0;
        green_ticks_ = active_ticks_ = 0;
        session_started_ms_ = 0;
        transition(State::SLEEP, t.now_ms);
    }
}

void App::handle_verify(const Tick& t) {
    // Accuracy check. Capture a flat reference, then read the live angle at 0.1
    // deg against it, so the user can lay the device on a known angle (a printed
    // wedge, an angle block) and see whether the number matches.
    //
    // This never touches g_flat_ or the session — checking accuracy must not cost
    // you the zero you already set.
    if (t.input == InputEvent::B_SHORT || t.input == InputEvent::A_LONG) {
        ui::clear();
        transition(State::SET_TARGET, t.now_ms);
        return;
    }

    if (!verify_captured_) {
        if (zc_fsm_.phase() == zero_cal::Phase::IDLE) {
            if (t.input == InputEvent::A_SHORT) zc_fsm_.start();
            else { ui::draw_verify_prompt(); return; }
        }
        zc_fsm_.update(t.accel_g, t.gyro_dps);
        if (zc_fsm_.done()) {
            verify_ref_      = normalized(zc_fsm_.result());
            verify_captured_ = true;
            filter_.nudge_to_gravity(t.accel_g);   // start from a known-good pose
            ui::clear();
        } else {
            // Same total as handle_zero_cal: during WARMUP, averaging_remaining()
            // is still 0, so add the averaging window or the countdown would jump
            // UP when averaging starts.
            int rem_ticks = zc_fsm_.warmup_remaining() + zc_fsm_.averaging_remaining();
            if (zc_fsm_.phase() == zero_cal::Phase::WARMUP) rem_ticks += zero_cal::AVERAGING_TICKS;
            const int remaining = rem_ticks * (int)kLoopTickMs;
            ui::draw_verify_capture(remaining, zc_fsm_.moving());
        }
        return;
    }

    filter_.update(t.gyro_dps, t.accel_g);
    if (snap_cooldown_ > 0) --snap_cooldown_;
    else if (mahony::should_snap(filter_.gravity(), t.accel_g, t.gyro_dps)) {
        filter_.nudge_to_gravity(t.accel_g);
        snap_cooldown_ = mahony::SNAP_COOLDOWN_TICKS;
    }
    // No edge axis here: this is a total-tilt check against the flat reference,
    // which is what a wedge or angle block presents. Passing a zero axis makes
    // bevel_angle fall back to exactly that.
    verify_reading_deg_ = bevel_angle(verify_ref_, {0.0f, 0.0f, 0.0f}, filter_.gravity());
    if (t.input != InputEvent::NONE) last_activity_ms_ = t.now_ms;
    ui::draw_verify_reading(verify_reading_deg_);
}

void App::handle_history(const Tick& t) {
    // Any press returns. A read-only screen should not need a legend to leave.
    if (t.input != InputEvent::NONE) transition(State::SUMMARY, t.now_ms);
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
        case State::ZERO_CAL:      handle_zero_cal(t); break;
        case State::SET_TARGET:    handle_set_target(t); break;
        case State::SET_TOLERANCE: handle_set_tolerance(t); break;
        case State::ACTIVE:        handle_active(t); break;
        case State::REZERO:        handle_rezero(t); break;
        case State::VERIFY:        handle_verify(t); break;
        case State::SUMMARY:       handle_summary(t); break;
        case State::HISTORY:       handle_history(t); break;
        case State::RESUME_PROMPT: handle_resume_prompt(t); break;
        case State::FAULT:         break;
        case State::SLEEP:         break;
    }
}
