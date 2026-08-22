#include <unity.h>
#include <cmath>
#include "app.h"
#include "angle.h"
#include "settings.h"
#include "session.h"

// On-target pose: 17 deg about the edge axis (body +x) from flat {0,0,-1}.
// = {0, sin17, -cos17}. With edge_axis={1,0,0} this reads a 17 deg bevel.
static const Vec3 g_now_at_target_17 = {0.0f, 0.2924f, -0.9563f};

static void advance(App& a, uint32_t& t, uint32_t dt_ms, InputEvent ev = InputEvent::NONE,
                    Vec3 accel = {0,0,-1}, Vec3 gyro = {0,0,0})
{
    uint32_t end = t + dt_ms;
    bool emitted = false;
    while (t < end) {
        t += 10;
        App::Tick tick{t, (emitted ? InputEvent::NONE : ev), accel, gyro, FaultCode::NONE};
        emitted = true;
        a.on_tick(tick);
    }
}

// ~1.6s of still ticks to complete one capture (500ms warmup + 1000ms avg + slack).
static void drive_still(App& a, uint32_t& t, Vec3 still_accel = {0.0f, 0.0f, -1.0f}) {
    advance(a, t, 1600, InputEvent::NONE, still_accel, {0.0f, 0.0f, 0.0f});
}

// From ZERO_CAL/PROMPT_FLAT: capture flat then raise, landing in ACTIVE.
static void zero_cal_flat_raise(App& a, uint32_t& t, Vec3 flat_pose, Vec3 raise_pose) {
    advance(a, t, 100, InputEvent::A_SHORT, flat_pose);   // -> CAPTURE_FLAT
    drive_still(a, t, flat_pose);                          // -> PROMPT_RAISE
    advance(a, t, 100, InputEvent::A_SHORT, raise_pose);   // -> CAPTURE_RAISE
    drive_still(a, t, raise_pose);                         // -> ACTIVE
}

void setUp(void) {
    settings::save_tolerance(Tolerance::NORMAL);
    settings::save_buzzer(false);
    session::clear();
    settings::clear_session_history();   // keep tests order-independent
}
void tearDown(void) {}

// Reach ACTIVE via the resume path (bypasses ZERO_CAL) with a valid single zero.
static void reach_active(App& a, uint32_t& t) {
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_flat     = {0.0f, 0.0f, -1.0f};
    s.edge_axis  = {1.0f, 0.0f,  0.0f};
    session::mark_active(s);
    a.begin(true);                                  // lands in RESUME_PROMPT
    advance(a, t, 100, InputEvent::A_SHORT);        // RESUME_PROMPT -> ACTIVE
}

void test_boot_without_session_goes_to_set_target(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_boot_with_session_goes_to_resume_prompt(void) {
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_flat     = {0.0f, 0.0f, -1.0f};
    s.edge_axis  = {1.0f, 0.0f,  0.0f};
    session::mark_active(s);
    a.begin(true);
    TEST_ASSERT_EQUAL_INT((int)State::RESUME_PROMPT, (int)a.current());
}

void test_incomplete_session_routes_to_zero_cal(void) {
    // A session with no flat reference (g_flat zero) must go to ZERO_CAL, not resume.
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    // g_flat left {0,0,0}
    session::mark_active(s);
    a.begin(true);
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_FLAT, (int)a.zero_cal_substate());
}

void test_resume_prompt_a_confirms_active(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_resume_preserves_restored_side(void) {
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_flat     = {0.0f, 0.0f, -1.0f};
    s.edge_axis  = {1.0f, 0.0f,  0.0f};
    s.current_side = Side::B;
    s.strokes_A  = 3;
    s.strokes_B  = 5;
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 100, InputEvent::A_SHORT);   // RESUME_PROMPT -> ACTIVE
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)a.current_side());
    TEST_ASSERT_EQUAL_UINT32(3, a.strokes_a());
    TEST_ASSERT_EQUAL_UINT32(5, a.strokes_b());
}

void test_resume_prompt_b_starts_new_session(void) {
    App a;
    uint32_t t = 0;
    SessionState s;
    s.target_deg = 17.0f; s.tolerance = Tolerance::NORMAL;
    s.g_flat = {0.0f, 0.0f, -1.0f}; s.edge_axis = {1.0f, 0.0f, 0.0f};
    session::mark_active(s);
    a.begin(true);
    advance(a, t, 100, InputEvent::B_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_resume_prompt_times_out_to_set_target(void) {
    App a;
    uint32_t t = 0;
    SessionState s;
    s.target_deg = 17.0f; s.tolerance = Tolerance::NORMAL;
    s.g_flat = {0.0f, 0.0f, -1.0f}; s.edge_axis = {1.0f, 0.0f, 0.0f};
    session::mark_active(s);
    a.begin(true);
    advance(a, t, 5500);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_set_target_a_captures_and_advances(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_set_target_b_enters_preset_mode_and_cycles(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::B_SHORT);
    advance(a, t, 100, InputEvent::B_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, a.target_deg());
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_preset_cycles_to_25_and_28(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    // B enters preset mode on 12; each further B steps 15/17/20/22/25/28.
    for (int i = 0; i < 6; i++) advance(a, t, 100, InputEvent::B_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, a.target_deg());

    App b;
    b.begin(false);
    t = 0;
    advance(b, t, 2100);
    for (int i = 0; i < 7; i++) advance(b, t, 100, InputEvent::B_SHORT);
    advance(b, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_FLOAT(28.0f, b.target_deg());
}

void test_preset_cancel_returns_to_live_capture(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    for (int i = 0; i < 8; i++) advance(a, t, 100, InputEvent::B_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

// --- VERIFY (accuracy check) ---

void test_set_target_a_long_opens_verify(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_LONG);
    TEST_ASSERT_EQUAL_INT((int)State::VERIFY, (int)a.current());
}

void test_verify_b_returns_without_starting_a_session(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_LONG);
    advance(a, t, 100, InputEvent::B_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_verify_reads_a_known_tilt_after_capture(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_LONG);          // -> VERIFY
    advance(a, t, 100, InputEvent::A_SHORT, {0.0f, 0.0f, -1.0f});   // start capture
    drive_still(a, t, {0.0f, 0.0f, -1.0f});                        // flat reference
    TEST_ASSERT_EQUAL_INT((int)State::VERIFY, (int)a.current());
    // Now present a known 17 deg tilt and let the filter settle on it.
    advance(a, t, 4000, InputEvent::NONE, g_now_at_target_17);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 17.0f, a.verify_reading_deg());
}

void test_verify_does_not_disturb_an_existing_zero(void) {
    // Checking accuracy must not cost you the zero you already set.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    const Vec3 before = a.g_flat();
    advance(a, t, 100, InputEvent::A_LONG);          // ACTIVE -> SUMMARY
    advance(a, t, 100, InputEvent::A_SHORT);         // -> SET_TARGET
    advance(a, t, 100, InputEvent::A_LONG);          // -> VERIFY
    advance(a, t, 100, InputEvent::A_SHORT, {0.0f, 0.3f, -0.95f});
    drive_still(a, t, {0.0f, 0.3f, -0.95f});         // capture a DIFFERENT reference
    TEST_ASSERT_EQUAL_FLOAT(before.x, a.g_flat().x);
    TEST_ASSERT_EQUAL_FLOAT(before.y, a.g_flat().y);
    TEST_ASSERT_EQUAL_FLOAT(before.z, a.g_flat().z);
}

void test_inverted_mount_reads_the_same_angle(void) {
    // Reversed/left-handed mount. bevel_angle() folds gravity in the opposite
    // hemisphere back into 0..90, so an upside-down device should report the same
    // bevel as an upright one. This pins that end-to-end through the app, so the
    // property cannot regress unnoticed.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 3000, InputEvent::NONE, g_now_at_target_17);
    const uint8_t upright_green = a.green_pct();

    App b;
    uint32_t t2 = 0;
    reach_active(b, t2);
    const Vec3 inverted = { -g_now_at_target_17.x, -g_now_at_target_17.y, -g_now_at_target_17.z };
    advance(b, t2, 3000, InputEvent::NONE, inverted);
    // Both should sit in tolerance: the fold makes the flipped pose read 17 deg.
    TEST_ASSERT_TRUE(upright_green > 50);
    TEST_ASSERT_TRUE(b.green_pct() > 50);
}

// --- Session history + time-on-angle ---

void test_green_pct_counts_time_in_tolerance(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    // Half the ticks on target (17 deg), half well off it.
    advance(a, t, 2000, InputEvent::NONE, g_now_at_target_17);
    advance(a, t, 2000, InputEvent::NONE, {0.0f, 0.0f, -1.0f});   // 0 deg = way low
    TEST_ASSERT_TRUE(a.green_pct() > 25);
    TEST_ASSERT_TRUE(a.green_pct() < 75);
}

void test_green_pct_is_zero_with_no_active_ticks(void) {
    // Must not divide by zero before ACTIVE has run.
    App a;
    a.begin(false);
    TEST_ASSERT_EQUAL_UINT8(0, a.green_pct());
}

void test_finished_session_is_recorded(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 2000, InputEvent::NONE, g_now_at_target_17);
    advance(a, t, 100, InputEvent::A_LONG);          // ACTIVE -> SUMMARY
    TEST_ASSERT_EQUAL_INT((int)State::SUMMARY, (int)a.current());

    SessionRecord recs[kSessionHistoryMax];
    int n = settings::load_session_history(recs, kSessionHistoryMax);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_UINT16(170, recs[0].target_deg_x10);   // 17.0 deg
    TEST_ASSERT_TRUE(recs[0].green_pct > 0);
}

void test_history_keeps_newest_first_and_caps_at_five(void) {
    for (int i = 0; i < 7; i++) {
        SessionRecord r;
        r.target_deg_x10 = (uint16_t)(100 + i);   // distinguishable per push
        settings::push_session_record(r);
    }
    SessionRecord recs[kSessionHistoryMax];
    int n = settings::load_session_history(recs, kSessionHistoryMax);
    TEST_ASSERT_EQUAL_INT(kSessionHistoryMax, n);
    TEST_ASSERT_EQUAL_UINT16(106, recs[0].target_deg_x10);   // newest first
    TEST_ASSERT_EQUAL_UINT16(102, recs[4].target_deg_x10);   // oldest kept
}

void test_summary_b_long_opens_history_and_any_press_returns(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 500, InputEvent::NONE, g_now_at_target_17);
    advance(a, t, 100, InputEvent::A_LONG);          // -> SUMMARY
    advance(a, t, 100, InputEvent::B_LONG);          // -> HISTORY
    TEST_ASSERT_EQUAL_INT((int)State::HISTORY, (int)a.current());
    advance(a, t, 100, InputEvent::A_SHORT);         // any press returns
    TEST_ASSERT_EQUAL_INT((int)State::SUMMARY, (int)a.current());
}

void test_new_session_from_summary_resets_green_stats(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 1000, InputEvent::NONE, g_now_at_target_17);
    advance(a, t, 100, InputEvent::A_LONG);          // -> SUMMARY
    advance(a, t, 100, InputEvent::A_SHORT);         // -> SET_TARGET (new session)
    TEST_ASSERT_EQUAL_UINT8(0, a.green_pct());
}

// --- Continuous gyro-bias refresh ---

void test_bias_converges_toward_offset_when_held_still(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    // A steady 1 dps reading on a device that is genuinely at rest IS bias.
    const Vec3 offset = {1.0f, 0.0f, 0.0f};
    advance(a, t, 6000, InputEvent::NONE, {0.0f, 0.0f, -1.0f}, offset);
    TEST_ASSERT_TRUE(a.gyro_bias().x > 0.3f);      // it learned it
    TEST_ASSERT_TRUE(a.gyro_bias().x <= 1.0f);     // and did not overshoot
}

void test_bias_ignores_a_slow_steady_rotation(void) {
    // THE failure mode that matters. A device turning slowly reads a low but
    // non-zero rate; absorbing that as bias would make the filter permanently
    // wrong in the direction of the turn. 5 dps is slow — and must be rejected.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 6000, InputEvent::NONE, {0.0f, 0.0f, -1.0f}, {5.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, a.gyro_bias().x);
}

void test_bias_ignores_ticks_while_accelerating(void) {
    // Gyro quiet but the device is being moved: |a| is not 1 g, so it is not at
    // rest and the gyro reading is not necessarily bias.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 6000, InputEvent::NONE, {0.0f, 0.4f, -1.0f}, {1.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, a.gyro_bias().x);
}

void test_bias_requires_a_sustained_still_run(void) {
    // A brief pause must not be enough — stillness has to hold for BIAS_STILL_TICKS
    // before anything is learned, or hand tremor between strokes would qualify.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    // Break any still run already accumulated on entry to ACTIVE, so this
    // measures the short run below and nothing else.
    advance(a, t, 200, InputEvent::NONE, {0.0f, 0.0f, -1.0f}, {50.0f, 0.0f, 0.0f});
    // 20 ticks of stillness — under BIAS_STILL_TICKS (25), so nothing is learned.
    advance(a, t, 200, InputEvent::NONE, {0.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, a.gyro_bias().x);
}

void test_bias_rejects_slow_rotation_below_gyro_threshold(void) {
    // The gyro-rate guard alone cannot see a rotation slower than its 2 dps
    // threshold — |a| stays at 1 g throughout. The accel-direction anchor must
    // catch it and revert anything learned inside the window. Without the
    // anchor, 6 s of this teaches the filter ~1.4 dps of phantom bias.
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    const float RATE = 1.5f;                       // dps, below BIAS_STILL_GYRO_DPS
    float theta = 0.0f;
    for (int i = 0; i < 300; i++) {                // 6 s at the 10 ms test tick
        t += 10;
        theta += RATE * 0.01f * (float)M_PI / 180.0f;
        Vec3 acc = {0.0f, std::sin(theta), -std::cos(theta)};
        App::Tick tick{t, InputEvent::NONE, acc, {RATE, 0.0f, 0.0f}, FaultCode::NONE};
        a.on_tick(tick);
    }
    TEST_ASSERT_TRUE(std::fabs(a.gyro_bias().x) < 0.4f);
}

void test_history_visit_does_not_duplicate_the_record(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 1000, InputEvent::NONE, g_now_at_target_17);
    advance(a, t, 100, InputEvent::A_LONG);          // ACTIVE -> SUMMARY (records)
    advance(a, t, 100, InputEvent::B_LONG);          // SUMMARY -> HISTORY
    advance(a, t, 100, InputEvent::A_SHORT);         // HISTORY -> SUMMARY (must NOT record)
    advance(a, t, 100, InputEvent::B_LONG);          // and again for good measure
    advance(a, t, 100, InputEvent::A_SHORT);
    SessionRecord recs[kSessionHistoryMax];
    TEST_ASSERT_EQUAL_INT(1, settings::load_session_history(recs, kSessionHistoryMax));
}

void test_resume_restores_time_on_angle(void) {
    // A session resumed after deep sleep must keep its pre-sleep green stats,
    // or green_pct and the final record silently drop everything before sleep.
    App a;
    SessionState s;
    s.target_deg   = 17.0f;
    s.tolerance    = Tolerance::NORMAL;
    s.g_flat       = {0.0f, 0.0f, -1.0f};
    s.edge_axis    = {1.0f, 0.0f,  0.0f};
    s.active_ticks = 1000;
    s.green_ticks  = 500;                            // 50% before sleep
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 100, InputEvent::A_SHORT);         // RESUME_PROMPT -> ACTIVE
    TEST_ASSERT_TRUE(a.green_pct() >= 45 && a.green_pct() <= 55);
}

void test_tolerance_b_long_toggles_steady_without_advancing(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);    // SET_TARGET -> SET_TOLERANCE
    bool before = a.steady_on();
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_EQUAL_INT((int)!before, (int)a.steady_on());
    // The A/B switch must not disturb the flow it lives on.
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_EQUAL_INT((int)before, (int)a.steady_on());
}

void test_tolerance_b_long_does_not_change_tolerance(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    Tolerance before = a.tolerance();
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_EQUAL_INT((int)before, (int)a.tolerance());
}

void test_tolerance_a_confirms_and_advances(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TARGET -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TOLERANCE -> ZERO_CAL
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_FLAT, (int)a.zero_cal_substate());
}

void test_active_long_a_goes_to_summary(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 100, InputEvent::A_LONG, g_now_at_target_17);
    TEST_ASSERT_EQUAL_INT((int)State::SUMMARY, (int)a.current());
}

void test_summary_a_starts_new_session(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 100, InputEvent::A_LONG, g_now_at_target_17);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_active_long_b_toggles_buzzer_persistently(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    TEST_ASSERT_FALSE(a.buzzer_on());
    advance(a, t, 100, InputEvent::B_LONG, g_now_at_target_17);
    TEST_ASSERT_TRUE(a.buzzer_on());
    TEST_ASSERT_TRUE(settings::load_buzzer());
    advance(a, t, 100, InputEvent::B_LONG, g_now_at_target_17);
    TEST_ASSERT_FALSE(a.buzzer_on());
    TEST_ASSERT_FALSE(settings::load_buzzer());
}

void test_imu_fault_at_boot_goes_to_fault(void) {
    App a;
    a.begin(false);
    uint32_t t = 50;
    App::Tick tick{t, InputEvent::NONE, {0,0,-1}, {0,0,0}, FaultCode::E01_BEGIN_FAILED};
    a.on_tick(tick);
    TEST_ASSERT_EQUAL_INT((int)State::FAULT, (int)a.current());
}

void test_zero_cal_flat_raise_flow_advances_to_active(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);   // -> ZERO_CAL (PROMPT_FLAT)
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_FLAT, (int)a.zero_cal_substate());

    Vec3 flat  = {0.0f, 0.0f, -1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, flat);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::CAPTURE_FLAT, (int)a.zero_cal_substate());
    drive_still(a, t, flat);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_RAISE, (int)a.zero_cal_substate());

    Vec3 raise = {0.0f, 0.2924f, -0.9563f};   // 17 deg about edge (body +x)
    advance(a, t, 100, InputEvent::A_SHORT, raise);
    drive_still(a, t, raise);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    Vec3 gf = a.g_flat();
    TEST_ASSERT_FLOAT_WITHIN(0.02f,  0.0f, gf.x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -1.0f, gf.z);
    Vec3 e = a.edge_axis();                    // unit(g_flat x g_raise) ~ {1,0,0}
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, std::fabs(e.x));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, e.z);
}

void test_zero_cal_long_a_aborts_to_set_target(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);   // -> ZERO_CAL
    advance(a, t, 100, InputEvent::A_LONG);    // abort
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_zero_cal_abort_mid_capture_then_reenter_completes(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);   // -> ZERO_CAL (PROMPT_FLAT)

    Vec3 flat = {0.0f, 0.0f, -1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, flat);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::CAPTURE_FLAT, (int)a.zero_cal_substate());
    advance(a, t, 600, InputEvent::NONE, flat);   // partial
    advance(a, t, 100, InputEvent::A_LONG);       // abort
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());

    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);      // -> ZERO_CAL (PROMPT_FLAT again)
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_FLAT, (int)a.zero_cal_substate());

    zero_cal_flat_raise(a, t, flat, (Vec3){0.0f, 0.2924f, -0.9563f});
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_active_green_at_target_pose_no_phantom_strokes(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);                       // g_flat={0,0,-1}, edge={1,0,0}
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
    advance(a, t, 500, InputEvent::NONE, g_now_at_target_17);  // held at 17 deg -> green
    TEST_ASSERT_EQUAL_UINT32(0, a.strokes_a());
    TEST_ASSERT_EQUAL_UINT32(0, a.strokes_b());
}

void test_e2e_tilted_stone_bevel_is_relative_to_flat(void) {
    // Stone tilted 5 deg; the flat capture absorbs it, so a 17 deg bevel reads 17.
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TARGET (default 17)
    advance(a, t, 100, InputEvent::A_SHORT);   // -> ZERO_CAL
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());

    const float r5  = 5.0f  * (float)M_PI / 180.0f;
    const float r22 = 22.0f * (float)M_PI / 180.0f;
    Vec3 flat  = {0.0f,  std::sin(r5),  -std::cos(r5)};   // 5 deg tilt about edge (x)
    Vec3 raise = {0.0f,  std::sin(r22), -std::cos(r22)};  // 17 deg above flat
    zero_cal_flat_raise(a, t, flat, raise);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    // At the on-target pose (17 deg above the tilted flat), the skew-corrected
    // bevel must read ~17 regardless of the stone tilt.
    float b = bevel_angle(a.g_flat(), a.edge_axis(), raise);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 17.0f, b);
}

void test_e2e_manual_side_switch_sticks(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)a.current_side());
    advance(a, t, 100, InputEvent::B_SHORT, g_now_at_target_17);
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)a.current_side());
    advance(a, t, 3000, InputEvent::NONE, g_now_at_target_17);   // must NOT auto-revert
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)a.current_side());
    advance(a, t, 100, InputEvent::B_SHORT, g_now_at_target_17);
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)a.current_side());
}

void test_rezero_updates_flat_reference(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);                       // g_flat = {0,0,-1}
    advance(a, t, 100, InputEvent::A_SHORT, {0.0f, 0.0f, -1.0f});
    TEST_ASSERT_EQUAL_INT((int)State::REZERO, (int)a.current());

    Vec3 new_flat = {0.0f, 0.5f, -0.8660254f};   // re-capture at a new still pose
    advance(a, t, 1700, InputEvent::NONE, new_flat, {0.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    Vec3 gf = a.g_flat();
    TEST_ASSERT_FLOAT_WITHIN(0.02f,  0.5f,        gf.y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.8660254f,  gf.z);
}

void test_rezero_abort_leaves_flat_unchanged(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 100, InputEvent::A_SHORT, {0.0f, 0.0f, -1.0f});
    TEST_ASSERT_EQUAL_INT((int)State::REZERO, (int)a.current());
    advance(a, t, 300, InputEvent::NONE,    {0.0f, 0.5f, -0.8660254f}, {0,0,0});
    advance(a, t, 100, InputEvent::B_SHORT, {0.0f, 0.5f, -0.8660254f}, {0,0,0});  // abort
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
    Vec3 gf = a.g_flat();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, gf.z);   // unchanged
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_without_session_goes_to_set_target);
    RUN_TEST(test_boot_with_session_goes_to_resume_prompt);
    RUN_TEST(test_incomplete_session_routes_to_zero_cal);
    RUN_TEST(test_resume_prompt_a_confirms_active);
    RUN_TEST(test_resume_preserves_restored_side);
    RUN_TEST(test_resume_prompt_b_starts_new_session);
    RUN_TEST(test_resume_prompt_times_out_to_set_target);
    RUN_TEST(test_set_target_a_captures_and_advances);
    RUN_TEST(test_set_target_b_enters_preset_mode_and_cycles);
    RUN_TEST(test_preset_cycles_to_25_and_28);
    RUN_TEST(test_preset_cancel_returns_to_live_capture);
    RUN_TEST(test_set_target_a_long_opens_verify);
    RUN_TEST(test_verify_b_returns_without_starting_a_session);
    RUN_TEST(test_verify_reads_a_known_tilt_after_capture);
    RUN_TEST(test_verify_does_not_disturb_an_existing_zero);
    RUN_TEST(test_inverted_mount_reads_the_same_angle);
    RUN_TEST(test_green_pct_counts_time_in_tolerance);
    RUN_TEST(test_green_pct_is_zero_with_no_active_ticks);
    RUN_TEST(test_finished_session_is_recorded);
    RUN_TEST(test_history_keeps_newest_first_and_caps_at_five);
    RUN_TEST(test_summary_b_long_opens_history_and_any_press_returns);
    RUN_TEST(test_new_session_from_summary_resets_green_stats);
    RUN_TEST(test_bias_converges_toward_offset_when_held_still);
    RUN_TEST(test_bias_ignores_a_slow_steady_rotation);
    RUN_TEST(test_bias_ignores_ticks_while_accelerating);
    RUN_TEST(test_bias_requires_a_sustained_still_run);
    RUN_TEST(test_bias_rejects_slow_rotation_below_gyro_threshold);
    RUN_TEST(test_history_visit_does_not_duplicate_the_record);
    RUN_TEST(test_resume_restores_time_on_angle);
    RUN_TEST(test_tolerance_b_long_toggles_steady_without_advancing);
    RUN_TEST(test_tolerance_b_long_does_not_change_tolerance);
    RUN_TEST(test_tolerance_a_confirms_and_advances);
    RUN_TEST(test_active_long_a_goes_to_summary);
    RUN_TEST(test_summary_a_starts_new_session);
    RUN_TEST(test_active_long_b_toggles_buzzer_persistently);
    RUN_TEST(test_imu_fault_at_boot_goes_to_fault);
    RUN_TEST(test_zero_cal_flat_raise_flow_advances_to_active);
    RUN_TEST(test_zero_cal_long_a_aborts_to_set_target);
    RUN_TEST(test_zero_cal_abort_mid_capture_then_reenter_completes);
    RUN_TEST(test_active_green_at_target_pose_no_phantom_strokes);
    RUN_TEST(test_e2e_tilted_stone_bevel_is_relative_to_flat);
    RUN_TEST(test_e2e_manual_side_switch_sticks);
    RUN_TEST(test_rezero_updates_flat_reference);
    RUN_TEST(test_rezero_abort_leaves_flat_unchanged);
    return UNITY_END();
}
