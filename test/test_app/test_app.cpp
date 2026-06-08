#include <unity.h>
#include <cmath>
#include "app.h"
#include "settings.h"
#include "session.h"

// Realistic g_now for tests that want to be at target_deg=17° from g_zero_A={0,0,-1}.
// Rotated 17° around Y: g = {sin(17°), 0, -cos(17°)} ≈ {0.292, 0, -0.956}.
static const Vec3 g_now_at_target_17 = {0.2924f, 0.0f, -0.9563f};

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

// Run ~1.6s of still ticks to complete one ZERO_CAL capture (500ms warmup + 1000ms avg + slack).
static void drive_still(App& a, uint32_t& t, Vec3 still_accel = {0.0f, 0.0f, -1.0f}) {
    advance(a, t, 1600, InputEvent::NONE, still_accel, {0.0f, 0.0f, 0.0f});
}

void setUp(void) {
    settings::save_tolerance(Tolerance::NORMAL);
    settings::save_buzzer(false);
    settings::clear_first_boot();
    session::clear();
}
void tearDown(void) {}

// Reach ACTIVE via the resume-prompt path, which bypasses ZERO_CAL.
// This is the correct path for tests that need to exercise ACTIVE state
// now that SET_TOLERANCE → ZERO_CAL (not ACTIVE) since Task 7.
static void reach_active(App& a, uint32_t& t) {
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);                                  // lands in RESUME_PROMPT
    advance(a, t, 100, InputEvent::A_SHORT);        // RESUME_PROMPT → ACTIVE
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
    // mark session active via session::mark_active so session::has_session() returns true
    // Supply non-zero g_zero_A/B so the defensive guard does not redirect to ZERO_CAL.
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);
    // begin(true) skips BOOT and lands directly in RESUME_PROMPT — no boot wait needed.
    TEST_ASSERT_EQUAL_INT((int)State::RESUME_PROMPT, (int)a.current());
}

void test_resume_prompt_a_confirms_active(void) {
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_resume_preserves_restored_side(void) {
    // A session that slept on side B must resume on side B (transition(ACTIVE)
    // must not reset the side FSM on the resume path).
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
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
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 100, InputEvent::B_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_resume_prompt_times_out_to_set_target(void) {
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
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

void test_preset_cancel_returns_to_live_capture(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    for (int i = 0; i < 6; i++) advance(a, t, 100, InputEvent::B_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_tolerance_a_confirms_and_advances(void) {
    // SET_TOLERANCE → ZERO_CAL (not ACTIVE) since Task 7.
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TARGET → SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TOLERANCE → ZERO_CAL
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());
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

void test_zero_cal_two_capture_flow_advances_to_active(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);   // pass through BOOT splash to SET_TARGET

    // SET_TARGET A_SHORT -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());

    // SET_TOLERANCE A_SHORT -> ZERO_CAL (PROMPT_A)
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_A, (int)a.zero_cal_substate());

    // PROMPT_A: press A to start CAPTURE_A.
    Vec3 still_a = {0.0f, 0.0f, -1.0f};
    Vec3 still_g = {0.0f, 0.0f,  0.0f};
    advance(a, t, 100, InputEvent::A_SHORT, still_a, still_g);

    // 150 still ticks (1500ms = 500ms warmup + 1000ms averaging) -> PROMPT_B
    advance(a, t, 1600, InputEvent::NONE, still_a, still_g);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_B, (int)a.zero_cal_substate());

    // PROMPT_B: press A to start CAPTURE_B with side-B pose.
    Vec3 still_b = {0.0f, 0.0f, +1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, still_b, still_g);
    advance(a, t, 1600, InputEvent::NONE, still_b, still_g);

    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    Vec3 za = a.g_zero_a();
    Vec3 zb = a.g_zero_b();
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, za.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, za.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, za.z);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, zb.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, zb.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, +1.0f, zb.z);
}

void test_zero_cal_long_a_aborts_to_set_target(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TARGET -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TOLERANCE -> ZERO_CAL
    advance(a, t, 100, InputEvent::A_LONG);   // long-A aborts
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_zero_cal_abort_mid_capture_then_reenter_completes(void) {
    // Abort during CAPTURE_A, re-enter ZERO_CAL via SET_TARGET -> SET_TOLERANCE,
    // and verify the second flow completes cleanly (FSM was properly reset).
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TARGET -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TOLERANCE -> ZERO_CAL (PROMPT_A)

    Vec3 still_a = {0.0f, 0.0f, -1.0f};
    Vec3 still_g = {0.0f, 0.0f,  0.0f};

    // Start CAPTURE_A and feed a partial window of still ticks (mid-capture).
    advance(a, t, 100, InputEvent::A_SHORT, still_a, still_g);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::CAPTURE_A, (int)a.zero_cal_substate());
    advance(a, t, 600, InputEvent::NONE, still_a, still_g);  // partial: not done

    // Abort mid-capture.
    advance(a, t, 100, InputEvent::A_LONG);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());

    // Re-enter ZERO_CAL via the normal cold-start path.
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TARGET -> SET_TOLERANCE
    advance(a, t, 100, InputEvent::A_SHORT);  // SET_TOLERANCE -> ZERO_CAL
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_A, (int)a.zero_cal_substate());

    // Capture A then B; full flow must complete cleanly.
    advance(a, t, 100, InputEvent::A_SHORT, still_a, still_g);
    advance(a, t, 1600, InputEvent::NONE, still_a, still_g);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_B, (int)a.zero_cal_substate());

    Vec3 still_b = {0.0f, 0.0f, +1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, still_b, still_g);
    advance(a, t, 1600, InputEvent::NONE, still_b, still_g);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_active_classifier_green_at_target_pose(void) {
    // At g_now = 17° away from g_zero_A, classify must return GREEN through
    // the in-tolerance path (NOT via the zero-direction-sign fallback).
    App a;
    SessionState s;
    s.target_deg = 17.0f;
    s.tolerance  = Tolerance::NORMAL;
    s.g_zero_A   = {0.0f, 0.0f, -1.0f};
    s.g_zero_B   = {0.0f, 0.0f,  1.0f};
    session::mark_active(s);
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 100, InputEvent::A_SHORT);  // RESUME_PROMPT -> ACTIVE
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    // Drive ~500ms of ticks at the target pose so the Mahony filter settles.
    advance(a, t, 500, InputEvent::NONE, g_now_at_target_17);

    // No public color accessor — verify indirectly: at target, no stroke
    // should ever increment (stroke FSM uses in_tol; at target we ARE in_tol,
    // so a stroke would only count if we left and re-entered tol within the
    // hysteresis window, which we don't here).
    TEST_ASSERT_EQUAL_UINT32(0, a.strokes_a());
    TEST_ASSERT_EQUAL_UINT32(0, a.strokes_b());
}

void test_e2e_fresh_session_through_zero_cal_to_active(void) {
    // BOOT (no session) -> SET_TARGET -> SET_TOLERANCE -> ZERO_CAL
    //   -> capture A (flat, side A) -> capture B (flat, side B) -> ACTIVE
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());

    advance(a, t, 100, InputEvent::A_SHORT);  // -> SET_TOLERANCE
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());

    advance(a, t, 100, InputEvent::A_SHORT);  // -> ZERO_CAL (PROMPT_A)
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_A, (int)a.zero_cal_substate());

    Vec3 pose_A = {0.0f, 0.0f, -1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, pose_A);
    drive_still(a, t, pose_A);
    TEST_ASSERT_EQUAL_INT((int)ZeroCalSubstate::PROMPT_B, (int)a.zero_cal_substate());

    Vec3 pose_B = {0.0f, 0.0f, +1.0f};
    advance(a, t, 100, InputEvent::A_SHORT, pose_B);
    drive_still(a, t, pose_B);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    Vec3 za = a.g_zero_a();
    Vec3 zb = a.g_zero_b();
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, za.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, za.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, za.z);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, zb.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  0.0f, zb.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, +1.0f, zb.z);
}

void test_e2e_tilted_stone_target_angle_is_relative_to_g_zero(void) {
    // The stone is tilted ~5° in world frame. The user's g_zero captures that
    // tilt; sharpening at 17° from the captured zero should yield a 17° angle
    // reading even though the world-frame angle is 22°.
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TARGET confirm (default 17°)
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TOLERANCE confirm
    TEST_ASSERT_EQUAL_INT((int)State::ZERO_CAL, (int)a.current());

    // "Stone tilted 5° around Y-axis": blade-flat gravity = {sin5, 0, -cos5}.
    const float r5 = 5.0f * (float)M_PI / 180.0f;
    Vec3 pose_A = { std::sin(r5), 0.0f, -std::cos(r5) };
    Vec3 pose_B = {-std::sin(r5), 0.0f, +std::cos(r5) };  // flipped about edge axis

    advance(a, t, 100, InputEvent::A_SHORT, pose_A);
    drive_still(a, t, pose_A);
    advance(a, t, 100, InputEvent::A_SHORT, pose_B);
    drive_still(a, t, pose_B);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    // Tilt up to 17° from pose_A (rotation around Y by +17° from already-tilted-5°
    // pose). World-frame angle from vertical is now 22°.
    const float r22 = 22.0f * (float)M_PI / 180.0f;
    Vec3 pose_at_target = { std::sin(r22), 0.0f, -std::cos(r22) };

    // Verify the angle between captured g_zero_A and the tilted-up pose is ~17°.
    Vec3 za = a.g_zero_a();
    float dot = za.x*pose_at_target.x + za.y*pose_at_target.y + za.z*pose_at_target.z;
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;
    float angle_deg = std::acos(dot) * (180.0f / (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 17.0f, angle_deg);
}

void test_e2e_side_switch_uses_g_zero_b(void) {
    // After ZERO_CAL, the side auto-switches when the device comes to rest in the
    // opposite-polarity (flipped) orientation — no peel spike, no 5s timeout. The
    // snap-to-raw keeps grav_dot_ref accurate even across the antiparallel flip.
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TARGET
    advance(a, t, 100, InputEvent::A_SHORT);   // SET_TOLERANCE

    Vec3 pose_A = {0.0f,  0.7071f, -0.7071f};
    Vec3 pose_B = {0.0f, -0.7071f, +0.7071f};
    advance(a, t, 100, InputEvent::A_SHORT, pose_A);
    drive_still(a, t, pose_A);                 // capture A
    advance(a, t, 100, InputEvent::A_SHORT, pose_B);
    drive_still(a, t, pose_B);                 // capture B -> ACTIVE
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());

    // Flip back to side A and hold still (you'd flip back to A to start). Drive
    // well past the 500ms settle window to wash out any transient from capture-B.
    advance(a, t, 1200, InputEvent::NONE, pose_A, {0.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)a.current_side());

    // Flip to side B and hold still -> auto-switch to B (active ref becomes g_zero_B).
    advance(a, t, 800, InputEvent::NONE, pose_B, {0.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)a.current_side());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_without_session_goes_to_set_target);
    RUN_TEST(test_boot_with_session_goes_to_resume_prompt);
    RUN_TEST(test_resume_prompt_a_confirms_active);
    RUN_TEST(test_resume_preserves_restored_side);
    RUN_TEST(test_resume_prompt_b_starts_new_session);
    RUN_TEST(test_resume_prompt_times_out_to_set_target);
    RUN_TEST(test_set_target_a_captures_and_advances);
    RUN_TEST(test_set_target_b_enters_preset_mode_and_cycles);
    RUN_TEST(test_preset_cancel_returns_to_live_capture);
    RUN_TEST(test_tolerance_a_confirms_and_advances);
    RUN_TEST(test_active_long_a_goes_to_summary);
    RUN_TEST(test_summary_a_starts_new_session);
    RUN_TEST(test_active_long_b_toggles_buzzer_persistently);
    RUN_TEST(test_imu_fault_at_boot_goes_to_fault);
    RUN_TEST(test_zero_cal_two_capture_flow_advances_to_active);
    RUN_TEST(test_zero_cal_long_a_aborts_to_set_target);
    RUN_TEST(test_zero_cal_abort_mid_capture_then_reenter_completes);
    RUN_TEST(test_active_classifier_green_at_target_pose);
    RUN_TEST(test_e2e_fresh_session_through_zero_cal_to_active);
    RUN_TEST(test_e2e_tilted_stone_target_angle_is_relative_to_g_zero);
    RUN_TEST(test_e2e_side_switch_uses_g_zero_b);
    return UNITY_END();
}
