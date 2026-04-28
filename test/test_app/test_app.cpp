#include <unity.h>
#include "app.h"
#include "settings.h"
#include "session.h"

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
    advance(a, t, 100, InputEvent::A_LONG);
    TEST_ASSERT_EQUAL_INT((int)State::SUMMARY, (int)a.current());
}

void test_summary_a_starts_new_session(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    advance(a, t, 100, InputEvent::A_LONG);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_active_long_b_toggles_buzzer_persistently(void) {
    App a;
    uint32_t t = 0;
    reach_active(a, t);
    TEST_ASSERT_FALSE(a.buzzer_on());
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_TRUE(a.buzzer_on());
    TEST_ASSERT_TRUE(settings::load_buzzer());
    advance(a, t, 100, InputEvent::B_LONG);
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_without_session_goes_to_set_target);
    RUN_TEST(test_boot_with_session_goes_to_resume_prompt);
    RUN_TEST(test_resume_prompt_a_confirms_active);
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
    return UNITY_END();
}
