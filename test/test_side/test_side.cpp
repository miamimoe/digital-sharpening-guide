#include <unity.h>
#include "side.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(SideFSM& fsm, uint32_t& t, uint32_t dt_ms, float accel_mag, float grav_dot_ref) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        fsm.update(t, accel_mag, grav_dot_ref);
    }
}

void test_starts_on_side_a_no_events(void) {
    SideFSM fsm;
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
    TEST_ASSERT_FALSE(fsm.consume_switch());
}

void test_peel_flip_settle_triggers_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);
    drive(fsm, t, 100,  1.80f, +1.0f);
    drive(fsm, t, 200,  0.40f, -1.0f);
    drive(fsm, t, 600,  1.00f, -1.0f);
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_peel_no_flip_settle_no_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);
    drive(fsm, t, 100,  1.80f, +1.0f);
    drive(fsm, t, 200,  0.40f, +1.0f);
    drive(fsm, t, 600,  1.00f, +1.0f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_peel_no_settle_within_timeout_resets(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);
    drive(fsm, t, 100,  1.80f, +1.0f);
    drive(fsm, t, 5500, 0.50f, -1.0f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_manual_toggle_switches_and_suppresses_auto(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, +1.0f);
    fsm.manual_toggle(t);
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
    TEST_ASSERT_TRUE(fsm.consume_switch());
    drive(fsm, t, 100, 1.80f, -1.0f);
    drive(fsm, t, 1800, 0.40f, -1.0f);
    drive(fsm, t, 600,  1.00f, +1.0f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_suppression_expires_after_2s(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, +1.0f);
    fsm.manual_toggle(t);
    fsm.consume_switch();
    drive(fsm, t, 2200, 1.0f, -1.0f);
    drive(fsm, t, 100,  1.80f, -1.0f);
    drive(fsm, t, 200,  0.40f, +1.0f);
    drive(fsm, t, 600,  1.00f, +1.0f);
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_on_side_a_no_events);
    RUN_TEST(test_peel_flip_settle_triggers_switch);
    RUN_TEST(test_peel_no_flip_settle_no_switch);
    RUN_TEST(test_peel_no_settle_within_timeout_resets);
    RUN_TEST(test_manual_toggle_switches_and_suppresses_auto);
    RUN_TEST(test_suppression_expires_after_2s);
    return UNITY_END();
}
