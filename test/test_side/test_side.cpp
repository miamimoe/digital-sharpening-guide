#include <unity.h>
#include "side.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(SideFSM& fsm, uint32_t& t, uint32_t dt_ms,
                  float accel_mag, float gyro_mag, float grav_dot_ref) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        fsm.update(t, accel_mag, gyro_mag, grav_dot_ref);
    }
}

void test_starts_on_side_a_no_events(void) {
    SideFSM fsm;
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
    TEST_ASSERT_FALSE(fsm.consume_switch());
}

void test_settled_in_flipped_pose_switches(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 300, 1.0f, 0.0f, +0.95f);   // resting on side A
    drive(fsm, t, 600, 1.0f, 0.0f, -0.95f);   // resting flipped (side B) for >500ms
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_same_polarity_never_switches(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 2000, 1.0f, 0.0f, +0.95f);  // long rest on side A (e.g. sharpening)
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_flipped_but_moving_does_not_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    // Flipped polarity but moving (high gyro) the whole time -> not at rest.
    drive(fsm, t, 1500, 1.0f, 50.0f, -0.95f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
    // Comes to rest -> switches.
    drive(fsm, t, 600, 1.0f, 0.0f, -0.95f);
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_brief_flipped_settle_then_back_does_not_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 300, 1.0f, 0.0f, -0.95f);   // flipped+settled but < 500ms
    drive(fsm, t, 300, 1.0f, 0.0f, +0.95f);   // back to side-A pose
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_ambiguous_polarity_does_not_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    // Settled but gravity nearly perpendicular (|grav_dot_ref| < FLIP_POLARITY_MIN).
    drive(fsm, t, 1500, 1.0f, 0.0f, -0.1f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_switch_back_from_b_to_a(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 600, 1.0f, 0.0f, -0.95f);   // A -> B
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
    drive(fsm, t, 600, 1.0f, 0.0f, +0.95f);   // B -> A (positive polarity is "flipped" for B)
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_manual_toggle_switches_and_suppresses_auto(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, 0.0f, +0.95f);
    fsm.manual_toggle(t);
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
    TEST_ASSERT_TRUE(fsm.consume_switch());
    // Still physically in the side-A pose (grav +0.95), which is "flipped" for B,
    // but suppression must block auto-switch for 2s.
    drive(fsm, t, 1500, 1.0f, 0.0f, +0.95f);
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_suppression_expires_after_2s(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, 0.0f, +0.95f);
    fsm.manual_toggle(t);            // -> B, suppress auto for 2s
    fsm.consume_switch();
    drive(fsm, t, 2100, 1.0f, 0.0f, +0.95f);  // wait out suppression (settled, flipped-for-B)
    drive(fsm, t, 600,  1.0f, 0.0f, +0.95f);  // now 500ms+ settled-flipped -> switch back to A
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_on_side_a_no_events);
    RUN_TEST(test_settled_in_flipped_pose_switches);
    RUN_TEST(test_same_polarity_never_switches);
    RUN_TEST(test_flipped_but_moving_does_not_switch);
    RUN_TEST(test_brief_flipped_settle_then_back_does_not_switch);
    RUN_TEST(test_ambiguous_polarity_does_not_switch);
    RUN_TEST(test_switch_back_from_b_to_a);
    RUN_TEST(test_manual_toggle_switches_and_suppresses_auto);
    RUN_TEST(test_suppression_expires_after_2s);
    return UNITY_END();
}
