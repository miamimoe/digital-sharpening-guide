#include <unity.h>
#include "stroke.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(StrokeFSM& fsm, uint32_t& t, uint32_t dt_ms, bool in_tol) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        fsm.update(t, in_tol);
    }
}

void test_starts_at_zero_count(void) {
    StrokeFSM fsm;
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_canonical_single_stroke(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  false);
    drive(fsm, t, 600,  true);
    drive(fsm, t, 300,  false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_sub_threshold_in_blip_does_not_count(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200, false);
    drive(fsm, t, 400, true);   // < IN_MIN_MS (500) -> not a stroke
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_sub_200ms_out_wobble_does_not_end_stroke(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 600, true);
    drive(fsm, t, 100, false);
    drive(fsm, t, 200, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_three_back_to_back_strokes(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    for (int i = 0; i < 3; i++) {
        drive(fsm, t, 600, true);
        drive(fsm, t, 300, false);
    }
    TEST_ASSERT_EQUAL_UINT32(3, fsm.stroke_count());
}

void test_long_unbroken_in_counts_exactly_one_on_exit(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 4000, true);
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_reset_zeros_count(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 600, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
    fsm.reset();
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_first_ever_update_at_zero_ms_still_counts_one_stroke(void) {
    // Edge case: the old '== 0' sentinel could have collided with now_ms == 0
    // on the first-ever call. Verify the canonical stroke still counts
    // correctly when started from t = 0.
    StrokeFSM fsm;
    uint32_t t = 0;
    // 600 ms in-tolerance starting at t=0 (>= IN_MIN_MS 500)
    for (int i = 0; i < 60; i++) { t += 10; fsm.update(t, true); }
    // 300 ms out-of-tolerance
    for (int i = 0; i < 30; i++) { t += 10; fsm.update(t, false); }
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_timer_rollover_preserves_debounce_window(void) {
    // Arrange: start the FSM near UINT32_MAX so the 300 ms window spans the
    // rollover. Unsigned subtraction should still compute the correct elapsed.
    StrokeFSM fsm;
    uint32_t t = UINT32_MAX - 150; // 150 ms before rollover
    // Drive false for 200 ms to establish OUT
    for (int i = 0; i < 20; i++) { t += 10; fsm.update(t, false); }
    // 600 ms true, straddling rollover (first 150 ms before wrap, 450 ms after)
    for (int i = 0; i < 60; i++) { t += 10; fsm.update(t, true); }
    TEST_ASSERT_TRUE(fsm.is_in_tolerance());
    // Exit with 300 ms out to count the stroke
    for (int i = 0; i < 30; i++) { t += 10; fsm.update(t, false); }
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_zero_count);
    RUN_TEST(test_canonical_single_stroke);
    RUN_TEST(test_sub_threshold_in_blip_does_not_count);
    RUN_TEST(test_sub_200ms_out_wobble_does_not_end_stroke);
    RUN_TEST(test_three_back_to_back_strokes);
    RUN_TEST(test_long_unbroken_in_counts_exactly_one_on_exit);
    RUN_TEST(test_reset_zeros_count);
    RUN_TEST(test_first_ever_update_at_zero_ms_still_counts_one_stroke);
    RUN_TEST(test_timer_rollover_preserves_debounce_window);
    return UNITY_END();
}
