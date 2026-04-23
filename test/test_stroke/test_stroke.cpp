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
    drive(fsm, t, 500,  true);
    drive(fsm, t, 300,  false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_sub_300ms_in_blip_does_not_count(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200, false);
    drive(fsm, t, 200, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_sub_200ms_out_wobble_does_not_end_stroke(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 500, true);
    drive(fsm, t, 100, false);
    drive(fsm, t, 200, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_three_back_to_back_strokes(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    for (int i = 0; i < 3; i++) {
        drive(fsm, t, 500, true);
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
    drive(fsm, t, 500, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
    fsm.reset();
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_zero_count);
    RUN_TEST(test_canonical_single_stroke);
    RUN_TEST(test_sub_300ms_in_blip_does_not_count);
    RUN_TEST(test_sub_200ms_out_wobble_does_not_end_stroke);
    RUN_TEST(test_three_back_to_back_strokes);
    RUN_TEST(test_long_unbroken_in_counts_exactly_one_on_exit);
    RUN_TEST(test_reset_zeros_count);
    return UNITY_END();
}
