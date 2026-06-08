#include <unity.h>
#include "stroke.h"

void setUp(void) {}
void tearDown(void) {}

// Drive the FSM for dt_ms at a fixed on-angle state and horizontal-accel level.
static void drive(StrokeFSM& fsm, uint32_t& t, uint32_t dt_ms, bool in_tol, float lat) {
    uint32_t end = t + dt_ms;
    while (t < end) { t += 10; fsm.update(t, in_tol, lat); }
}

void test_starts_at_zero(void) {
    StrokeFSM fsm;
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_counts_one_pass(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true, 0.0f);    // on-angle, settled
    drive(fsm, t, 100, true, 0.25f);   // one pass (accel peak)
    drive(fsm, t, 100, true, 0.0f);    // settle
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_counts_multiple_continuous_passes(void) {
    // The key case: continuous back-and-forth passes while staying on-angle.
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true, 0.0f);
    for (int i = 0; i < 5; i++) {
        drive(fsm, t, 80,  true, 0.25f);   // pass
        drive(fsm, t, 320, true, 0.0f);    // settle between passes
    }
    TEST_ASSERT_EQUAL_UINT32(5, fsm.stroke_count());
}

void test_close_peaks_merge_via_interval(void) {
    // Two accel humps within MIN_INTERVAL_MS (one pass's accel + decel) count once.
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true, 0.0f);
    drive(fsm, t,  50, true, 0.25f);   // peak1 -> count 1 (~t=410)
    drive(fsm, t, 100, true, 0.0f);    // brief dip (re-arm)
    drive(fsm, t,  50, true, 0.25f);   // peak2 ~t=520, < 350ms since count -> ignored
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_no_count_off_angle(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 1000, false, 0.25f);   // never on-angle: motion ignored
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_sustained_high_without_dip_counts_once(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true, 0.0f);
    drive(fsm, t, 1000, true, 0.25f);    // stays high, never dips below re-arm
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_grace_allows_brief_off_angle_mid_pass(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true,  0.0f);     // establish on-angle
    drive(fsm, t,  50, true,  0.25f);    // pass 1 -> count 1 (~410)
    drive(fsm, t, 100, true,  0.0f);     // settle (last on-angle ~550)
    drive(fsm, t, 300, false, 0.0f);     // brief off-angle, still within grace
    drive(fsm, t,  50, false, 0.25f);    // pass ~910 (>350 since count, within 600ms grace) -> count 2
    TEST_ASSERT_EQUAL_UINT32(2, fsm.stroke_count());
}

void test_reset_zeros(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 400, true, 0.0f);
    drive(fsm, t,  80, true, 0.25f);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
    fsm.reset();
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_zero);
    RUN_TEST(test_counts_one_pass);
    RUN_TEST(test_counts_multiple_continuous_passes);
    RUN_TEST(test_close_peaks_merge_via_interval);
    RUN_TEST(test_no_count_off_angle);
    RUN_TEST(test_sustained_high_without_dip_counts_once);
    RUN_TEST(test_grace_allows_brief_off_angle_mid_pass);
    RUN_TEST(test_reset_zeros);
    return UNITY_END();
}
