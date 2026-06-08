#include <unity.h>
#include "zero_cal.h"

void setUp(void) {}
void tearDown(void) {}

void test_still_sample_passes_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.0f};   // 1g magnitude
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};   // no rotation
    TEST_ASSERT_TRUE(zero_cal::is_still_instant(accel, gyro));
}

void test_high_gyro_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.0f};
    Vec3 gyro  = {0.0f, 0.0f, 10.0f};   // 10 dps > 8 dps threshold
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

void test_off_gravity_magnitude_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.1f};   // |a| - 1g = 0.1 > 0.05g
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

void test_below_gravity_magnitude_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -0.9f};   // |a| = 0.9g, deviation 0.1g > 0.05g
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

void test_zero_accel_vector_fails_gate(void) {
    // Zero-magnitude accel (e.g. IMU not yet producing samples) deviates 1g
    // from gravity — well outside the 0.01g threshold — so it correctly fails.
    Vec3 accel = {0.0f, 0.0f, 0.0f};
    Vec3 gyro  = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

static Vec3 still_accel = {0.0f, 0.0f, -1.0f};
static Vec3 still_gyro  = {0.0f, 0.0f,  0.0f};
static Vec3 jitter_accel = {0.0f, 0.0f, -1.1f}; // |a| = 1.1g, deviation 0.1g > 0.01g threshold

void test_capture_completes_after_warmup_and_averaging(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    // 500ms warmup at 50 Hz = 25 ticks; 1s averaging = 50 ticks (75 total).
    for (int i = 0; i < 100; ++i) {
        fsm.update(still_accel, still_gyro);
    }
    TEST_ASSERT_TRUE(fsm.done());
    Vec3 result = fsm.result();
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, result.z);
}

void test_jitter_during_warmup_restarts(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 15; ++i) fsm.update(still_accel, still_gyro);   // mid-warmup (<25)
    fsm.update(jitter_accel, still_gyro);                                // jitter -> restart
    for (int i = 0; i < 15; ++i) fsm.update(still_accel, still_gyro);
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_jitter_during_averaging_restarts(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 25; ++i) fsm.update(still_accel, still_gyro);   // complete warmup
    for (int i = 0; i < 20; ++i) fsm.update(still_accel, still_gyro);   // mid-averaging (<50)
    fsm.update(jitter_accel, still_gyro);                                // jitter -> restart
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_idle_update_is_noop(void) {
    zero_cal::CaptureFSM fsm;
    // Did NOT call start(); FSM is IDLE.
    for (int i = 0; i < 200; ++i) {
        fsm.update(still_accel, still_gyro);
    }
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::IDLE, fsm.phase());
}

void test_reset_clears_state(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 150; ++i) fsm.update(still_accel, still_gyro);
    TEST_ASSERT_TRUE(fsm.done());
    fsm.start();
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_still_sample_passes_gate);
    RUN_TEST(test_high_gyro_fails_gate);
    RUN_TEST(test_off_gravity_magnitude_fails_gate);
    RUN_TEST(test_below_gravity_magnitude_fails_gate);
    RUN_TEST(test_zero_accel_vector_fails_gate);
    RUN_TEST(test_capture_completes_after_warmup_and_averaging);
    RUN_TEST(test_jitter_during_warmup_restarts);
    RUN_TEST(test_jitter_during_averaging_restarts);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_idle_update_is_noop);
    return UNITY_END();
}
