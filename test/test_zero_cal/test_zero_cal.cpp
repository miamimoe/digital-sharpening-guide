#include <unity.h>
#include "zero_cal.h"

void setUp(void) {}
void tearDown(void) {}

static Vec3 still_accel  = {0.0f, 0.0f, -1.0f};
static Vec3 still_gyro   = {0.0f, 0.0f,  0.0f};
// Magnitude jitter: |a| = 1.3g, deviation 0.3g > MAG_TOL.
static Vec3 mag_jitter   = {0.0f, 0.0f, -1.3f};
// Direction jitter: |a| ~ 1g but rotated ~30deg from the {0,0,-1} anchor
// (|a - anchor| ~ 0.52g > DRIFT_TOL) — caught WITHOUT any gyro.
static Vec3 drift_jitter = {0.5f, 0.0f, -0.8660254f};

void test_capture_completes_after_warmup_and_averaging(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 100; ++i) fsm.update(still_accel, still_gyro);
    TEST_ASSERT_TRUE(fsm.done());
    Vec3 result = fsm.result();
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, result.z);
}

void test_magnitude_jitter_restarts(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 15; ++i) fsm.update(still_accel, still_gyro);
    fsm.update(mag_jitter, still_gyro);                       // -> restart
    for (int i = 0; i < 15; ++i) fsm.update(still_accel, still_gyro);
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_direction_drift_restarts_without_gyro(void) {
    // The whole point of the new gate: a rotation (direction drift) is caught by
    // the accelerometer alone, with the gyro reading ZERO the entire time.
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 30; ++i) fsm.update(still_accel, still_gyro);  // into averaging
    fsm.update(drift_jitter, still_gyro);                              // rotated -> restart
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_hand_tremor_gyro_does_not_block_capture(void) {
    // Regression: large gyro (hand tremor velocity) must NOT stall the capture as
    // long as the accel direction stays put. Feed a steady accel with 30 dps gyro.
    zero_cal::CaptureFSM fsm;
    fsm.start();
    Vec3 tremor_gyro = {30.0f, -25.0f, 20.0f};
    for (int i = 0; i < 100; ++i) fsm.update(still_accel, tremor_gyro);
    TEST_ASSERT_TRUE(fsm.done());
}

void test_moving_flag_tracks_stillness(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    fsm.update(still_accel, still_gyro);
    TEST_ASSERT_FALSE(fsm.moving());
    fsm.update(drift_jitter, still_gyro);
    TEST_ASSERT_TRUE(fsm.moving());
}

void test_gyro_bias_is_mean_over_window(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    Vec3 bias = {1.0f, 2.0f, -3.0f};
    for (int i = 0; i < 100; ++i) fsm.update(still_accel, bias);
    TEST_ASSERT_TRUE(fsm.done());
    Vec3 gb = fsm.gyro_bias();
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  1.0f, gb.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  2.0f, gb.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -3.0f, gb.z);
}

void test_idle_update_is_noop(void) {
    zero_cal::CaptureFSM fsm;
    for (int i = 0; i < 200; ++i) fsm.update(still_accel, still_gyro);
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
    RUN_TEST(test_capture_completes_after_warmup_and_averaging);
    RUN_TEST(test_magnitude_jitter_restarts);
    RUN_TEST(test_direction_drift_restarts_without_gyro);
    RUN_TEST(test_hand_tremor_gyro_does_not_block_capture);
    RUN_TEST(test_moving_flag_tracks_stillness);
    RUN_TEST(test_gyro_bias_is_mean_over_window);
    RUN_TEST(test_idle_update_is_noop);
    RUN_TEST(test_reset_clears_state);
    return UNITY_END();
}
