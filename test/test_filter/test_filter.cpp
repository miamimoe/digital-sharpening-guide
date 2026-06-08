#include <unity.h>
#include <cmath>
#include "filter.h"

void setUp(void) {}
void tearDown(void) {}

void test_converges_to_stationary_gravity(void) {
    MahonyFilter f;
    f.begin(100.0f);
    for (int i = 0; i < 500; i++) {
        f.update({0,0,0}, {0.0f, 0.0f, -1.0f});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, g.x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f,-1.0f, g.z);
}

void test_tilts_toward_new_gravity_direction(void) {
    MahonyFilter f;
    f.begin(100.0f);
    const float g30y = -0.5f;
    const float g30z = -std::sqrt(3.0f)/2.0f;
    for (int i = 0; i < 2000; i++) {
        f.update({0,0,0}, {0.0f, g30y, g30z});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.03f, g30y, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, g30z, g.z);
}

void test_gyro_bias_subtracted(void) {
    MahonyFilter f;
    f.begin(100.0f);
    f.set_bias({1.0f, 0.0f, 0.0f});
    for (int i = 0; i < 500; i++) {
        f.update({1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.02f,-1.0f, g.z);
}

// --- Snap-to-raw recovery (ported idea from evaEko/knife_whetting_level) ---

static void assert_nudge_aligns(Vec3 accel) {
    MahonyFilter f;
    f.begin(100.0f);
    f.nudge_to_gravity(accel);
    Vec3 g = f.gravity();
    float m = std::sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, accel.x/m, g.x);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, accel.y/m, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, accel.z/m, g.z);
}

void test_nudge_aligns_gravity_to_accel(void) {
    assert_nudge_aligns({0.0f,    0.0f,   -1.0f});        // flat, screen up
    assert_nudge_aligns({0.0f,   -0.5f,   -0.8660254f});  // 30° around X
    assert_nudge_aligns({0.3420f, 0.0f,   -0.9397f});     // 20° around Y
    assert_nudge_aligns({0.2f,   -0.3f,   -0.9f});         // arbitrary (renormalized)
    assert_nudge_aligns({0.0f,    0.0f,    1.0f});         // inverted — singular branch
}

void test_nudge_ignores_zero_magnitude(void) {
    // A degenerate accel sample must not corrupt the orientation.
    MahonyFilter f;
    f.begin(100.0f);
    for (int i = 0; i < 200; i++) f.update({0,0,0}, {0.0f, -0.5f, -0.8660254f});
    Vec3 before = f.gravity();
    f.nudge_to_gravity({0.0f, 0.0f, 0.0f});
    Vec3 after = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, before.x, after.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, before.y, after.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, before.z, after.z);
}

void test_should_snap_true_when_still_and_divergent(void) {
    // Filter still believes "flat"; device is actually tilted 20° and held still.
    TEST_ASSERT_TRUE(mahony::should_snap(
        {0.0f, 0.0f, -1.0f}, {0.3420f, 0.0f, -0.9397f}, {0.0f, 0.0f, 0.0f}));
}

void test_should_not_snap_when_moving(void) {
    // Same divergence, but the device is spinning — trust the filter, not raw accel.
    TEST_ASSERT_FALSE(mahony::should_snap(
        {0.0f, 0.0f, -1.0f}, {0.3420f, 0.0f, -0.9397f}, {50.0f, 0.0f, 0.0f}));
}

void test_should_not_snap_when_aligned(void) {
    // Only 3° apart — within normal jitter; don't fight the filter.
    TEST_ASSERT_FALSE(mahony::should_snap(
        {0.0f, 0.0f, -1.0f}, {0.0524f, 0.0f, -0.9986f}, {0.0f, 0.0f, 0.0f}));
}

void test_should_not_snap_during_linear_accel(void) {
    // Large divergence but |accel| far from 1g → not trustworthy gravity.
    TEST_ASSERT_FALSE(mahony::should_snap(
        {0.0f, 0.0f, -1.0f}, {0.6f, 0.0f, -1.3f}, {0.0f, 0.0f, 0.0f}));
}

void test_snap_does_not_repeat_on_steady_accel(void) {
    // After a snap aligns the filter to a still pose, should_snap must report
    // false on the next steady tick — otherwise it would re-fire every tick.
    MahonyFilter f;
    f.begin(100.0f);
    Vec3 accel = {0.3420f, 0.0f, -0.9397f};   // 20° tilt; filter starts at {0,0,-1}
    TEST_ASSERT_TRUE(mahony::should_snap(f.gravity(), accel, {0.0f, 0.0f, 0.0f}));
    f.nudge_to_gravity(accel);
    TEST_ASSERT_FALSE(mahony::should_snap(f.gravity(), accel, {0.0f, 0.0f, 0.0f}));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_stationary_gravity);
    RUN_TEST(test_tilts_toward_new_gravity_direction);
    RUN_TEST(test_gyro_bias_subtracted);
    RUN_TEST(test_nudge_aligns_gravity_to_accel);
    RUN_TEST(test_nudge_ignores_zero_magnitude);
    RUN_TEST(test_should_snap_true_when_still_and_divergent);
    RUN_TEST(test_should_not_snap_when_moving);
    RUN_TEST(test_should_not_snap_when_aligned);
    RUN_TEST(test_should_not_snap_during_linear_accel);
    RUN_TEST(test_snap_does_not_repeat_on_steady_accel);
    return UNITY_END();
}
