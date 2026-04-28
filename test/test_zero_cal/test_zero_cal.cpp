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
    Vec3 gyro  = {0.0f, 0.0f,  1.0f};   // 1 dps > 0.5 dps threshold
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

void test_off_gravity_magnitude_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.05f};  // |a| - 1g = 0.05 > 0.01g
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_still_sample_passes_gate);
    RUN_TEST(test_high_gyro_fails_gate);
    RUN_TEST(test_off_gravity_magnitude_fails_gate);
    return UNITY_END();
}
