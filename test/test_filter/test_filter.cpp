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
    for (int i = 0; i < 500; i++) {
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_stationary_gravity);
    RUN_TEST(test_tilts_toward_new_gravity_direction);
    RUN_TEST(test_gyro_bias_subtracted);
    return UNITY_END();
}
