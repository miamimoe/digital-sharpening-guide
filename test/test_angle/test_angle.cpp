#include <unity.h>
#include <cmath>
#include "angle.h"

void setUp(void) {}
void tearDown(void) {}

static void expect_near(float expected, float actual, float tol, const char* msg) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected, actual, msg);
}

void test_zero_deviation_is_zero_degrees(void) {
    Vec3 g = {0.0f, 0.0f, -1.0f};
    AngleResult r = compute_angle(g, g);
    expect_near(0.0f, r.degrees, 0.01f, "theta");
    TEST_ASSERT_EQUAL_INT(0, r.direction_sign);
}

void test_five_degree_increase_returns_red_sign(void) {
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r20 = 20.0f * (float)M_PI / 180.0f;
    Vec3 g_ref = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now = {std::cos(r20), 0.0f, -std::sin(r20)};
    AngleResult r = compute_angle(g_ref, g_now);
    expect_near(5.0f, r.degrees, 0.05f, "theta 5 deg");
    TEST_ASSERT_EQUAL_INT(+1, r.direction_sign);
}

void test_five_degree_decrease_returns_blue_sign(void) {
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r10 = 10.0f * (float)M_PI / 180.0f;
    Vec3 g_ref = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now = {std::cos(r10), 0.0f, -std::sin(r10)};
    AngleResult r = compute_angle(g_ref, g_now);
    expect_near(5.0f, r.degrees, 0.05f, "theta 5 deg");
    TEST_ASSERT_EQUAL_INT(-1, r.direction_sign);
}

void test_orientation_agnostic_rotation_around_n_back(void) {
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r20 = 20.0f * (float)M_PI / 180.0f;
    Vec3 g_ref_unrot = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now_unrot = {std::cos(r20), 0.0f, -std::sin(r20)};
    Vec3 g_ref_rot   = {0.0f, std::cos(r15), -std::sin(r15)};
    Vec3 g_now_rot   = {0.0f, std::cos(r20), -std::sin(r20)};
    AngleResult a = compute_angle(g_ref_unrot, g_now_unrot);
    AngleResult b = compute_angle(g_ref_rot,   g_now_rot);
    expect_near(a.degrees, b.degrees, 0.01f, "theta invariant under Z rot");
    TEST_ASSERT_EQUAL_INT(a.direction_sign, b.direction_sign);
}

void test_classify_within_tolerance_is_green(void) {
    AngleResult r = {1.5f, +1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::GREEN, (int)classify(r, 2.0f));
}

void test_classify_above_tolerance_with_positive_sign_is_red(void) {
    AngleResult r = {3.0f, +1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::RED, (int)classify(r, 2.0f));
}

void test_classify_above_tolerance_with_negative_sign_is_blue(void) {
    AngleResult r = {3.0f, -1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::BLUE, (int)classify(r, 2.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_deviation_is_zero_degrees);
    RUN_TEST(test_five_degree_increase_returns_red_sign);
    RUN_TEST(test_five_degree_decrease_returns_blue_sign);
    RUN_TEST(test_orientation_agnostic_rotation_around_n_back);
    RUN_TEST(test_classify_within_tolerance_is_green);
    RUN_TEST(test_classify_above_tolerance_with_positive_sign_is_red);
    RUN_TEST(test_classify_above_tolerance_with_negative_sign_is_blue);
    return UNITY_END();
}
