#include <unity.h>
#include "power.h"

void setUp(void) {}
void tearDown(void) {}

// Fresh quantization (no previous value): 5 bands of 20 %.
void test_bars_fresh_bands(void) {
    TEST_ASSERT_EQUAL_INT(0, power::battery_bars(0,  -1));
    TEST_ASSERT_EQUAL_INT(0, power::battery_bars(19, -1));
    TEST_ASSERT_EQUAL_INT(1, power::battery_bars(20, -1));
    TEST_ASSERT_EQUAL_INT(1, power::battery_bars(39, -1));
    TEST_ASSERT_EQUAL_INT(2, power::battery_bars(40, -1));
    TEST_ASSERT_EQUAL_INT(3, power::battery_bars(60, -1));
    TEST_ASSERT_EQUAL_INT(4, power::battery_bars(80, -1));
    TEST_ASSERT_EQUAL_INT(4, power::battery_bars(100, -1));
}

// Unknown / error readings (negative from M5Unified) report -1 so the UI can
// draw an "unknown" icon rather than an empty battery.
void test_bars_unknown_passthrough(void) {
    TEST_ASSERT_EQUAL_INT(-1, power::battery_bars(-1, -1));
    TEST_ASSERT_EQUAL_INT(-1, power::battery_bars(-2,  3));
}

// Out-of-range percentages are clamped rather than producing a 5th bar.
void test_bars_clamps_over_100(void) {
    TEST_ASSERT_EQUAL_INT(4, power::battery_bars(150, -1));
}

// Hysteresis: once at a level, a reading that drifts a few percent across the
// boundary keeps the previous bar count so the icon does not flicker.
void test_bars_hysteresis_holds_near_boundary(void) {
    TEST_ASSERT_EQUAL_INT(2, power::battery_bars(38, 2));   // 2 px below 40, keep 2
    TEST_ASSERT_EQUAL_INT(2, power::battery_bars(61, 2));   // 1 px above 60, keep 2
    TEST_ASSERT_EQUAL_INT(1, power::battery_bars(36, 2));   // past the band -> drop
    TEST_ASSERT_EQUAL_INT(3, power::battery_bars(64, 2));   // past the band -> rise
}

// A big jump (e.g. after a charge) is never held back by hysteresis.
void test_bars_big_jump_not_held(void) {
    TEST_ASSERT_EQUAL_INT(4, power::battery_bars(95, 0));
    TEST_ASSERT_EQUAL_INT(0, power::battery_bars(5, 4));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_bars_fresh_bands);
    RUN_TEST(test_bars_unknown_passthrough);
    RUN_TEST(test_bars_clamps_over_100);
    RUN_TEST(test_bars_hysteresis_holds_near_boundary);
    RUN_TEST(test_bars_big_jump_not_held);
    return UNITY_END();
}
