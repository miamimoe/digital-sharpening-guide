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

// Boot-time ADC race: the AXP192's VBAT ADC samples at 25 Hz and is enabled
// only in M5.begin(), so the first poll often reads 0 mV — which M5Unified
// clamps to a "valid" 0 % instead of an error. The gate must turn any
// physically impossible VBAT into -1 (unknown) so the UI draws the dash, not
// an empty battery.
void test_gate_rejects_implausible_vbat(void) {
    TEST_ASSERT_EQUAL_INT(-1, power::battery_level_gate(0,   0));     // ADC not ready
    TEST_ASSERT_EQUAL_INT(-1, power::battery_level_gate(100, 2499));  // below any LiPo
    TEST_ASSERT_EQUAL_INT(-1, power::battery_level_gate(50,  -1));    // M5PM1 "undetermined"
    TEST_ASSERT_EQUAL_INT(-1, power::battery_level_gate(50,   0));    // M5PM1 "no battery"
}

// With a plausible VBAT behind it, the percentage passes through untouched —
// including a genuine 0 % (flat cell at ~3.3 V) and M5Unified error codes.
void test_gate_passes_plausible_readings(void) {
    TEST_ASSERT_EQUAL_INT(85, power::battery_level_gate(85, 4100));
    TEST_ASSERT_EQUAL_INT(0,  power::battery_level_gate(0,  3300));   // honestly flat
    TEST_ASSERT_EQUAL_INT(7,  power::battery_level_gate(7,  2500));   // boundary is inclusive
    TEST_ASSERT_EQUAL_INT(-2, power::battery_level_gate(-2, 4100));   // error code passthrough
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_bars_fresh_bands);
    RUN_TEST(test_bars_unknown_passthrough);
    RUN_TEST(test_bars_clamps_over_100);
    RUN_TEST(test_bars_hysteresis_holds_near_boundary);
    RUN_TEST(test_bars_big_jump_not_held);
    RUN_TEST(test_gate_rejects_implausible_vbat);
    RUN_TEST(test_gate_passes_plausible_readings);
    return UNITY_END();
}
