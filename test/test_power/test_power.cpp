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

// The reading that prompted the rewrite: a Plus on charge at 89 mA showed
// 3.78 V at the terminals and rested at 3.72 V. Both must land on the same
// answer, and that answer is "low", not the library's 52-60 %.
void test_ocv_removes_charge_and_load_offset(void) {
    TEST_ASSERT_INT_WITHIN(8, 3720, power::battery_ocv_mv(3780, 89));    // charging reads high
    TEST_ASSERT_EQUAL_INT(3720, power::battery_ocv_mv(3720, 0));         // resting: untouched
    TEST_ASSERT_EQUAL_INT(3780, power::battery_ocv_mv(3720, -100));      // under load reads low
}

void test_lipo_percent_follows_the_cell_curve(void) {
    TEST_ASSERT_EQUAL_INT(0,   power::lipo_percent(3000));
    TEST_ASSERT_EQUAL_INT(0,   power::lipo_percent(3300));
    TEST_ASSERT_EQUAL_INT(10,  power::lipo_percent(3690));
    TEST_ASSERT_INT_WITHIN(3, 18, power::lipo_percent(3720));   // the measured cell: one bar at most
    TEST_ASSERT_EQUAL_INT(50,  power::lipo_percent(3840));
    TEST_ASSERT_EQUAL_INT(80,  power::lipo_percent(4020));
    TEST_ASSERT_EQUAL_INT(100, power::lipo_percent(4150));
    TEST_ASSERT_EQUAL_INT(100, power::lipo_percent(4250));
}

void test_lipo_percent_never_goes_backwards(void) {
    int prev = 0;
    for (int mv = 3200; mv <= 4300; mv += 5) {
        int p = power::lipo_percent(mv);
        TEST_ASSERT_TRUE(p >= prev);
        TEST_ASSERT_TRUE(p >= 0 && p <= 100);
        prev = p;
    }
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
    RUN_TEST(test_ocv_removes_charge_and_load_offset);
    RUN_TEST(test_lipo_percent_follows_the_cell_curve);
    RUN_TEST(test_lipo_percent_never_goes_backwards);
    return UNITY_END();
}
