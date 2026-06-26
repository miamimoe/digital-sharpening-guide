#include <unity.h>
#include "board.h"

void setUp(void) {}
void tearDown(void) {}

// Native build defines no SG_BOARD_* flag, so board.h must default to PLUS.
void test_native_default_is_plus(void) {
    TEST_ASSERT_TRUE(board::variant() == board::Variant::PLUS);
}

void test_plus_led_pin_is_10(void) {
    TEST_ASSERT_EQUAL_INT(10, board::led_pin());
}

void test_plus_has_axp192(void) {
    TEST_ASSERT_TRUE(board::has_axp192());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_native_default_is_plus);
    RUN_TEST(test_plus_led_pin_is_10);
    RUN_TEST(test_plus_has_axp192);
    return UNITY_END();
}
