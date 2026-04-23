#include <unity.h>
#include "input.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(InputFSM& fsm, uint32_t& t, uint32_t dt_ms, bool a, bool b, InputEvent expected_last) {
    InputEvent last = InputEvent::NONE;
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        InputEvent ev = fsm.update(t, a, b);
        if (ev != InputEvent::NONE) last = ev;
    }
    TEST_ASSERT_EQUAL_INT((int)expected_last, (int)last);
}

void test_short_press_a_on_release(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,  false, false, InputEvent::NONE);
    drive(fsm, t, 300, true,  false, InputEvent::NONE);
    drive(fsm, t, 50,  false, false, InputEvent::A_SHORT);
}

void test_long_press_a_at_800ms_while_held(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
    drive(fsm, t, 900,  true,  false, InputEvent::A_LONG);
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
}

void test_short_press_b(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,  false, false, InputEvent::NONE);
    drive(fsm, t, 200, false, true,  InputEvent::NONE);
    drive(fsm, t, 50,  false, false, InputEvent::B_SHORT);
}

void test_long_press_b(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
    drive(fsm, t, 900,  false, true,  InputEvent::B_LONG);
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
}

void test_debounces_spurious_short_blip(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50, false, false, InputEvent::NONE);
    drive(fsm, t, 20, true,  false, InputEvent::NONE);
    drive(fsm, t, 80, false, false, InputEvent::NONE);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_short_press_a_on_release);
    RUN_TEST(test_long_press_a_at_800ms_while_held);
    RUN_TEST(test_short_press_b);
    RUN_TEST(test_long_press_b);
    RUN_TEST(test_debounces_spurious_short_blip);
    return UNITY_END();
}
