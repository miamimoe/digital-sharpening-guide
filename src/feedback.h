#pragma once
#include "types.h"

namespace feedback {
    void begin();
    void set_color(ColorState c);   // drives LED on only during RED
    void fault_led();               // solid on (for FAULT state)
    void beep_out_of_tolerance();   // buzzer gating must be done at caller
    void beep_confirm();            // short rising chirp (e.g. buzzer turned on)
#ifdef UNIT_TEST
    // Test-only: the native stub counts beep_out_of_tolerance() calls so tests
    // can assert on beep edges. Not compiled into firmware.
    int  test_beep_out_count();
    void test_reset_beep_count();
#endif
}
