#pragma once
#include "types.h"

namespace feedback {
    void begin();
    void set_color(ColorState c);   // drives LED on only during RED
    void fault_led();               // solid on (for FAULT state)
    void beep_out_of_tolerance();   // buzzer gating must be done at caller
    void tick(uint32_t now_ms);     // service in-progress beep durations
}
