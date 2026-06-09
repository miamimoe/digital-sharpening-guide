#pragma once
#include "types.h"

namespace feedback {
    void begin();
    void set_color(ColorState c);   // drives LED on only during RED
    void fault_led();               // solid on (for FAULT state)
    void beep_out_of_tolerance();   // buzzer gating must be done at caller
    void beep_confirm();            // short rising chirp (e.g. buzzer turned on)
}
