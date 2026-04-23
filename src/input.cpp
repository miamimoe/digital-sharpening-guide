#include "input.h"

InputEvent InputFSM::step_button(uint32_t now_ms, Button& btn, bool raw,
                                  InputEvent on_short, InputEvent on_long)
{
    if (raw != btn.raw_current) {
        btn.raw_current    = raw;
        btn.raw_changed_ms = now_ms;
    }
    if (btn.raw_current != btn.stable && (now_ms - btn.raw_changed_ms) >= DEBOUNCE_MS) {
        btn.stable = btn.raw_current;
        if (btn.stable) {
            btn.pressed_since_ms = now_ms;
            btn.long_emitted     = false;
        } else {
            if (!btn.long_emitted) {
                return on_short;
            }
        }
    }
    if (btn.stable && !btn.long_emitted && (now_ms - btn.pressed_since_ms) >= LONG_PRESS_MS) {
        btn.long_emitted = true;
        return on_long;
    }
    return InputEvent::NONE;
}

InputEvent InputFSM::update(uint32_t now_ms, bool a_pressed, bool b_pressed) {
    InputEvent ea = step_button(now_ms, a_, a_pressed, InputEvent::A_SHORT, InputEvent::A_LONG);
    InputEvent eb = step_button(now_ms, b_, b_pressed, InputEvent::B_SHORT, InputEvent::B_LONG);
    if (ea != InputEvent::NONE) return ea;
    return eb;
}
