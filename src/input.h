#pragma once
#include <cstdint>
#include "types.h"

class InputFSM {
public:
    static constexpr uint32_t DEBOUNCE_MS   = 30;
    static constexpr uint32_t LONG_PRESS_MS = 800;

    InputEvent update(uint32_t now_ms, bool a_pressed, bool b_pressed);

private:
    struct Button {
        bool     raw_current       = false;
        bool     stable            = false;
        uint32_t raw_changed_ms    = 0;
        uint32_t pressed_since_ms  = 0;
        bool     long_emitted      = false;
    };
    Button a_;
    Button b_;
    InputEvent step_button(uint32_t now_ms, Button& btn, bool raw, InputEvent on_short, InputEvent on_long);
};
