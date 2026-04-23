#pragma once
#include <cstdint>
#include "types.h"

namespace power {
    struct IdleConfig {
        uint32_t dim_ms;     // 0 = never dim
        uint32_t sleep_ms;   // 0 = never sleep
    };

    IdleConfig config_for(State s);

    void begin();

    // Returns true if the caller should begin the sleep sequence.
    bool check_idle(uint32_t now_ms, State current,
                    uint32_t last_activity_ms,
                    uint32_t last_stroke_ms);

    void update_backlight(uint32_t now_ms, State current,
                          uint32_t last_activity_ms, uint32_t last_stroke_ms);

    [[noreturn]] void enter_deep_sleep();
}
