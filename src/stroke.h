#pragma once
#include <cstdint>

class StrokeFSM {
public:
    // Sensitivity knob: how long the angle must be held in-tolerance before it
    // counts as a real stroke. Raised from 300ms after hardware testing showed
    // brief incidental passes through the green zone were over-counting. Bump
    // higher to be stricter (fewer false counts), lower to catch faster strokes.
    static constexpr uint32_t IN_MIN_MS  = 500;
    static constexpr uint32_t OUT_MIN_MS = 200;

    void      update(uint32_t now_ms, bool in_tolerance);
    uint32_t  stroke_count() const { return count_; }
    bool      is_in_tolerance() const { return sustained_; }
    void      reset();

private:
    bool     sustained_             = false;
    bool     contrary_pending_      = false;  // explicit flag replacing "== 0" sentinel
    bool     contrary_current_      = false;
    uint32_t contrary_started_ms_   = 0;
    uint32_t count_                 = 0;
};
