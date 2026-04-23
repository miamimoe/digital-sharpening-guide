#include "stroke.h"

void StrokeFSM::update(uint32_t now_ms, bool in_tolerance) {
    if (in_tolerance == sustained_) {
        // No contrary evidence — clear any pending attempt.
        contrary_pending_   = false;
        contrary_current_   = in_tolerance;
        return;
    }

    // Contrary evidence this sample.
    if (!contrary_pending_ || contrary_current_ != in_tolerance) {
        contrary_pending_    = true;
        contrary_started_ms_ = now_ms;
        contrary_current_    = in_tolerance;
    }

    // Note on uint32_t rollover at ~49 days: the subtraction below wraps
    // correctly by unsigned arithmetic rules, so the debounce window is
    // preserved across millis() rollover.
    uint32_t required = sustained_ ? OUT_MIN_MS : IN_MIN_MS;
    if (now_ms - contrary_started_ms_ >= required) {
        sustained_         = in_tolerance;
        contrary_pending_  = false;
        if (!sustained_) {
            count_++;
        }
    }
}

void StrokeFSM::reset() {
    sustained_           = false;
    contrary_pending_    = false;
    contrary_current_    = false;
    contrary_started_ms_ = 0;
    count_               = 0;
}
