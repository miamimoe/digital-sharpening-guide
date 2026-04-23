#include "stroke.h"

void StrokeFSM::update(uint32_t now_ms, bool in_tolerance) {
    if (in_tolerance == sustained_) {
        contrary_started_ms_ = 0;
        contrary_current_    = in_tolerance;
        return;
    }

    if (contrary_started_ms_ == 0 || contrary_current_ != in_tolerance) {
        contrary_started_ms_ = now_ms;
        contrary_current_    = in_tolerance;
    }

    uint32_t required = sustained_ ? OUT_MIN_MS : IN_MIN_MS;
    if (now_ms - contrary_started_ms_ >= required) {
        sustained_           = in_tolerance;
        contrary_started_ms_ = 0;
        if (!sustained_) {
            count_++;
        }
    }
}

void StrokeFSM::reset() {
    sustained_           = false;
    contrary_current_    = false;
    contrary_started_ms_ = 0;
    count_               = 0;
}
