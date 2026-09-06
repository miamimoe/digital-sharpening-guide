#include "stroke.h"

void StrokeFSM::update(uint32_t now_ms, bool in_tolerance, float lat_accel_g) {
    if (in_tolerance) {
        have_in_tol_    = true;
        last_in_tol_ms_ = now_ms;
    }
    // Only count while on-angle; a brief dip out of green mid-pass is tolerated.
    bool on_angle = in_tolerance ||
                    (have_in_tol_ && (now_ms - last_in_tol_ms_) < ON_ANGLE_GRACE_MS);
    if (!on_angle) {
        armed_ = false;
        return;
    }

    if (lat_accel_g < low_g_) {
        armed_ = true;                                   // settled between passes — re-arm
    } else if (lat_accel_g >= high_g_ && armed_ &&
               (now_ms - last_count_ms_) >= MIN_INTERVAL_MS) {
        count_++;                                        // rising edge of a pass
        armed_         = false;
        last_count_ms_ = now_ms;
    }
}

void StrokeFSM::reset() {
    armed_          = true;
    have_in_tol_    = false;
    last_count_ms_  = 0;
    last_in_tol_ms_ = 0;
    count_          = 0;
}

void StrokeFSM::set_thresholds(float high_g, float low_g) {
    if (high_g < 0.08f) high_g = 0.08f;
    if (low_g < 0.04f)  low_g  = 0.04f;
    if (low_g >= high_g) low_g = high_g * 0.55f;
    high_g_ = high_g;
    low_g_  = low_g;
}

void StrokeFSM::reset_thresholds() {
    high_g_ = PEAK_HIGH_G;
    low_g_  = PEAK_LOW_G;
}

void StrokeLearner::reset() {
    n_            = 0;
    armed_        = true;
    peak_lat_     = 0.0f;
    last_peak_ms_ = 0;
}

void StrokeLearner::update(uint32_t now_ms, float lat_accel_g) {
    if (lat_accel_g < DETECT_LOW_G) {
        if (!armed_ && peak_lat_ > 0.0f && n_ < MAX_PEAKS) {
            peaks_[n_++] = peak_lat_;
        }
        armed_    = true;
        peak_lat_ = 0.0f;
        return;
    }
    const bool interval_ok = (last_peak_ms_ == 0)
                          || ((now_ms - last_peak_ms_) >= MIN_INTERVAL_MS);
    if (lat_accel_g >= DETECT_HIGH_G && armed_ && interval_ok) {
        armed_        = false;
        last_peak_ms_ = now_ms;
        peak_lat_     = lat_accel_g;
    } else if (!armed_ && lat_accel_g > peak_lat_) {
        peak_lat_ = lat_accel_g;
    }
}

bool StrokeLearner::commit(StrokeFSM& fsm) const {
    if (n_ < MIN_PEAKS) {
        fsm.reset_thresholds();
        return false;
    }
    float tmp[MAX_PEAKS];
    for (int i = 0; i < n_; i++) tmp[i] = peaks_[i];
    for (int i = 1; i < n_; i++) {
        float v = tmp[i];
        int j = i;
        while (j > 0 && tmp[j - 1] > v) { tmp[j] = tmp[j - 1]; --j; }
        tmp[j] = v;
    }
    float median = tmp[n_ / 2];
    float high   = median * 0.70f;
    if (high < 0.10f) high = 0.10f;
    if (high > 0.45f) high = 0.45f;
    fsm.set_thresholds(high, high * 0.55f);
    return true;
}
