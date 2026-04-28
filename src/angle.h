#pragma once
#include "types.h"

struct AngleResult {
    float degrees;
    int   direction_sign; // -1 = angle decreased (BLUE), 0 = at ref, +1 = angle increased (RED)
};

// Returns the angle (degrees) between g_ref and g_now plus a direction sign.
// Inputs that are not already unit vectors are normalized internally.
// If either input has magnitude ~0 (e.g., from an IMU that hasn't produced a sample
// yet), returns {0.0f, 0} as a safe fallback rather than a garbage angle.
AngleResult compute_angle(Vec3 g_ref, Vec3 g_now);
// Decide GREEN/BLUE/RED given the angle magnitude vs target ± tolerance.
// direction_sign disambiguates BLUE vs RED when out of tolerance.
// direction_sign == 0 => conservative GREEN fallback.
ColorState classify(float magnitude_deg,
                    float target_deg,
                    float tolerance_deg,
                    int   direction_sign);
