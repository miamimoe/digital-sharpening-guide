#pragma once
#include "types.h"

struct AngleResult {
    float degrees;
    int   direction_sign; // -1 = angle decreased (BLUE), 0 = at ref, +1 = angle increased (RED)
};

// Compile-time axis pointing out the back of the device (into the blade).
// Set for MPU6886 body frame on M5StickC Plus PCB; validated by bring-up test in docs/superpowers/bringup/.
constexpr Vec3 N_BACK = {0.0f, 0.0f, -1.0f};

// Returns the angle (degrees) between g_ref and g_now plus a direction sign.
// Inputs that are not already unit vectors are normalized internally.
// If either input has magnitude ~0 (e.g., from an IMU that hasn't produced a sample
// yet), returns {0.0f, 0} as a safe fallback rather than a garbage angle.
AngleResult compute_angle(Vec3 g_ref, Vec3 g_now);
ColorState  classify(AngleResult r, float tolerance_deg);
