#pragma once
#include "types.h"

struct AngleResult {
    float degrees;
    int   direction_sign; // -1 = angle decreased (BLUE), 0 = at ref, +1 = angle increased (RED)
};

// Compile-time axis pointing out the back of the device (into the blade).
// Set for MPU6886 body frame on M5StickC Plus PCB; validated by bring-up test in docs/superpowers/bringup/.
constexpr Vec3 N_BACK = {0.0f, 0.0f, -1.0f};

AngleResult compute_angle(Vec3 g_ref, Vec3 g_now);
ColorState  classify(AngleResult r, float tolerance_deg);
