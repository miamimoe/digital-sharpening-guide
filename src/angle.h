#pragma once
#include "types.h"

struct AngleResult {
    float degrees;
    // Signed projection onto n_back: -1 below the reference tilt, +1 above, 0 at
    // the reference. NOT used by classify() (which is magnitude-vs-target); kept
    // for a possible future edge-axis/signed-pitch formulation.
    int   direction_sign;
};

// Returns the angle (degrees) between g_ref and g_now plus a direction sign.
// Inputs that are not already unit vectors are normalized internally.
// If either input has magnitude ~0 (e.g., from an IMU that hasn't produced a sample
// yet), returns {0.0f, 0} as a safe fallback rather than a garbage angle.
AngleResult compute_angle(Vec3 g_ref, Vec3 g_now);
// Decide GREEN/BLUE/RED from the angle magnitude vs target +/- tolerance.
// magnitude_deg is the (unsigned) sharpening angle = distance from the captured
// flat-on-stone zero, so it directly encodes high/low vs the target:
//   in  [target-tol, target+tol] => GREEN
//   <   target-tol               => BLUE (below target: raise the spine)
//   >   target+tol               => RED  (above target: lower the spine)
ColorState classify(float magnitude_deg, float target_deg, float tolerance_deg);

// --- Edge-axis (skew-corrected) bevel measurement ---

// The cutting-edge / hinge axis = unit(g_flat x g_raised), captured by laying the
// blade flat then raising the spine. Returns {0,0,0} if the raise was too small
// (cross product near zero) to define a reliable axis.
Vec3 compute_edge_axis(Vec3 g_flat, Vec3 g_raised);

// Bevel angle (degrees, 0..90): the tilt about the edge axis only, so lengthwise
// (tip-to-heel) skew does not inflate it. Both gravity vectors are projected onto
// the plane perpendicular to edge_axis; the angle between those projections is the
// bevel. The flipped blade face (gravity in the opposite hemisphere) is folded
// back, so a single capture works on both sides. Returns 0 on degenerate input.
float bevel_angle(Vec3 g_flat, Vec3 edge_axis, Vec3 g_now);
