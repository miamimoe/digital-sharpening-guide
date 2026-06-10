#pragma once
#include "types.h"

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
