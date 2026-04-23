#include "angle.h"
#include <cmath>

static inline float dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

AngleResult compute_angle(Vec3 g_ref, Vec3 g_now) {
    float d = dot(g_ref, g_now);
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    float theta_rad = std::acos(d);
    float theta_deg = theta_rad * (180.0f / (float)M_PI);

    float alpha_ref = dot(g_ref, N_BACK);
    float alpha_now = dot(g_now, N_BACK);
    float delta     = alpha_now - alpha_ref;

    int sign = 0;
    const float eps = 1e-6f;
    if (delta >  eps) sign = +1;
    if (delta < -eps) sign = -1;

    return {theta_deg, sign};
}

ColorState classify(AngleResult r, float tolerance_deg) {
    if (r.degrees <= tolerance_deg) return ColorState::GREEN;
    if (r.direction_sign >= 0)      return ColorState::RED;
    return ColorState::BLUE;
}
