#include "angle.h"
#include <cmath>

// Axis pointing out the back of the device (into the blade). MPU6886 body frame on M5StickC Plus PCB.
// Sign validated by hardware bring-up step 3; flip to {0,0,1} if gravity Z reads +1g when screen-up.
static constexpr Vec3 N_BACK = {0.0f, 0.0f, -1.0f};

static inline float dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

AngleResult compute_angle(Vec3 g_ref, Vec3 g_now) {
    // Guard against near-zero-magnitude inputs (e.g., IMU not yet producing samples).
    constexpr float MIN_MAG_SQ = 0.25f;           // rejects vectors shorter than 0.5
    float ref_mag_sq = dot(g_ref, g_ref);
    float now_mag_sq = dot(g_now, g_now);
    if (ref_mag_sq < MIN_MAG_SQ || now_mag_sq < MIN_MAG_SQ) {
        return {0.0f, 0};
    }

    // Normalize defensively.
    float ref_inv_mag = 1.0f / std::sqrt(ref_mag_sq);
    float now_inv_mag = 1.0f / std::sqrt(now_mag_sq);
    Vec3 u_ref = { g_ref.x * ref_inv_mag, g_ref.y * ref_inv_mag, g_ref.z * ref_inv_mag };
    Vec3 u_now = { g_now.x * now_inv_mag, g_now.y * now_inv_mag, g_now.z * now_inv_mag };

    // Magnitude of deviation (unsigned, degrees).
    float d = dot(u_ref, u_now);
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    float theta_rad = std::acos(d);
    float theta_deg = theta_rad * (180.0f / (float)M_PI);

    // Direction signal: signed projection onto n_back = sin(sharpening angle).
    float alpha_ref = dot(u_ref, N_BACK);
    float alpha_now = dot(u_now, N_BACK);
    float delta     = alpha_now - alpha_ref;

    int sign = 0;
    const float eps = 1e-6f;
    if (delta >  eps) sign = +1;
    if (delta < -eps) sign = -1;

    return {theta_deg, sign};
}

ColorState classify(float magnitude_deg, float target_deg, float tolerance_deg) {
    float low  = target_deg - tolerance_deg;
    float high = target_deg + tolerance_deg;
    if (magnitude_deg < low)  return ColorState::BLUE;   // below target: raise spine
    if (magnitude_deg > high) return ColorState::RED;    // above target: lower spine
    return ColorState::GREEN;
}

static inline Vec3 cross(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}

static inline Vec3 scale(Vec3 v, float s) { return { v.x*s, v.y*s, v.z*s }; }
static inline Vec3 sub(Vec3 a, Vec3 b)    { return { a.x-b.x, a.y-b.y, a.z-b.z }; }

// Normalize; returns false (and leaves out untouched) if magnitude is too small.
static inline bool try_unit(Vec3 v, Vec3& out) {
    float m2 = dot(v, v);
    if (m2 < 1e-12f) return false;
    float inv = 1.0f / std::sqrt(m2);
    out = { v.x*inv, v.y*inv, v.z*inv };
    return true;
}

Vec3 compute_edge_axis(Vec3 g_flat, Vec3 g_raised) {
    Vec3 axis;
    // |g_flat x g_raised| ~ sin(raise angle); too small => raise was insufficient.
    Vec3 c = cross(g_flat, g_raised);
    if (!try_unit(c, axis)) return {0.0f, 0.0f, 0.0f};
    return axis;
}

float bevel_angle(Vec3 g_flat, Vec3 edge_axis, Vec3 g_now) {
    Vec3 f, n;
    if (!try_unit(g_flat, f) || !try_unit(g_now, n)) return 0.0f;

    Vec3 pf, pn, e;
    if (try_unit(edge_axis, e)
        && try_unit(sub(f, scale(e, dot(f, e))), pf)
        && try_unit(sub(n, scale(e, dot(n, e))), pn)) {
        // Edge-axis path: project both onto the plane perpendicular to the edge,
        // isolating the bevel rotation from lengthwise skew.
    } else {
        // Degenerate edge axis (raise was too small) — fall back to total tilt
        // (cone angle from flat). Same fold; just not skew-corrected.
        pf = f;
        pn = n;
    }

    float d = dot(pf, pn);
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    float deg = std::acos(d) * (180.0f / (float)M_PI);   // 0..180
    // Fold the flipped blade face (gravity in the opposite hemisphere) into 0..90,
    // so one capture serves both sides.
    if (deg > 90.0f) deg = 180.0f - deg;
    return deg;
}
