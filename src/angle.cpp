#include "angle.h"
#include <cmath>

static inline float dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

ColorState classify(float magnitude_deg, float target_deg, float tolerance_deg,
                    ColorState prev, float hyst_deg) {
    float low  = target_deg - tolerance_deg;
    float high = target_deg + tolerance_deg;

    // Widen the band we are currently in, so leaving it costs hyst_deg of real
    // movement rather than a noise sample. Each state is made sticky against the
    // boundary it would otherwise chatter across.
    if (hyst_deg > 0.0f) {
        switch (prev) {
            case ColorState::GREEN: low -= hyst_deg; high += hyst_deg; break;
            case ColorState::BLUE:  low += hyst_deg; break;   // must rise clear of low
            case ColorState::RED:   high -= hyst_deg; break;  // must fall clear of high
        }
    }

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

    // atan2(|a x b|, a.b) rather than acos(a.b): acos loses precision badly as the
    // dot product approaches +/-1, where float32 puts a resolution floor around
    // 0.04 deg. At a working bevel of 15-25 deg that is not a meaningful error,
    // but it matters for the near-flat cases (verifying the zero, checking a flat
    // reference) and it costs nothing. The sine term carries the precision where
    // the cosine term has none.
    Vec3 c   = cross(pf, pn);
    float s  = std::sqrt(dot(c, c));
    float deg = std::atan2(s, dot(pf, pn)) * (180.0f / (float)M_PI);   // 0..180
    // Fold the flipped blade face (gravity in the opposite hemisphere) into 0..90,
    // so one capture serves both sides.
    if (deg > 90.0f) deg = 180.0f - deg;
    return deg;
}
