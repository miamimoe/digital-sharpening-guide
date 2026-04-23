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

ColorState classify(AngleResult r, float tolerance_deg) {
    if (r.degrees <= tolerance_deg) return ColorState::GREEN;
    // Direction ambiguous (exactly-on-the-fence input): be conservative and stay GREEN
    // rather than flashing a misleading RED/BLUE correction direction.
    if (r.direction_sign == 0)      return ColorState::GREEN;
    if (r.direction_sign > 0)       return ColorState::RED;
    return ColorState::BLUE;
}
