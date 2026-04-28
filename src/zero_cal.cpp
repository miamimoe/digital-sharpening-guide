#include "zero_cal.h"
#include <cmath>

namespace zero_cal {

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps) {
    float a_mag = std::sqrt(accel_g.x*accel_g.x + accel_g.y*accel_g.y + accel_g.z*accel_g.z);
    if (std::fabs(a_mag - 1.0f) >= STILL_ACCEL_MAG_TOL_G) return false;
    float g_mag = std::sqrt(gyro_dps.x*gyro_dps.x + gyro_dps.y*gyro_dps.y + gyro_dps.z*gyro_dps.z);
    if (g_mag >= STILL_GYRO_MAG_DPS) return false;
    return true;
}

}  // namespace zero_cal
