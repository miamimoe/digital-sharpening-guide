#pragma once
#include "types.h"

namespace imu {
    // Inspect the M5 IMU state (M5.begin must already have been called).
    // Returns NONE on success, else a FaultCode explaining what failed.
    FaultCode begin();

    // Read latest accel (g) + gyro (deg/s). Returns false on transient I/O failure.
    bool read(Vec3& accel_g, Vec3& gyro_dps);
}
