#pragma once
#include "types.h"

namespace imu {
    // Inspect the M5 IMU state (M5.begin must already have been called).
    // Returns NONE on success, else a FaultCode explaining what failed.
    FaultCode begin();

    // Read latest accel (g) + gyro (deg/s). Returns false on transient I/O failure.
    bool read(Vec3& accel_g, Vec3& gyro_dps);

    // Blocking 10-second stillness-gated gyro bias capture. Call only when
    // device is expected to be motionless. Averages gyro samples at 100 Hz.
    // Returns false if stillness criterion is violated during the window
    // (caller should restart the capture; BIAS_CAL state handles that UX).
    bool capture_gyro_bias(Vec3& bias_out_dps);
}
