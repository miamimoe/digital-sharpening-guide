#include "imu.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include "board.h"

namespace imu {

FaultCode begin() {
    // Assumes M5.begin(cfg) was already invoked with cfg.internal_imu = true.
    if (!M5.Imu.isEnabled()) {
        return FaultCode::E01_BEGIN_FAILED;
    }

    // Sanity-read.
    Vec3 a, g;
    bool ok = false;
    for (int i = 0; i < 10; i++) {
        if (read(a, g)) { ok = true; break; }
        delay(5);
    }
    if (!ok) return FaultCode::E02_SELF_TEST_FAILED;

    // The expected auto-detected IMU is board-specific: MPU6886 on Plus/Plus2,
    // BMI270 on the S3. A mismatch means a different board was detected, the wrong
    // firmware was flashed, or (Plus only) the AXP192/MPU6886 I²C conflict kept the
    // IMU from answering the type query.
    m5::imu_t got = M5.Imu.getType();
    bool type_ok = (board::variant() == board::Variant::S3)
                       ? (got == m5::imu_bmi270)
                       : (got == m5::imu_mpu6886);
    if (!type_ok) {
        return FaultCode::E03_WHO_AM_I_MISMATCH;
    }
    return FaultCode::NONE;
}

bool read(Vec3& accel_g, Vec3& gyro_dps) {
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    // In M5Unified's default (non-FIFO) mode, getAccel/getGyro return false simply
    // when no fresh sample is ready (MPU6886 data-ready bit clear) — a normal,
    // frequent condition at 50 Hz, not an error. They still write the most recent
    // cached sample into the outputs, so always capture it and report freshness
    // via the return. Callers must treat false as "no new data this instant" and
    // only escalate to a fault after a sustained run of failures.
    bool a_ok = M5.Imu.getAccel(&ax, &ay, &az);
    bool g_ok = M5.Imu.getGyro (&gx, &gy, &gz);
    accel_g  = {ax, ay, az};
    gyro_dps = {gx, gy, gz};
    return a_ok && g_ok;
}

} // namespace imu

#else
// Native stub — tests don't exercise IMU; keep shared code compilable.
namespace imu {
    FaultCode begin() { return FaultCode::NONE; }
    bool read(Vec3& a, Vec3& g) { a = {0,0,-1}; g = {0,0,0}; return true; }
}
#endif
