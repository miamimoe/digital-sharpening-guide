#include "imu.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include <cmath>

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

    // WHO_AM_I mismatch indicator: M5Unified's auto-detected type should be mpu6886
    // on an M5StickC Plus. Any other value means either a different board was
    // detected, or the documented AXP192/MPU6886 I²C conflict kept the IMU chip
    // from answering the type query.
    if (M5.Imu.getType() != m5::imu_mpu6886) {
        return FaultCode::E03_WHO_AM_I_MISMATCH;
    }
    return FaultCode::NONE;
}

bool read(Vec3& accel_g, Vec3& gyro_dps) {
    float ax, ay, az, gx, gy, gz;
    if (!M5.Imu.getAccel(&ax, &ay, &az)) return false;
    if (!M5.Imu.getGyro (&gx, &gy, &gz)) return false;
    accel_g  = {ax, ay, az};
    gyro_dps = {gx, gy, gz};
    return true;
}

bool capture_gyro_bias(Vec3& bias_out_dps) {
    constexpr int      SAMPLES          = 1000;    // 10 s @ 100 Hz
    constexpr float    STILLNESS_G      = 0.15f;   // |accel mag - 1| must stay under this
    constexpr uint32_t TICK_MS          = 10;
    constexpr uint32_t MAX_DURATION_MS  = 60000;   // give up after 60s of continuous motion
    double sx = 0, sy = 0, sz = 0;
    uint32_t start = millis();
    uint32_t last  = start;
    int n = 0;
    while (n < SAMPLES) {
        uint32_t now = millis();
        if (now - start > MAX_DURATION_MS) {
            // Device never settled; return failure so BIAS_CAL UX can retry or fall back.
            return false;
        }
        if (now - last < TICK_MS) { delay(1); continue; }
        last = now;
        Vec3 a, g;
        if (!read(a, g)) return false;
        float mag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
        if (fabsf(mag - 1.0f) > STILLNESS_G) {
            sx = sy = sz = 0;
            n  = 0;
            continue;
        }
        sx += g.x; sy += g.y; sz += g.z;
        n++;
    }
    if (n > 0) {
        bias_out_dps = {(float)(sx / n), (float)(sy / n), (float)(sz / n)};
    }
    return true;
}

} // namespace imu

#else
// Native stub — tests don't exercise IMU; keep shared code compilable.
namespace imu {
    FaultCode begin() { return FaultCode::NONE; }
    bool read(Vec3& a, Vec3& g) { a = {0,0,-1}; g = {0,0,0}; return true; }
    bool capture_gyro_bias(Vec3& out) { out = {0,0,0}; return true; }
}
#endif
