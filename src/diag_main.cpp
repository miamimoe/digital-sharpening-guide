// Temporary hardware bring-up diagnostic. Built ONLY by [env:diag] (which defines
// DIAG_BUILD and selects this file via build_src_filter). Excluded from the real
// firmware by the #ifdef guard. Prints board + IMU detection and live read status.
#ifdef DIAG_BUILD
#include <Arduino.h>
#include <M5Unified.h>

static const char* imu_type_name(m5::imu_t t) {
    switch (t) {
        case m5::imu_none:    return "none";
        case m5::imu_sh200q:  return "sh200q";
        case m5::imu_mpu6050: return "mpu6050";
        case m5::imu_mpu6886: return "mpu6886";
        case m5::imu_mpu9250: return "mpu9250";
        case m5::imu_bmi270:  return "bmi270";
        default:              return "other";
    }
}

// Mirror production read() exactly (imu.cpp): getAccel then getGyro, treat either
// false as a failure.
static bool prod_read(float* a, float* g) {
    if (!M5.Imu.getAccel(&a[0], &a[1], &a[2])) return false;
    if (!M5.Imu.getGyro (&g[0], &g[1], &g[2])) return false;
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(400);
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;          // mirror production
    M5.begin(cfg);
    setCpuFrequencyMhz(80);           // mirror production
    delay(200);

    Serial.println();
    Serial.println("===== IMU DIAG =====");
    Serial.printf("board id (m5::board_t)   : %d\n", (int)M5.getBoard());
    Serial.printf("is M5StickCPlus          : %d\n", (int)(M5.getBoard() == m5::board_t::board_M5StickCPlus));
    Serial.printf("imu.isEnabled()          : %d\n", (int)M5.Imu.isEnabled());
    Serial.printf("imu.getType()            : %d (%s)\n", (int)M5.Imu.getType(), imu_type_name(M5.Imu.getType()));
    Serial.println("--- 50 Hz production-mirror read() failure counters ---");
}

void loop() {
    // Run a full second of 50 Hz ticks like the real loop, counting read() failures.
    static uint32_t second = 0;
    uint32_t ticks = 0, fails = 0, accel_fail = 0, gyro_fail = 0;
    float a[3] = {0,0,0}, g[3] = {0,0,0};
    uint32_t next = millis();
    while (ticks < 50) {
        if ((int32_t)(millis() - next) < 0) { delay(1); continue; }
        next += 20;                   // 50 Hz
        M5.update();
        // Call both unconditionally, exactly like production imu::read(), so the
        // per-sensor failure counts are independent and faithful.
        bool a_ok = M5.Imu.getAccel(&a[0], &a[1], &a[2]);
        bool g_ok = M5.Imu.getGyro(&g[0], &g[1], &g[2]);
        if (!a_ok) ++accel_fail;
        if (!g_ok) ++gyro_fail;
        if (!(a_ok && g_ok)) ++fails;
        ++ticks;
    }
    (void)prod_read;
    Serial.printf("t=%us  ticks=%u  read_fail=%u (accel=%u gyro=%u)  last a=(% .2f % .2f % .2f)\n",
                  (unsigned)second, (unsigned)ticks, (unsigned)fails,
                  (unsigned)accel_fail, (unsigned)gyro_fail, a[0], a[1], a[2]);
    ++second;
}
#endif // DIAG_BUILD
