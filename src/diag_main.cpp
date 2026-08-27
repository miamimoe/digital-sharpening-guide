// Hardware diagnostic. Built ONLY by [env:diag] (which defines DIAG_BUILD and
// selects this file via build_src_filter). Excluded from the real firmware by the
// #ifdef guard.
//
// Two jobs:
//
//  1. Board + IMU detection, and the 50 Hz read()-failure counters that the
//     original bring-up used.
//
//  2. STEADY MODE A/B. Runs two MahonyFilters side by side on the SAME IMU
//     stream — one with the accel-reference low-pass (steady mode) and one
//     without (legacy) — and reports how much each moves while you hold an
//     angle. Since both see identical input, the difference between them IS the
//     filter change, measured on your hardware instead of argued from theory.
//
//     Also reports what a per-tick oversample would buy (see SAMPLE HEADROOM
//     below), which is the open question for the anti-alias work.
//
// Usage: flash the environment that MATCHES YOUR BOARD —
//   [env:diag]        M5StickC Plus   (MPU6886)
//   [env:diag-plus2]  M5StickC Plus2  (MPU6886)
//   [env:diag-s3]     M5StickS3       (BMI270)
// then open the serial monitor at 115200, hold the device still at your angle
// for the reference capture, and sharpen normally while reading the columns.
// Flashing the Plus build onto an S3 (or the reverse) trips the WRONG FIRMWARE
// guard in the real firmware, but this diagnostic has no such guard — so pick
// the right one.
#ifdef DIAG_BUILD
#include <Arduino.h>
#include <M5Unified.h>
#include <cmath>

#include "filter.h"
#include "types.h"

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

// Rolling window statistics for one filter's reported angle.
struct Stats {
    int   n    = 0;
    float sum  = 0.0f;
    float sq   = 0.0f;
    float lo   =  1e9f;
    float hi   = -1e9f;

    void add(float v) {
        ++n; sum += v; sq += v * v;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    float mean() const { return n ? sum / n : 0.0f; }
    float sd() const {
        if (n < 2) return 0.0f;
        float m = mean();
        float var = sq / n - m * m;
        return var > 0.0f ? std::sqrt(var) : 0.0f;
    }
    float pp() const { return (n && hi >= lo) ? hi - lo : 0.0f; }
    void reset() { *this = Stats{}; }
};

static MahonyFilter f_raw;      // legacy path: accel used as-is
static MahonyFilter f_steady;   // steady mode: accel low-passed first
static Vec3  g_ref = {0.0f, 0.0f, -1.0f};   // "hold this angle" reference
static Stats s_raw, s_steady;
static Stats s_drift;           // |single sample - oversampled mean|, in milli-g

// Angle between two vectors, degrees. atan2 form: well conditioned at both ends.
static float angle_between_deg(Vec3 a, Vec3 b) {
    float d  = a.x*b.x + a.y*b.y + a.z*b.z;
    Vec3  c  = { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
    float cm = std::sqrt(c.x*c.x + c.y*c.y + c.z*c.z);
    return std::atan2(cm, d) * 180.0f / (float)M_PI;
}

static bool read_one(Vec3& a, Vec3& g) {
    float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    bool a_ok = M5.Imu.getAccel(&ax, &ay, &az);
    bool g_ok = M5.Imu.getGyro (&gx, &gy, &gz);
    a = {ax, ay, az};
    g = {gx, gy, gz};
    return a_ok && g_ok;
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
    Serial.println("===== SHARPENING GUIDE DIAG =====");
    Serial.printf("board id (m5::board_t)   : %d\n", (int)M5.getBoard());
    Serial.printf("imu.isEnabled()          : %d\n", (int)M5.Imu.isEnabled());
    Serial.printf("imu.getType()            : %d (%s)\n",
                  (int)M5.Imu.getType(), imu_type_name(M5.Imu.getType()));

    f_raw.begin(50.0f, 0.8f, 0.02f);
    f_raw.set_accel_tau(0.0f);
    f_steady.begin(50.0f, 0.8f, 0.02f);
    f_steady.set_accel_tau(mahony::ACCEL_LP_TAU_S);
    Serial.printf("steady accel tau         : %.2f s\n", (double)mahony::ACCEL_LP_TAU_S);

    Serial.println();
    Serial.println(">>> Hold the device STILL at your sharpening angle for 2 s...");
    Vec3 acc{}, gyr{}, sum{0,0,0};
    int n = 0;
    uint32_t next = millis();
    for (int i = 0; i < 100; i++) {                 // 2 s at 50 Hz
        while ((int32_t)(millis() - next) < 0) delay(1);
        next += 20;
        if (read_one(acc, gyr)) { sum.x+=acc.x; sum.y+=acc.y; sum.z+=acc.z; ++n; }
        f_raw.update(gyr, acc);
        f_steady.update(gyr, acc);
    }
    if (n) {
        float m = std::sqrt(sum.x*sum.x + sum.y*sum.y + sum.z*sum.z);
        if (m > 1e-3f) g_ref = { sum.x/m, sum.y/m, sum.z/m };
    }
    Serial.printf(">>> Reference captured from %d samples.\n\n", n);
    Serial.println("Now sharpen normally, holding your angle as steady as you can.");
    Serial.println("sd/pp = how much the reading moves. LOWER IS BETTER.");
    Serial.println("gain  = how many times steadier the new filter is on YOUR data.");
    Serial.println("hdrm  = spare IMU samples per tick / how much a per-tick average");
    Serial.println("        would change the reading (milli-g). Large => oversampling");
    Serial.println("        is worth doing; near zero => it is not.");
    Serial.println();
}

void loop() {
    static uint32_t next   = millis();
    static uint32_t window = millis();
    static uint32_t ticks = 0, fails = 0;
    static uint32_t drained_total = 0, drain_samples = 0, drain_ticks = 0;

    while ((int32_t)(millis() - next) < 0) delay(1);
    next += 20;                                    // 50 Hz, as production
    M5.update();

    Vec3 acc{}, gyr{};
    bool ok = read_one(acc, gyr);
    ++ticks;

    if (!ok) {
        // getAccel/getGyro return false when no NEW sample is ready — they still
        // write the last cached one. Feeding that duplicate to the filters would
        // integrate the same sample twice and skew the very numbers this build
        // exists to report, so the tick is dropped rather than measured. Both
        // filters see identical input either way, so the comparison stays fair.
        ++fails;
    } else {
        // SAMPLE HEADROOM, throttled. Drain any further fresh samples and compare
        // their mean against the single sample production would have used: the
        // gap is the per-tick sampling noise an oversample would remove, which is
        // what decides whether the anti-alias work is worth doing.
        //
        // Throttled because draining CONSUMES those samples — doing it every tick
        // starves the next one and manufactures the read failure handled above,
        // corrupting the A/B measurement with an artefact of measuring it. One
        // tick in 25 (~2 Hz) leaves the other 24 with a production-like stream.
        if (++drain_ticks >= 25) {
            drain_ticks = 0;
            Vec3 more{}, mg{}, asum = acc;
            int  extra = 0;
            while (extra < 31 && read_one(more, mg)) {
                asum.x += more.x; asum.y += more.y; asum.z += more.z;
                ++extra;
            }
            drained_total += extra;
            ++drain_samples;
            if (extra > 0) {
                Vec3 mean = { asum.x/(extra+1), asum.y/(extra+1), asum.z/(extra+1) };
                float dx = acc.x-mean.x, dy = acc.y-mean.y, dz = acc.z-mean.z;
                s_drift.add(std::sqrt(dx*dx + dy*dy + dz*dz) * 1000.0f);  // milli-g
            }
        }

        f_raw.update(gyr, acc);
        f_steady.update(gyr, acc);

        s_raw.add(angle_between_deg(f_raw.gravity(), g_ref));
        s_steady.add(angle_between_deg(f_steady.gravity(), g_ref));
    }

    // Report every 2 s.
    if ((int32_t)(millis() - window) >= 2000) {
        window = millis();
        float gain_sd = (s_steady.sd() > 1e-4f) ? s_raw.sd() / s_steady.sd() : 0.0f;
        float gain_pp = (s_steady.pp() > 1e-4f) ? s_raw.pp() / s_steady.pp() : 0.0f;
        Serial.printf(
            "raw sd=%5.2f pp=%5.2f | steady sd=%5.2f pp=%5.2f | gain %4.1fx/%4.1fx"
            " | hdrm %.1f smp %.1fmg | readfail %u/%u\n",
            (double)s_raw.sd(),    (double)s_raw.pp(),
            (double)s_steady.sd(), (double)s_steady.pp(),
            (double)gain_sd,       (double)gain_pp,
            (double)drained_total / (drain_samples ? drain_samples : 1),
            (double)s_drift.mean(),
            (unsigned)fails, (unsigned)ticks);
        s_raw.reset(); s_steady.reset(); s_drift.reset();
        ticks = fails = 0; drained_total = 0; drain_samples = 0;
    }
}
#endif // DIAG_BUILD
