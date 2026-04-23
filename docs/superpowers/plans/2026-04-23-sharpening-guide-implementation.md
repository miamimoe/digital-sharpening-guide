# Sharpening Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the v1 firmware for the M5StickC Plus-based electronic sharpening angle guide defined in `docs/superpowers/specs/2026-04-23-sharpening-guide-design.md`.

**Architecture:** Single-threaded cooperative loop at 100 Hz on ESP32 (Arduino framework) driving a mode state machine. Logic layers (angle math, stroke FSM, side FSM, state machine, input FSM) are pure C++ and unit-tested natively on desktop. Hardware-coupled layers (IMU, UI, feedback, power, NVS, RTC RAM) are Arduino-only, compile-tested against the target platform, with bring-up tests documented for when hardware arrives.

**Tech Stack:** Arduino-ESP32 core 2.x, PlatformIO 6.x, `m5stack/M5Unified` library, Unity testing framework (bundled with PlatformIO), Mahony AHRS (inlined, ~100 LOC public domain).

---

## File Structure

```
./
├── platformio.ini                    # Two envs: m5stick-c-plus (target), native (desktop tests)
├── .gitignore
├── src/
│   ├── main.cpp                      # Arduino setup()/loop(), 100 Hz scheduler, wires modules
│   ├── types.h                       # Vec3, enums (State, Side, Tolerance, ColorState, InputEvent)
│   ├── angle.h / angle.cpp           # Pure: (g_ref, g_now) → (θ degrees, direction sign)
│   ├── stroke.h / stroke.cpp         # StrokeFSM: in/out hysteresis per side
│   ├── side.h / side.cpp             # SideFSM: spike/settle + gravity-sign + manual override
│   ├── filter.h / filter.cpp         # MahonyFilter: gyro+accel → gravity unit vector
│   ├── input.h / input.cpp           # InputFSM: debounce + short/long press events
│   ├── settings.h / settings.cpp     # NVS wrapper (Arduino) + in-memory mock (native)
│   ├── session.h / session.cpp       # RTC RAM wrapper (Arduino) + globals mock (native)
│   ├── imu.h / imu.cpp               # Arduino-only: MPU6886 begin/read + gyro bias capture
│   ├── ui.h / ui.cpp                 # Arduino-only: M5GFX rendering per screen
│   ├── feedback.h / feedback.cpp     # Arduino-only: LED + buzzer
│   ├── power.h / power.cpp           # Arduino-only: idle detection, backlight dim, deep sleep
│   └── app.h / app.cpp               # State machine transitions, event wiring
├── test/
│   ├── test_angle/test_angle.cpp
│   ├── test_stroke/test_stroke.cpp
│   ├── test_side/test_side.cpp
│   ├── test_filter/test_filter.cpp
│   ├── test_input/test_input.cpp
│   └── test_app/test_app.cpp
└── docs/
    └── superpowers/
        ├── specs/2026-04-23-sharpening-guide-design.md
        ├── plans/2026-04-23-sharpening-guide-implementation.md
        └── bringup/2026-04-23-hardware-bringup.md   # created in final task
```

**Responsibilities at a glance:**
- `types.h` — shared types only, no logic. Tiny.
- `angle.cpp` — pure function. Stateless.
- `stroke.cpp` — per-side hysteresis FSM. State: `bool sustained`, last-change timestamp, count.
- `side.cpp` — peel/settle FSM. State: phase, phase-entered timestamp, suppress-until timestamp, current side.
- `filter.cpp` — Mahony AHRS. State: quaternion, integral terms, gyro bias.
- `input.cpp` — two independent button FSMs (A, B). State: pressed since, long-emitted flag.
- `settings.cpp` — `Preferences` (Arduino) or static in-memory (native).
- `session.cpp` — `RTC_DATA_ATTR` struct (Arduino) or static globals (native).
- `imu.cpp` — MPU6886 init (with error codes), gyro+accel read, 10s bias capture routine.
- `ui.cpp` — per-state render functions; dirty-region tracking via "last rendered" snapshot.
- `feedback.cpp` — LED on/off, buzzer beep queue.
- `power.cpp` — idle timers, backlight LDO cut, `esp_deep_sleep_start()` sequence.
- `app.cpp` — the state machine. Owns transitions; calls into other modules.
- `main.cpp` — Arduino `setup()`/`loop()`, 100 Hz tick scheduler. Tiny — all logic lives in modules.

---

## Task 1: Project scaffold

**Files:**
- Create: `platformio.ini`
- Create: `.gitignore`
- Create: `src/main.cpp` (stub)
- Create: `src/types.h`

- [ ] **Step 1.1: Create `platformio.ini` with two environments**

Create file `./platformio.ini`:

```ini
[platformio]
default_envs = m5stick-c-plus

[env:m5stick-c-plus]
platform = espressif32@^6.5.0
board = m5stick-c
framework = arduino
upload_speed = 1500000
monitor_speed = 115200
build_flags =
    -D CORE_DEBUG_LEVEL=3
    -D ARDUINO_M5StickC_Plus
    -std=gnu++17
lib_deps =
    m5stack/M5Unified@^0.1.16

[env:native]
platform = native
test_framework = unity
build_flags =
    -std=gnu++17
    -D UNIT_TEST
    -Wall -Wextra
    -I include
test_build_src = yes
```

- [ ] **Step 1.2: Create `.gitignore`**

Create file `./.gitignore`:

```
.pio/
.pioenvs/
.piolibdeps/
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch
.DS_Store
build/
*.o
*.elf
*.bin
```

- [ ] **Step 1.3: Create `src/types.h`**

Create file `./src/types.h`:

```cpp
#pragma once
#include <cstdint>

struct Vec3 {
    float x;
    float y;
    float z;
};

enum class State : uint8_t {
    BOOT,
    BIAS_CAL,
    SET_TARGET,
    SET_TOLERANCE,
    ACTIVE,
    SUMMARY,
    FAULT,
    RESUME_PROMPT,
    SLEEP
};

enum class Side : uint8_t { A, B };

enum class Tolerance : uint8_t { TIGHT, NORMAL, EASY };

enum class ColorState : uint8_t { GREEN, BLUE, RED };

enum class InputEvent : uint8_t {
    NONE,
    A_SHORT,
    A_LONG,
    B_SHORT,
    B_LONG
};

enum class PresetSelection : uint8_t {
    P12,
    P15,
    P17,
    P20,
    P22,
    CANCEL
};

enum class FaultCode : uint8_t {
    NONE = 0,
    E01_BEGIN_FAILED = 1,
    E02_SELF_TEST_FAILED = 2,
    E03_WHO_AM_I_MISMATCH = 3
};

inline float tolerance_degrees(Tolerance t) {
    switch (t) {
        case Tolerance::TIGHT:  return 1.0f;
        case Tolerance::NORMAL: return 2.0f;
        case Tolerance::EASY:   return 3.0f;
    }
    return 2.0f;
}

inline float preset_degrees(PresetSelection p) {
    switch (p) {
        case PresetSelection::P12:    return 12.0f;
        case PresetSelection::P15:    return 15.0f;
        case PresetSelection::P17:    return 17.0f;
        case PresetSelection::P20:    return 20.0f;
        case PresetSelection::P22:    return 22.0f;
        case PresetSelection::CANCEL: return 0.0f;
    }
    return 17.0f;
}
```

- [ ] **Step 1.4: Create `src/main.cpp` stub**

Create file `./src/main.cpp`:

```cpp
#ifndef UNIT_TEST
#include <Arduino.h>

void setup() {
    // wired in Task 15
}

void loop() {
    // wired in Task 15
}
#endif
```

- [ ] **Step 1.5: Verify target env builds**

Run: `cd "." && pio run -e m5stick-c-plus`
Expected: compiles cleanly, produces `.pio/build/m5stick-c-plus/firmware.bin`. Network will download toolchain + M5Unified on first run — allow time.

- [ ] **Step 1.6: Verify native env builds**

Run: `cd "." && pio test -e native --without-uploading --without-testing`
Expected: exits cleanly with "No tests defined" — we have no tests yet. If it errors on toolchain, install via `pio platform install native`.

- [ ] **Step 1.7: Commit**

```bash
cd "."
git add platformio.ini .gitignore src/main.cpp src/types.h
git -c commit.gpgsign=false commit -m "scaffold: PlatformIO envs + shared types"
```

---

## Task 2: `angle` module (pure math, TDD)

**Files:**
- Create: `src/angle.h`
- Create: `src/angle.cpp`
- Create: `test/test_angle/test_angle.cpp`

- [ ] **Step 2.1: Write failing tests**

Create `./test/test_angle/test_angle.cpp`:

```cpp
#include <unity.h>
#include <cmath>
#include "angle.h"

void setUp(void) {}
void tearDown(void) {}

static void expect_near(float expected, float actual, float tol, const char* msg) {
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tol, expected, actual, msg);
}

void test_zero_deviation_is_zero_degrees(void) {
    Vec3 g = {0.0f, 0.0f, -1.0f};
    AngleResult r = compute_angle(g, g);
    expect_near(0.0f, r.degrees, 0.01f, "theta");
    TEST_ASSERT_EQUAL_INT(0, r.direction_sign);
}

void test_five_degree_increase_returns_red_sign(void) {
    // g_ref at "sharpening angle 15 degrees": n_back = (0,0,-1), so g_ref · n_back = sin(15°)
    // g_ref = (cos(15°), 0, -sin(15°)) chosen so gravity magnitude is 1 and projection onto n_back = -(-sin(15°)) = sin(15°)
    // Actually: dot((cos15, 0, -sin15), (0,0,-1)) = sin(15°). Good.
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r20 = 20.0f * (float)M_PI / 180.0f;
    Vec3 g_ref = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now = {std::cos(r20), 0.0f, -std::sin(r20)};
    AngleResult r = compute_angle(g_ref, g_now);
    expect_near(5.0f, r.degrees, 0.05f, "theta 5 deg");
    TEST_ASSERT_EQUAL_INT(+1, r.direction_sign);
}

void test_five_degree_decrease_returns_blue_sign(void) {
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r10 = 10.0f * (float)M_PI / 180.0f;
    Vec3 g_ref = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now = {std::cos(r10), 0.0f, -std::sin(r10)};
    AngleResult r = compute_angle(g_ref, g_now);
    expect_near(5.0f, r.degrees, 0.05f, "theta 5 deg");
    TEST_ASSERT_EQUAL_INT(-1, r.direction_sign);
}

void test_orientation_agnostic_rotation_around_n_back(void) {
    // Rotating g_ref by 90° around n_back (which is (0,0,-1)) = rotating around Z axis
    // should produce identical theta and sign for the same physical deviation.
    const float r15 = 15.0f * (float)M_PI / 180.0f;
    const float r20 = 20.0f * (float)M_PI / 180.0f;
    Vec3 g_ref_unrot = {std::cos(r15), 0.0f, -std::sin(r15)};
    Vec3 g_now_unrot = {std::cos(r20), 0.0f, -std::sin(r20)};
    // Rotate both by 90° around Z
    Vec3 g_ref_rot   = {0.0f, std::cos(r15), -std::sin(r15)};
    Vec3 g_now_rot   = {0.0f, std::cos(r20), -std::sin(r20)};
    AngleResult a = compute_angle(g_ref_unrot, g_now_unrot);
    AngleResult b = compute_angle(g_ref_rot,   g_now_rot);
    expect_near(a.degrees, b.degrees, 0.01f, "theta invariant under Z rot");
    TEST_ASSERT_EQUAL_INT(a.direction_sign, b.direction_sign);
}

void test_classify_within_tolerance_is_green(void) {
    AngleResult r = {1.5f, +1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::GREEN, (int)classify(r, 2.0f));
}

void test_classify_above_tolerance_with_positive_sign_is_red(void) {
    AngleResult r = {3.0f, +1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::RED, (int)classify(r, 2.0f));
}

void test_classify_above_tolerance_with_negative_sign_is_blue(void) {
    AngleResult r = {3.0f, -1};
    TEST_ASSERT_EQUAL_INT((int)ColorState::BLUE, (int)classify(r, 2.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_deviation_is_zero_degrees);
    RUN_TEST(test_five_degree_increase_returns_red_sign);
    RUN_TEST(test_five_degree_decrease_returns_blue_sign);
    RUN_TEST(test_orientation_agnostic_rotation_around_n_back);
    RUN_TEST(test_classify_within_tolerance_is_green);
    RUN_TEST(test_classify_above_tolerance_with_positive_sign_is_red);
    RUN_TEST(test_classify_above_tolerance_with_negative_sign_is_blue);
    return UNITY_END();
}
```

- [ ] **Step 2.2: Run tests to verify they fail (compile error — header doesn't exist yet)**

Run: `cd "." && pio test -e native -f test_angle`
Expected: compile error, `angle.h: No such file or directory`.

- [ ] **Step 2.3: Create header `src/angle.h`**

```cpp
#pragma once
#include "types.h"

struct AngleResult {
    float degrees;
    int   direction_sign; // -1 = angle decreased (BLUE), 0 = at ref, +1 = angle increased (RED)
};

// Compile-time axis pointing out the back of the device (into the blade).
// Set for MPU6886 body frame on M5StickC Plus PCB; validated by bring-up test in docs/superpowers/bringup/.
constexpr Vec3 N_BACK = {0.0f, 0.0f, -1.0f};

AngleResult compute_angle(Vec3 g_ref, Vec3 g_now);
ColorState  classify(AngleResult r, float tolerance_deg);
```

- [ ] **Step 2.4: Create implementation `src/angle.cpp`**

```cpp
#include "angle.h"
#include <cmath>

static inline float dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

AngleResult compute_angle(Vec3 g_ref, Vec3 g_now) {
    // Magnitude: angle between the two unit vectors.
    float d = dot(g_ref, g_now);
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    float theta_rad = std::acos(d);
    float theta_deg = theta_rad * (180.0f / (float)M_PI);

    // Direction: signed projection onto n_back.
    // g·n_back = sin(sharpening_angle), monotonic over ±90° range.
    float alpha_ref = dot(g_ref, N_BACK);
    float alpha_now = dot(g_now, N_BACK);
    float delta     = alpha_now - alpha_ref;

    int sign = 0;
    const float eps = 1e-6f;
    if (delta >  eps) sign = +1;
    if (delta < -eps) sign = -1;

    return {theta_deg, sign};
}

ColorState classify(AngleResult r, float tolerance_deg) {
    if (r.degrees <= tolerance_deg) return ColorState::GREEN;
    if (r.direction_sign >= 0)      return ColorState::RED;
    return ColorState::BLUE;
}
```

- [ ] **Step 2.5: Run tests to verify they pass**

Run: `cd "." && pio test -e native -f test_angle`
Expected: 7 passed, 0 failed.

- [ ] **Step 2.6: Verify module compiles on target too**

Run: `cd "." && pio run -e m5stick-c-plus`
Expected: clean build.

- [ ] **Step 2.7: Commit**

```bash
cd "."
git add src/angle.h src/angle.cpp test/test_angle/
git -c commit.gpgsign=false commit -m "feat(angle): pure angle math with unit tests"
```

---

## Task 3: `stroke` FSM (hysteresis, TDD)

**Files:**
- Create: `src/stroke.h`
- Create: `src/stroke.cpp`
- Create: `test/test_stroke/test_stroke.cpp`

- [ ] **Step 3.1: Write failing tests**

Create `./test/test_stroke/test_stroke.cpp`:

```cpp
#include <unity.h>
#include "stroke.h"

void setUp(void) {}
void tearDown(void) {}

// Helper: step FSM forward by dt_ms with a constant in_tolerance value,
// in 10ms increments (matching 100 Hz sample cadence).
static void drive(StrokeFSM& fsm, uint32_t& t, uint32_t dt_ms, bool in_tol) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        fsm.update(t, in_tol);
    }
}

void test_starts_at_zero_count(void) {
    StrokeFSM fsm;
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_canonical_single_stroke(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  false); // starts OUT
    drive(fsm, t, 500,  true);  // >= 300ms IN sustained
    drive(fsm, t, 300,  false); // >= 200ms OUT sustained => one stroke counted
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_sub_300ms_in_blip_does_not_count(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200, false);
    drive(fsm, t, 200, true);  // only 200ms IN — too short
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

void test_sub_200ms_out_wobble_does_not_end_stroke(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 500, true);  // enter IN
    drive(fsm, t, 100, false); // wobble — too short to end
    drive(fsm, t, 200, true);  // still IN
    drive(fsm, t, 300, false); // proper exit — one stroke
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_three_back_to_back_strokes(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    for (int i = 0; i < 3; i++) {
        drive(fsm, t, 500, true);
        drive(fsm, t, 300, false);
    }
    TEST_ASSERT_EQUAL_UINT32(3, fsm.stroke_count());
}

void test_long_unbroken_in_counts_exactly_one_on_exit(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 4000, true);  // 4 seconds in, never exits
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count()); // not counted until exit
    drive(fsm, t, 300, false);  // finally exits
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
}

void test_reset_zeros_count(void) {
    StrokeFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 500, true);
    drive(fsm, t, 300, false);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.stroke_count());
    fsm.reset();
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stroke_count());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_zero_count);
    RUN_TEST(test_canonical_single_stroke);
    RUN_TEST(test_sub_300ms_in_blip_does_not_count);
    RUN_TEST(test_sub_200ms_out_wobble_does_not_end_stroke);
    RUN_TEST(test_three_back_to_back_strokes);
    RUN_TEST(test_long_unbroken_in_counts_exactly_one_on_exit);
    RUN_TEST(test_reset_zeros_count);
    return UNITY_END();
}
```

- [ ] **Step 3.2: Run tests to verify they fail**

Run: `pio test -e native -f test_stroke`
Expected: compile error, header missing.

- [ ] **Step 3.3: Create `src/stroke.h`**

```cpp
#pragma once
#include <cstdint>

class StrokeFSM {
public:
    static constexpr uint32_t IN_MIN_MS  = 300;
    static constexpr uint32_t OUT_MIN_MS = 200;

    void      update(uint32_t now_ms, bool in_tolerance);
    uint32_t  stroke_count() const { return count_; }
    bool      is_in_tolerance() const { return sustained_; }
    void      reset();

private:
    bool     sustained_            = false; // current stable state (start OUT)
    bool     contrary_current_     = false; // last observed reading
    uint32_t contrary_started_ms_  = 0;     // when contrary run began (0 = no contrary)
    uint32_t count_                = 0;
};
```

- [ ] **Step 3.4: Create `src/stroke.cpp`**

```cpp
#include "stroke.h"

void StrokeFSM::update(uint32_t now_ms, bool in_tolerance) {
    if (in_tolerance == sustained_) {
        // No contrary evidence — reset attempt timer.
        contrary_started_ms_ = 0;
        contrary_current_    = in_tolerance;
        return;
    }

    // Contrary evidence this sample.
    if (contrary_started_ms_ == 0 || contrary_current_ != in_tolerance) {
        contrary_started_ms_ = now_ms;
        contrary_current_    = in_tolerance;
    }

    uint32_t required = sustained_ ? OUT_MIN_MS : IN_MIN_MS;
    if (now_ms - contrary_started_ms_ >= required) {
        sustained_           = in_tolerance;
        contrary_started_ms_ = 0;
        // Count a stroke only on IN_TOL -> OUT_TOL
        if (!sustained_) {
            count_++;
        }
    }
}

void StrokeFSM::reset() {
    sustained_           = false;
    contrary_current_    = false;
    contrary_started_ms_ = 0;
    count_               = 0;
}
```

- [ ] **Step 3.5: Run tests to verify they pass**

Run: `pio test -e native -f test_stroke`
Expected: 7 passed.

- [ ] **Step 3.6: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 3.7: Commit**

```bash
git add src/stroke.h src/stroke.cpp test/test_stroke/
git -c commit.gpgsign=false commit -m "feat(stroke): per-side hysteresis FSM with unit tests"
```

---

## Task 4: `side` FSM (peel/settle + manual override, TDD)

**Files:**
- Create: `src/side.h`
- Create: `src/side.cpp`
- Create: `test/test_side/test_side.cpp`

- [ ] **Step 4.1: Write failing tests**

Create `./test/test_side/test_side.cpp`:

```cpp
#include <unity.h>
#include "side.h"

void setUp(void) {}
void tearDown(void) {}

// Drive the FSM at 100 Hz (10ms steps). grav_dot_ref ≈ +1 (same side) or -1 (flipped).
static void drive(SideFSM& fsm, uint32_t& t, uint32_t dt_ms, float accel_mag, float grav_dot_ref) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        fsm.update(t, accel_mag, grav_dot_ref);
    }
}

void test_starts_on_side_a_no_events(void) {
    SideFSM fsm;
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
    TEST_ASSERT_FALSE(fsm.consume_switch());
}

void test_peel_flip_settle_triggers_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);  // quiescent
    drive(fsm, t, 100,  1.80f, +1.0f);  // spike: |1.8-1| = 0.8g > 0.5g
    drive(fsm, t, 200,  0.40f, -1.0f);  // handling motion, device is flipped in the user's hand
    drive(fsm, t, 600,  1.00f, -1.0f);  // settles, flipped — dot is now -1
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_peel_no_flip_settle_no_switch(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);
    drive(fsm, t, 100,  1.80f, +1.0f);
    drive(fsm, t, 200,  0.40f, +1.0f);
    drive(fsm, t, 600,  1.00f, +1.0f);   // settled, same side
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_peel_no_settle_within_timeout_resets(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 200,  1.00f, +1.0f);
    drive(fsm, t, 100,  1.80f, +1.0f);
    drive(fsm, t, 5500, 0.50f, -1.0f);   // continuous motion > 5s timeout, never settles
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

void test_manual_toggle_switches_and_suppresses_auto(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, +1.0f);
    fsm.manual_toggle(t);
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
    TEST_ASSERT_TRUE(fsm.consume_switch());
    // Now during suppression window, a peel+flip+settle should NOT trigger auto-switch.
    drive(fsm, t, 100, 1.80f, -1.0f);
    drive(fsm, t, 1800, 0.40f, -1.0f);
    drive(fsm, t, 600,  1.00f, +1.0f);   // grav_dot_ref sign relative to CAPTURE (unchanged)
    TEST_ASSERT_FALSE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::B, (int)fsm.current_side());
}

void test_suppression_expires_after_2s(void) {
    SideFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 100, 1.0f, +1.0f);
    fsm.manual_toggle(t);               // t = 100ms, suppression until t + 2000ms = 2100ms
    fsm.consume_switch();
    drive(fsm, t, 2200, 1.0f, -1.0f);   // advance past suppression (t now ~2300ms)
    drive(fsm, t, 100,  1.80f, -1.0f);  // spike after suppression expired
    drive(fsm, t, 200,  0.40f, +1.0f);  // handling; dot relative to original g_ref
    drive(fsm, t, 600,  1.00f, +1.0f);  // settled back to original side — flipped from B's perspective
    // manual_toggle set side to B. A flip from B back to A should switch.
    TEST_ASSERT_TRUE(fsm.consume_switch());
    TEST_ASSERT_EQUAL_INT((int)Side::A, (int)fsm.current_side());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_on_side_a_no_events);
    RUN_TEST(test_peel_flip_settle_triggers_switch);
    RUN_TEST(test_peel_no_flip_settle_no_switch);
    RUN_TEST(test_peel_no_settle_within_timeout_resets);
    RUN_TEST(test_manual_toggle_switches_and_suppresses_auto);
    RUN_TEST(test_suppression_expires_after_2s);
    return UNITY_END();
}
```

- [ ] **Step 4.2: Run tests — fail compile**

Run: `pio test -e native -f test_side`
Expected: compile error.

- [ ] **Step 4.3: Create `src/side.h`**

```cpp
#pragma once
#include <cstdint>
#include "types.h"

class SideFSM {
public:
    static constexpr float    SPIKE_DEVIATION_G  = 0.5f;
    static constexpr float    SETTLE_TOL_G       = 0.1f;
    static constexpr uint32_t SETTLE_REQUIRED_MS = 500;
    static constexpr uint32_t POST_SPIKE_TIMEOUT_MS = 5000;
    static constexpr uint32_t SUPPRESS_MS        = 2000;

    void  update(uint32_t now_ms, float accel_mag_g, float grav_dot_ref);
    void  manual_toggle(uint32_t now_ms);
    bool  consume_switch();   // returns true if a switch was pending; clears it
    Side  current_side() const { return side_; }
    void  reset();

private:
    enum Phase : uint8_t { STABLE, WAITING_SETTLE };

    Phase    phase_              = STABLE;
    uint32_t phase_entered_ms_   = 0;
    uint32_t settle_started_ms_  = 0;   // 0 when not currently settling
    uint32_t suppress_until_ms_  = 0;
    bool     switch_pending_     = false;
    Side     side_               = Side::A;
};
```

- [ ] **Step 4.4: Create `src/side.cpp`**

```cpp
#include "side.h"
#include <cmath>

static inline bool suppressed(uint32_t now_ms, uint32_t until_ms) {
    return until_ms > now_ms;
}

void SideFSM::update(uint32_t now_ms, float accel_mag_g, float grav_dot_ref) {
    float dev = std::fabs(accel_mag_g - 1.0f);

    if (phase_ == STABLE) {
        if (dev > SPIKE_DEVIATION_G) {
            phase_              = WAITING_SETTLE;
            phase_entered_ms_   = now_ms;
            settle_started_ms_  = 0;
        }
        return;
    }

    // WAITING_SETTLE
    if (now_ms - phase_entered_ms_ > POST_SPIKE_TIMEOUT_MS) {
        // Never settled in time — give up, back to STABLE.
        phase_ = STABLE;
        settle_started_ms_ = 0;
        return;
    }

    if (dev <= SETTLE_TOL_G) {
        if (settle_started_ms_ == 0) settle_started_ms_ = now_ms;
        if (now_ms - settle_started_ms_ >= SETTLE_REQUIRED_MS) {
            // Settled. Check for flip.
            if (grav_dot_ref < 0.0f && !suppressed(now_ms, suppress_until_ms_)) {
                side_            = (side_ == Side::A) ? Side::B : Side::A;
                switch_pending_  = true;
            }
            phase_             = STABLE;
            settle_started_ms_ = 0;
        }
    } else {
        settle_started_ms_ = 0;
    }
}

void SideFSM::manual_toggle(uint32_t now_ms) {
    side_              = (side_ == Side::A) ? Side::B : Side::A;
    switch_pending_    = true;
    suppress_until_ms_ = now_ms + SUPPRESS_MS;
}

bool SideFSM::consume_switch() {
    bool v = switch_pending_;
    switch_pending_ = false;
    return v;
}

void SideFSM::reset() {
    phase_              = STABLE;
    phase_entered_ms_   = 0;
    settle_started_ms_  = 0;
    suppress_until_ms_  = 0;
    switch_pending_     = false;
    side_               = Side::A;
}
```

- [ ] **Step 4.5: Run tests — verify pass**

Run: `pio test -e native -f test_side`
Expected: 6 passed.

- [ ] **Step 4.6: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 4.7: Commit**

```bash
git add src/side.h src/side.cpp test/test_side/
git -c commit.gpgsign=false commit -m "feat(side): peel/settle FSM with manual override + unit tests"
```

---

## Task 5: `filter` module — Mahony AHRS

**Files:**
- Create: `src/filter.h`
- Create: `src/filter.cpp`
- Create: `test/test_filter/test_filter.cpp`

- [ ] **Step 5.1: Write failing tests**

Create `./test/test_filter/test_filter.cpp`:

```cpp
#include <unity.h>
#include <cmath>
#include "filter.h"

void setUp(void) {}
void tearDown(void) {}

void test_converges_to_stationary_gravity(void) {
    MahonyFilter f;
    f.begin(100.0f); // 100 Hz
    // Feed 500 samples of pure -Z gravity, zero gyro.
    for (int i = 0; i < 500; i++) {
        f.update({0,0,0}, {0.0f, 0.0f, -1.0f});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, g.x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.02f,-1.0f, g.z);
}

void test_tilts_toward_new_gravity_direction(void) {
    MahonyFilter f;
    f.begin(100.0f);
    // Tilt gravity 30° around X axis: new accel reading = (0, -sin30, -cos30)
    const float g30y = -0.5f;
    const float g30z = -std::sqrt(3.0f)/2.0f;
    for (int i = 0; i < 500; i++) {
        f.update({0,0,0}, {0.0f, g30y, g30z});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.03f, g30y, g.y);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, g30z, g.z);
}

void test_gyro_bias_subtracted(void) {
    MahonyFilter f;
    f.begin(100.0f);
    f.set_bias({1.0f, 0.0f, 0.0f}); // 1 deg/s X-axis bias
    // Feed constant gravity and matching gyro (which should be cancelled by bias).
    for (int i = 0; i < 500; i++) {
        f.update({1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});
    }
    Vec3 g = f.gravity();
    TEST_ASSERT_FLOAT_WITHIN(0.02f,-1.0f, g.z);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_stationary_gravity);
    RUN_TEST(test_tilts_toward_new_gravity_direction);
    RUN_TEST(test_gyro_bias_subtracted);
    return UNITY_END();
}
```

- [ ] **Step 5.2: Run tests — fail compile**

Run: `pio test -e native -f test_filter`
Expected: compile error.

- [ ] **Step 5.3: Create `src/filter.h`**

```cpp
#pragma once
#include "types.h"

class MahonyFilter {
public:
    void begin(float sample_hz, float kp = 0.5f, float ki = 0.0f);
    void update(Vec3 gyro_dps, Vec3 accel_g);
    void set_bias(Vec3 gyro_bias_dps) { bias_ = gyro_bias_dps; }
    Vec3 gravity() const; // unit vector in device body frame
    void reset();

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;
    float kp_ = 0.5f, ki_ = 0.0f;
    float dt_ = 0.01f;
    Vec3  bias_ = {0.0f, 0.0f, 0.0f};
};
```

- [ ] **Step 5.4: Create `src/filter.cpp`**

Implementation is adapted from the canonical Mahony reference implementation (public domain, originally by Sebastian Madgwick).

```cpp
#include "filter.h"
#include <cmath>

static inline float inv_sqrt(float x) {
    // Plain reciprocal sqrt. Not using the fast-inverse-sqrt hack — modern FPUs are faster.
    return 1.0f / std::sqrt(x);
}

void MahonyFilter::begin(float sample_hz, float kp, float ki) {
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    ix_ = iy_ = iz_ = 0.0f;
    kp_ = kp;
    ki_ = ki;
    dt_ = 1.0f / sample_hz;
}

void MahonyFilter::reset() {
    begin(1.0f / dt_, kp_, ki_);
}

void MahonyFilter::update(Vec3 gyro_dps, Vec3 accel_g) {
    // Convert gyro to radians/sec and subtract bias.
    const float DEG2RAD = (float)M_PI / 180.0f;
    float gx = (gyro_dps.x - bias_.x) * DEG2RAD;
    float gy = (gyro_dps.y - bias_.y) * DEG2RAD;
    float gz = (gyro_dps.z - bias_.z) * DEG2RAD;

    float ax = accel_g.x, ay = accel_g.y, az = accel_g.z;

    // If accel is valid (non-zero), use it for correction.
    float an = ax*ax + ay*ay + az*az;
    if (an > 0.0f) {
        float inv = inv_sqrt(an);
        ax *= inv; ay *= inv; az *= inv;

        // Estimated direction of gravity (rotate (0,0,1) by current q).
        // Using convention where gravity accel reads +1 on the axis aligned with world "up".
        // For the M5StickC Plus flat-on-table (screen up) case, local Z reads -1 g; to align
        // with the standard Mahony convention we take the sensed accel as the measurement and
        // the filter's predicted (0,0,1) unit vector; we pass accel in the MPU frame directly.
        float vx = 2.0f * (q1_*q3_ - q0_*q2_);
        float vy = 2.0f * (q0_*q1_ + q2_*q3_);
        float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

        // Error is cross product between measured and estimated.
        float ex = (ay*vz - az*vy);
        float ey = (az*vx - ax*vz);
        float ez = (ax*vy - ay*vx);

        if (ki_ > 0.0f) {
            ix_ += ki_ * ex * dt_;
            iy_ += ki_ * ey * dt_;
            iz_ += ki_ * ez * dt_;
            gx += ix_;
            gy += iy_;
            gz += iz_;
        }

        gx += kp_ * ex;
        gy += kp_ * ey;
        gz += kp_ * ez;
    }

    // Integrate quaternion rate.
    gx *= 0.5f * dt_;
    gy *= 0.5f * dt_;
    gz *= 0.5f * dt_;

    float qa = q0_, qb = q1_, qc = q2_;
    q0_ += -qb*gx - qc*gy - q3_*gz;
    q1_ +=  qa*gx + qc*gz - q3_*gy;
    q2_ +=  qa*gy - qb*gz + q3_*gx;
    q3_ +=  qa*gz + qb*gy - qc*gx;

    float inv = inv_sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
}

Vec3 MahonyFilter::gravity() const {
    // Gravity in body frame: rotate world -Z (down) by inverse of orientation quaternion.
    // Equivalently: the sensed accel direction when stationary is -v where v = predicted up.
    // We return the unit-gravity vector in body frame (the direction the body measures 1 g toward).
    float vx = 2.0f * (q1_*q3_ - q0_*q2_);
    float vy = 2.0f * (q0_*q1_ + q2_*q3_);
    float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;
    // Flip sign so the returned vector matches the accel sign convention (gravity points "down"
    // in body frame; stationary screen-up → Z = -1).
    return {-vx, -vy, -vz};
}
```

- [ ] **Step 5.5: Run tests — verify pass**

Run: `pio test -e native -f test_filter`
Expected: 3 passed.

- [ ] **Step 5.6: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 5.7: Commit**

```bash
git add src/filter.h src/filter.cpp test/test_filter/
git -c commit.gpgsign=false commit -m "feat(filter): Mahony AHRS with convergence tests"
```

---

## Task 6: `input` module (debounce + short/long press, TDD)

**Files:**
- Create: `src/input.h`
- Create: `src/input.cpp`
- Create: `test/test_input/test_input.cpp`

- [ ] **Step 6.1: Write failing tests**

Create `./test/test_input/test_input.cpp`:

```cpp
#include <unity.h>
#include "input.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(InputFSM& fsm, uint32_t& t, uint32_t dt_ms, bool a, bool b, InputEvent expected_last) {
    InputEvent last = InputEvent::NONE;
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        InputEvent ev = fsm.update(t, a, b);
        if (ev != InputEvent::NONE) last = ev;
    }
    TEST_ASSERT_EQUAL_INT((int)expected_last, (int)last);
}

void test_short_press_a_on_release(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,  false, false, InputEvent::NONE);
    drive(fsm, t, 300, true,  false, InputEvent::NONE);   // held 300ms (<800ms) — no long yet
    drive(fsm, t, 50,  false, false, InputEvent::A_SHORT); // released → A_SHORT
}

void test_long_press_a_at_800ms_while_held(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
    drive(fsm, t, 900,  true,  false, InputEvent::A_LONG); // fires at 800ms threshold
    // Releasing after long-fire must NOT emit a short.
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
}

void test_short_press_b(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,  false, false, InputEvent::NONE);
    drive(fsm, t, 200, false, true,  InputEvent::NONE);
    drive(fsm, t, 50,  false, false, InputEvent::B_SHORT);
}

void test_long_press_b(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
    drive(fsm, t, 900,  false, true,  InputEvent::B_LONG);
    drive(fsm, t, 50,   false, false, InputEvent::NONE);
}

void test_debounces_spurious_short_blip(void) {
    InputFSM fsm;
    uint32_t t = 0;
    drive(fsm, t, 50, false, false, InputEvent::NONE);
    // 20ms blip — below debounce threshold — must not emit anything.
    drive(fsm, t, 20, true,  false, InputEvent::NONE);
    drive(fsm, t, 80, false, false, InputEvent::NONE);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_short_press_a_on_release);
    RUN_TEST(test_long_press_a_at_800ms_while_held);
    RUN_TEST(test_short_press_b);
    RUN_TEST(test_long_press_b);
    RUN_TEST(test_debounces_spurious_short_blip);
    return UNITY_END();
}
```

- [ ] **Step 6.2: Run tests — fail compile**

Run: `pio test -e native -f test_input`
Expected: compile error.

- [ ] **Step 6.3: Create `src/input.h`**

```cpp
#pragma once
#include <cstdint>
#include "types.h"

class InputFSM {
public:
    static constexpr uint32_t DEBOUNCE_MS      = 30;
    static constexpr uint32_t LONG_PRESS_MS    = 800;

    // Raw button states: true = pressed.
    // Returns a single event per tick (short on release, long when crossing threshold).
    InputEvent update(uint32_t now_ms, bool a_pressed, bool b_pressed);

private:
    struct Button {
        bool     raw_current       = false;
        bool     stable            = false;
        uint32_t raw_changed_ms    = 0;
        uint32_t pressed_since_ms  = 0;
        bool     long_emitted      = false;
    };
    Button a_;
    Button b_;
    InputEvent step_button(uint32_t now_ms, Button& btn, bool raw, InputEvent on_short, InputEvent on_long);
};
```

- [ ] **Step 6.4: Create `src/input.cpp`**

```cpp
#include "input.h"

InputEvent InputFSM::step_button(uint32_t now_ms, Button& btn, bool raw,
                                  InputEvent on_short, InputEvent on_long)
{
    if (raw != btn.raw_current) {
        btn.raw_current    = raw;
        btn.raw_changed_ms = now_ms;
    }
    // Debounce: raw must be stable for DEBOUNCE_MS.
    if (btn.raw_current != btn.stable && (now_ms - btn.raw_changed_ms) >= DEBOUNCE_MS) {
        btn.stable = btn.raw_current;
        if (btn.stable) {
            // Press edge.
            btn.pressed_since_ms = now_ms;
            btn.long_emitted     = false;
        } else {
            // Release edge — emit short only if we never emitted a long.
            if (!btn.long_emitted) {
                return on_short;
            }
        }
    }
    // While held, check for long threshold.
    if (btn.stable && !btn.long_emitted && (now_ms - btn.pressed_since_ms) >= LONG_PRESS_MS) {
        btn.long_emitted = true;
        return on_long;
    }
    return InputEvent::NONE;
}

InputEvent InputFSM::update(uint32_t now_ms, bool a_pressed, bool b_pressed) {
    InputEvent ea = step_button(now_ms, a_, a_pressed, InputEvent::A_SHORT, InputEvent::A_LONG);
    InputEvent eb = step_button(now_ms, b_, b_pressed, InputEvent::B_SHORT, InputEvent::B_LONG);
    // Prefer A events if both fire this tick (rare).
    if (ea != InputEvent::NONE) return ea;
    return eb;
}
```

- [ ] **Step 6.5: Run tests — verify pass**

Run: `pio test -e native -f test_input`
Expected: 5 passed.

- [ ] **Step 6.6: Target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 6.7: Commit**

```bash
git add src/input.h src/input.cpp test/test_input/
git -c commit.gpgsign=false commit -m "feat(input): debounce + short/long press FSM with tests"
```

---

## Task 7: `settings` module (NVS wrapper + native mock)

**Files:**
- Create: `src/settings.h`
- Create: `src/settings.cpp`

- [ ] **Step 7.1: Create `src/settings.h`**

```cpp
#pragma once
#include "types.h"

namespace settings {
    void begin();

    Tolerance  load_tolerance();
    void       save_tolerance(Tolerance t);

    bool       load_buzzer();
    void       save_buzzer(bool on);

    Vec3       load_gyro_bias();
    void       save_gyro_bias(Vec3 bias_dps);

    bool       is_first_boot();
    void       clear_first_boot();
}
```

- [ ] **Step 7.2: Create `src/settings.cpp`**

```cpp
#include "settings.h"

#ifdef UNIT_TEST
// Native/test mode: in-memory backing.
namespace {
    Tolerance g_tol   = Tolerance::NORMAL;
    bool      g_buzz  = false;
    Vec3      g_bias  = {0.0f, 0.0f, 0.0f};
    bool      g_first = true;
}
namespace settings {
    void begin() {}
    Tolerance load_tolerance()            { return g_tol; }
    void      save_tolerance(Tolerance t) { g_tol = t; }
    bool      load_buzzer()               { return g_buzz; }
    void      save_buzzer(bool on)        { g_buzz = on; }
    Vec3      load_gyro_bias()            { return g_bias; }
    void      save_gyro_bias(Vec3 b)      { g_bias = b; }
    bool      is_first_boot()             { return g_first; }
    void      clear_first_boot()          { g_first = false; }
}
#else
// Arduino: NVS via ESP32 Preferences.
#include <Preferences.h>
namespace {
    Preferences prefs;
    constexpr const char* NS = "sharpguide";
}
namespace settings {
    void begin() {
        prefs.begin(NS, /*readOnly=*/false);
    }
    Tolerance load_tolerance() {
        return static_cast<Tolerance>(prefs.getUChar("tol", (uint8_t)Tolerance::NORMAL));
    }
    void save_tolerance(Tolerance t) {
        prefs.putUChar("tol", (uint8_t)t);
    }
    bool load_buzzer()               { return prefs.getBool("buzz", false); }
    void save_buzzer(bool on)        { prefs.putBool("buzz", on); }
    Vec3 load_gyro_bias() {
        Vec3 v;
        v.x = prefs.getFloat("bx", 0.0f);
        v.y = prefs.getFloat("by", 0.0f);
        v.z = prefs.getFloat("bz", 0.0f);
        return v;
    }
    void save_gyro_bias(Vec3 b) {
        prefs.putFloat("bx", b.x);
        prefs.putFloat("by", b.y);
        prefs.putFloat("bz", b.z);
    }
    bool is_first_boot()       { return prefs.getBool("first", true); }
    void clear_first_boot()    { prefs.putBool("first", false); }
}
#endif
```

- [ ] **Step 7.3: Verify both envs build**

Run: `pio test -e native --without-uploading --without-testing` then `pio run -e m5stick-c-plus`
Expected: both clean.

- [ ] **Step 7.4: Commit**

```bash
git add src/settings.h src/settings.cpp
git -c commit.gpgsign=false commit -m "feat(settings): NVS wrapper with in-memory native mock"
```

---

## Task 8: `session` module (RTC RAM wrapper + native mock)

**Files:**
- Create: `src/session.h`
- Create: `src/session.cpp`

- [ ] **Step 8.1: Create `src/session.h`**

```cpp
#pragma once
#include "types.h"
#include <cstdint>

struct SessionState {
    bool      active         = false;
    float     target_deg     = 0.0f;
    Tolerance tolerance      = Tolerance::NORMAL;
    Vec3      g_ref          = {0.0f, 0.0f, 0.0f};
    uint32_t  strokes_A      = 0;
    uint32_t  strokes_B      = 0;
    Side      current_side   = Side::A;
    uint32_t  session_started_ms = 0;
};

namespace session {
    void          begin();
    SessionState& state();
    bool          has_session();
    void          mark_active(const SessionState& s);
    void          clear();
}
```

- [ ] **Step 8.2: Create `src/session.cpp`**

```cpp
#include "session.h"

#ifdef UNIT_TEST
namespace {
    SessionState g_state;
}
namespace session {
    void          begin() {}
    SessionState& state() { return g_state; }
    bool          has_session() { return g_state.active; }
    void          mark_active(const SessionState& s) { g_state = s; g_state.active = true; }
    void          clear() { g_state = SessionState{}; g_state.active = false; }
}
#else
#include <Arduino.h>
// RTC_DATA_ATTR survives deep sleep.
RTC_DATA_ATTR static SessionState g_state;
RTC_DATA_ATTR static bool         g_state_valid = false;

namespace session {
    void begin() {
        if (!g_state_valid) {
            g_state = SessionState{};
            g_state_valid = true;
        }
    }
    SessionState& state() { return g_state; }
    bool has_session()    { return g_state.active; }
    void mark_active(const SessionState& s) {
        g_state = s;
        g_state.active = true;
    }
    void clear() {
        g_state = SessionState{};
        g_state.active = false;
    }
}
#endif
```

- [ ] **Step 8.3: Verify both envs build**

Run: `pio run -e m5stick-c-plus` + `pio test -e native --without-uploading --without-testing`
Expected: both clean.

- [ ] **Step 8.4: Commit**

```bash
git add src/session.h src/session.cpp
git -c commit.gpgsign=false commit -m "feat(session): RTC RAM session state + native mock"
```

---

## Task 9: `imu` module (Arduino-only; MPU6886 + bias capture)

**Files:**
- Create: `src/imu.h`
- Create: `src/imu.cpp`

No desktop tests — this is pure hardware interface. Bring-up validation is in the final task's bring-up doc.

- [ ] **Step 9.1: Create `src/imu.h`**

```cpp
#pragma once
#include "types.h"

namespace imu {
    // Initialize MPU6886 via M5Unified's built-in IMU driver.
    // Returns NONE on success, else a FaultCode explaining what failed.
    FaultCode begin();

    // Read latest accel (g) + gyro (deg/s) into outputs. Must be called
    // AFTER a successful begin(). Returns false on transient I/O failure.
    bool read(Vec3& accel_g, Vec3& gyro_dps);

    // Blocking 10-second bias capture. Call only when device is still.
    // Averages gyro samples at 100 Hz; returns the mean.
    // Returns false if stillness criterion is violated (|accel-1g| > thresh) during capture.
    bool capture_gyro_bias(Vec3& bias_out_dps);
}
```

- [ ] **Step 9.2: Create `src/imu.cpp`**

```cpp
#include "imu.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace imu {

FaultCode begin() {
    auto cfg = M5.config();
    M5.begin(cfg);

    // M5Unified auto-detects the IMU. For M5StickC Plus the sensor is MPU6886.
    if (!M5.Imu.isEnabled()) {
        return FaultCode::E01_BEGIN_FAILED;
    }

    // Sanity-read to confirm it returns non-zero samples.
    Vec3 a, g;
    bool ok = false;
    for (int i = 0; i < 10; i++) {
        if (read(a, g)) { ok = true; break; }
        delay(5);
    }
    if (!ok) return FaultCode::E02_SELF_TEST_FAILED;

    // WHO_AM_I can be checked via M5.Imu.getType() on newer M5Unified revisions.
    // The type enum for MPU6886 is imu_mpu6886. If it's not what we expect, the I2C
    // conflict known-issue likely triggered.
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
    constexpr int   SAMPLES = 1000;        // 10 s @ 100 Hz
    constexpr float STILLNESS_G = 0.15f;   // |accel mag − 1| must stay under this
    double sx = 0, sy = 0, sz = 0;
    uint32_t last = millis();
    int n = 0;
    while (n < SAMPLES) {
        uint32_t now = millis();
        if (now - last < 10) { delay(1); continue; }
        last = now;
        Vec3 a, g;
        if (!read(a, g)) return false;
        float mag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
        if (fabsf(mag - 1.0f) > STILLNESS_G) {
            // Motion detected — restart the capture.
            sx = sy = sz = 0;
            n = 0;
            continue;
        }
        sx += g.x; sy += g.y; sz += g.z;
        n++;
    }
    bias_out_dps = {(float)(sx / n), (float)(sy / n), (float)(sz / n)};
    return true;
}

} // namespace imu
#else
// Native stub so anything including imu.h in shared code still links for tests.
namespace imu {
    FaultCode begin() { return FaultCode::NONE; }
    bool read(Vec3&, Vec3&) { return true; }
    bool capture_gyro_bias(Vec3& out) { out = {0,0,0}; return true; }
}
#endif
```

- [ ] **Step 9.3: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean. If the M5Unified API for `getType()` differs, consult `.pio/libdeps/m5stick-c-plus/M5Unified/src/utility/IMU_Class.hpp` and swap the enum to match.

- [ ] **Step 9.4: Commit**

```bash
git add src/imu.h src/imu.cpp
git -c commit.gpgsign=false commit -m "feat(imu): MPU6886 init with fault codes + bias capture"
```

---

## Task 10: `ui` module (Arduino-only; per-screen render)

**Files:**
- Create: `src/ui.h`
- Create: `src/ui.cpp`

Dirty-region rendering: the module keeps last-rendered values and only repaints what changed.

- [ ] **Step 10.1: Create `src/ui.h`**

```cpp
#pragma once
#include "types.h"

namespace ui {
    struct ActiveView {
        ColorState        color;
        Side              current_side;
        uint32_t          strokes_A;
        uint32_t          strokes_B;
        bool              buzzer_flash; // true = draw the brief BUZZER ON/OFF overlay
        bool              buzzer_flash_on;
    };

    void begin();
    void clear();
    void draw_boot();
    void draw_bias_cal(int seconds_remaining);
    void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset);
    void draw_set_tolerance(Tolerance tol);
    void draw_active(const ActiveView& v);          // dirty-region
    void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b, uint32_t duration_s);
    void draw_fault(FaultCode code);
    void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining);

    void set_backlight(uint8_t percent); // 0 = off, 100 = full
}
```

- [ ] **Step 10.2: Create `src/ui.cpp`**

```cpp
#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace {
    constexpr uint16_t COL_GREEN = 0x07E0;
    constexpr uint16_t COL_RED   = 0xF800;
    constexpr uint16_t COL_BLUE  = 0x001F;
    constexpr uint16_t COL_BLACK = 0x0000;
    constexpr uint16_t COL_WHITE = 0xFFFF;

    ui::ActiveView s_last {};
    bool           s_last_valid = false;

    uint16_t color_for(ColorState c) {
        switch (c) {
            case ColorState::GREEN: return COL_GREEN;
            case ColorState::BLUE:  return COL_BLUE;
            case ColorState::RED:   return COL_RED;
        }
        return COL_BLACK;
    }
}

namespace ui {

void begin() {
    M5.Display.setRotation(0);
    M5.Display.setTextWrap(false);
    clear();
}

void clear() {
    M5.Display.fillScreen(COL_BLACK);
    s_last_valid = false;
}

void draw_boot() {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 60);
    M5.Display.print("SHARPENING");
    M5.Display.setCursor(30, 85);
    M5.Display.print("GUIDE");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(45, 160);
    M5.Display.print("v0.1.0");
}

void draw_bias_cal(int seconds_remaining) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 50);
    M5.Display.print("Hold still");
    M5.Display.setCursor(10, 80);
    M5.Display.print("Calibrating");
    M5.Display.setTextSize(4);
    M5.Display.setCursor(45, 130);
    M5.Display.printf("%d", seconds_remaining);
}

void draw_set_target(float live_angle_deg, bool in_preset_mode, PresetSelection preset) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.print("SET TARGET");
    M5.Display.setTextSize(5);
    M5.Display.setCursor(20, 50);
    if (in_preset_mode) {
        if (preset == PresetSelection::CANCEL) {
            M5.Display.setTextSize(3);
            M5.Display.setCursor(10, 70);
            M5.Display.print("CANCEL");
        } else {
            M5.Display.printf("%02d", (int)preset_degrees(preset));
            M5.Display.print((char)247); // degree symbol in default font
        }
    } else {
        M5.Display.printf("%4.1f", live_angle_deg);
    }
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200);
    M5.Display.print(in_preset_mode ? "A:Pick  B:Next" : "A:Capture  B:Presets");
}

void draw_set_tolerance(Tolerance tol) {
    clear();
    M5.Display.setTextColor(COL_WHITE, COL_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 10);
    M5.Display.print("TOLERANCE");
    const char* label = "NORMAL +-2";
    switch (tol) {
        case Tolerance::TIGHT:  label = "TIGHT  +-1"; break;
        case Tolerance::NORMAL: label = "NORMAL +-2"; break;
        case Tolerance::EASY:   label = "EASY   +-3"; break;
    }
    M5.Display.setTextSize(3);
    M5.Display.setCursor(10, 90);
    M5.Display.print(label);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200);
    M5.Display.print("A:Confirm  B:Change");
}

void draw_active(const ActiveView& v) {
    // Full-screen color repaint only on transitions.
    if (!s_last_valid || s_last.color != v.color) {
        M5.Display.fillScreen(color_for(v.color));
        // Legend strip at top.
        M5.Display.fillRect(5,  5, 14, 14, COL_BLUE);
        M5.Display.fillRect(50, 5, 14, 14, COL_GREEN);
        M5.Display.fillRect(95, 5, 14, 14, COL_RED);
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(22, 9);  M5.Display.print("LOW");
        M5.Display.setCursor(67, 9);  M5.Display.print("OK");
        M5.Display.setCursor(112, 9); M5.Display.print("HIGH");
    }

    // Stroke numbers: redraw on count or side change.
    bool counts_changed =
        !s_last_valid ||
        s_last.current_side != v.current_side ||
        s_last.strokes_A != v.strokes_A ||
        s_last.strokes_B != v.strokes_B;
    if (counts_changed) {
        // Clear the number regions.
        M5.Display.fillRect(0, 90, 135, 100, color_for(v.color));
        uint32_t big = (v.current_side == Side::A) ? v.strokes_A : v.strokes_B;
        uint32_t sm  = (v.current_side == Side::A) ? v.strokes_B : v.strokes_A;
        char other_label = (v.current_side == Side::A) ? 'B' : 'A';
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(8);
        M5.Display.setCursor(30, 100);
        M5.Display.printf("%u", (unsigned)big);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(90, 215);
        M5.Display.printf("%c:%u", other_label, (unsigned)sm);
    }

    // Buzzer flash overlay: simple rectangle with text, drawn once when requested.
    if (v.buzzer_flash) {
        M5.Display.fillRect(10, 150, 115, 40, COL_BLACK);
        M5.Display.setTextColor(COL_WHITE);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(20, 160);
        M5.Display.print(v.buzzer_flash_on ? "BUZZER ON" : "BUZZER OFF");
    }

    s_last       = v;
    s_last_valid = true;
}

void draw_summary(float target_deg, Tolerance tol, uint32_t a, uint32_t b, uint32_t duration_s) {
    clear();
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(30, 5); M5.Display.print("SESSION");
    M5.Display.setTextSize(2);
    M5.Display.setCursor(5, 30);  M5.Display.printf("Target:%2d", (int)target_deg);
    const char* t = (tol == Tolerance::TIGHT) ? "T1" : (tol == Tolerance::NORMAL) ? "N2" : "E3";
    M5.Display.setCursor(5, 55);  M5.Display.printf("Tol:%s", t);
    M5.Display.setCursor(5, 85);  M5.Display.printf("A:%u", (unsigned)a);
    M5.Display.setCursor(5, 110); M5.Display.printf("B:%u", (unsigned)b);
    M5.Display.setCursor(5, 140); M5.Display.printf("%02u:%02u", (unsigned)(duration_s/60), (unsigned)(duration_s%60));
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200); M5.Display.print("A:New  B:Sleep");
}

void draw_fault(FaultCode code) {
    clear();
    M5.Display.setTextColor(COL_RED);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(5, 40);  M5.Display.print("IMU FAULT");
    M5.Display.setTextSize(3);
    M5.Display.setCursor(35, 90); M5.Display.printf("E%02u", (unsigned)code);
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 170); M5.Display.print("Power-cycle");
    M5.Display.setCursor(10, 185); M5.Display.print("to retry");
}

void draw_resume_prompt(float target_deg, Tolerance tol, uint32_t a, uint32_t b, int seconds_remaining) {
    clear();
    M5.Display.setTextColor(COL_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(20, 10); M5.Display.print("RESUME?");
    M5.Display.setTextSize(1);
    const char* t = (tol == Tolerance::TIGHT) ? "1" : (tol == Tolerance::NORMAL) ? "2" : "3";
    M5.Display.setCursor(5, 50);  M5.Display.printf("Tgt:%2d  Tol:+-%s", (int)target_deg, t);
    M5.Display.setCursor(5, 75);  M5.Display.printf("A:%u   B:%u", (unsigned)a, (unsigned)b);
    M5.Display.setTextSize(3);
    M5.Display.setCursor(55, 110); M5.Display.printf("%d", seconds_remaining);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(5, 200); M5.Display.print("A:Resume  B:New");
}

void set_backlight(uint8_t percent) {
    if (percent > 100) percent = 100;
    M5.Display.setBrightness((uint8_t)(percent * 255u / 100u));
}

} // namespace ui
#else
// Native stubs — UI isn't unit-tested; these exist so app.cpp links in tests.
namespace ui {
    void begin() {}
    void clear() {}
    void draw_boot() {}
    void draw_bias_cal(int) {}
    void draw_set_target(float, bool, PresetSelection) {}
    void draw_set_tolerance(Tolerance) {}
    void draw_active(const ActiveView&) {}
    void draw_summary(float, Tolerance, uint32_t, uint32_t, uint32_t) {}
    void draw_fault(FaultCode) {}
    void draw_resume_prompt(float, Tolerance, uint32_t, uint32_t, int) {}
    void set_backlight(uint8_t) {}
}
#endif
```

- [ ] **Step 10.3: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean. If any M5Unified method name differs (e.g. `M5.Display.fillScreen` → `drawRect`), correct per the library version.

- [ ] **Step 10.4: Commit**

```bash
git add src/ui.h src/ui.cpp
git -c commit.gpgsign=false commit -m "feat(ui): per-state screen rendering with dirty-region ACTIVE"
```

---

## Task 11: `feedback` module (LED + buzzer; Arduino-only)

**Files:**
- Create: `src/feedback.h`
- Create: `src/feedback.cpp`

- [ ] **Step 11.1: Create `src/feedback.h`**

```cpp
#pragma once
#include "types.h"

namespace feedback {
    void begin();
    void set_color(ColorState c);   // drives LED (on only when RED)
    void fault_led();               // solid on
    void beep_out_of_tolerance();   // gated by buzzer setting in caller
    void tick(uint32_t now_ms);     // service in-progress beep durations
}
```

- [ ] **Step 11.2: Create `src/feedback.cpp`**

```cpp
#include "feedback.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace {
    constexpr int   LED_PIN       = 10;      // M5StickC Plus red LED GPIO
    constexpr int   BEEP_HZ       = 2000;
    constexpr int   BEEP_MS       = 80;
    uint32_t        s_beep_until  = 0;
}

namespace feedback {

void begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // active LOW on M5StickC Plus — HIGH = off
}

void set_color(ColorState c) {
    digitalWrite(LED_PIN, (c == ColorState::RED) ? LOW : HIGH);
}

void fault_led() {
    digitalWrite(LED_PIN, LOW);
}

void beep_out_of_tolerance() {
    M5.Speaker.tone(BEEP_HZ, BEEP_MS);
    s_beep_until = millis() + BEEP_MS + 10;
}

void tick(uint32_t now_ms) {
    if (s_beep_until && now_ms > s_beep_until) {
        // M5.Speaker stops itself after the tone duration; this is a safety no-op.
        s_beep_until = 0;
    }
}

} // namespace feedback
#else
namespace feedback {
    void begin() {}
    void set_color(ColorState) {}
    void fault_led() {}
    void beep_out_of_tolerance() {}
    void tick(uint32_t) {}
}
#endif
```

- [ ] **Step 11.3: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 11.4: Commit**

```bash
git add src/feedback.h src/feedback.cpp
git -c commit.gpgsign=false commit -m "feat(feedback): LED + buzzer driver"
```

---

## Task 12: `power` module (idle, dim, deep sleep; Arduino-only)

**Files:**
- Create: `src/power.h`
- Create: `src/power.cpp`

- [ ] **Step 12.1: Create `src/power.h`**

```cpp
#pragma once
#include <cstdint>
#include "types.h"

namespace power {
    struct IdleConfig {
        uint32_t dim_ms;
        uint32_t sleep_ms;
    };

    IdleConfig config_for(State s);

    void begin();

    // Called each tick. Returns true if the caller should begin the sleep sequence.
    bool check_idle(uint32_t now_ms, State current,
                    uint32_t last_activity_ms,       // reset on meaningful input/motion
                    uint32_t last_stroke_ms);        // only used in ACTIVE

    // Perform backlight dimming; no-op if state is one with dim_ms=0 or never.
    void update_backlight(uint32_t now_ms, State current,
                          uint32_t last_activity_ms, uint32_t last_stroke_ms);

    // Actually enters deep sleep and never returns.
    [[noreturn]] void enter_deep_sleep();
}
```

- [ ] **Step 12.2: Create `src/power.cpp`**

```cpp
#include "power.h"
#include "ui.h"

#ifndef UNIT_TEST
#include <M5Unified.h>
#include <esp_sleep.h>
#endif

namespace power {

IdleConfig config_for(State s) {
    switch (s) {
        case State::BOOT:
        case State::BIAS_CAL:
        case State::FAULT:
        case State::RESUME_PROMPT:
        case State::SLEEP:
            return {0, 0}; // no idle sleep here
        case State::SET_TARGET:     return { 90'000, 120'000};
        case State::SET_TOLERANCE:  return { 60'000,  90'000};
        case State::ACTIVE:         return {180'000, 300'000}; // strokes-based, not motion
        case State::SUMMARY:        return { 60'000,  90'000};
    }
    return {0, 0};
}

void begin() {}

bool check_idle(uint32_t now_ms, State current,
                uint32_t last_activity_ms, uint32_t last_stroke_ms)
{
    auto cfg = config_for(current);
    if (cfg.sleep_ms == 0) return false;

    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    return (now_ms - reference) >= cfg.sleep_ms;
}

void update_backlight(uint32_t now_ms, State current,
                      uint32_t last_activity_ms, uint32_t last_stroke_ms)
{
    auto cfg = config_for(current);
    if (cfg.dim_ms == 0) {
        ui::set_backlight(100);
        return;
    }
    uint32_t reference = (current == State::ACTIVE) ? last_stroke_ms : last_activity_ms;
    uint32_t idle = now_ms - reference;
    uint8_t pct = (idle >= cfg.dim_ms) ? 10 : 100;
    ui::set_backlight(pct);
}

#ifndef UNIT_TEST
[[noreturn]] void enter_deep_sleep() {
    // Cut backlight power via AXP192 LDO3 before deep sleep (M5StickC Plus quirk).
    M5.Power.setBacklight(false);
    // Wake only on the AXP192 power key path, which M5Unified's Power class routes
    // through its internal ISR; ESP32 deep-sleep with EXT1 on GPIO35 is the documented
    // equivalent for M5StickC Plus (BtnA on M5StickC Plus is IO37; the AXP power key is
    // routed via AXP interrupt line on IO35 depending on board rev).
    const uint64_t WAKE_MASK = (1ULL << 35);
    esp_sleep_enable_ext1_wakeup(WAKE_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
    while (true) {} // unreachable
}
#else
[[noreturn]] void enter_deep_sleep() {
    while (true) {} // native stub
}
#endif

} // namespace power
```

- [ ] **Step 12.3: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean. If the wake GPIO for power-key differs on your board revision, adjust `WAKE_MASK` — the bring-up doc tracks this as a known unknown.

- [ ] **Step 12.4: Commit**

```bash
git add src/power.h src/power.cpp
git -c commit.gpgsign=false commit -m "feat(power): per-state idle/dim thresholds + deep sleep entry"
```

---

## Task 13: `app` state machine (TDD with mocks)

This is the biggest task. The state machine owns transitions. We test it by driving synthetic inputs (events + timestamps + gravity samples) and asserting state changes and module calls.

**Files:**
- Create: `src/app.h`
- Create: `src/app.cpp`
- Create: `test/test_app/test_app.cpp`

- [ ] **Step 13.1: Create `src/app.h`**

```cpp
#pragma once
#include "types.h"
#include "stroke.h"
#include "side.h"
#include "filter.h"

class App {
public:
    struct Tick {
        uint32_t    now_ms;
        InputEvent  input;
        Vec3        accel_g;
        Vec3        gyro_dps;
        FaultCode   imu_fault;          // NONE on a normal tick
    };

    void      begin(bool had_session_in_rtc_ram);
    void      on_tick(const Tick& t);
    State     current() const { return state_; }

    // Test-only accessors
    float     target_deg()   const { return target_deg_; }
    Tolerance tolerance()    const { return tol_; }
    bool      buzzer_on()    const { return buzzer_on_; }
    uint32_t  strokes_a()    const { return strokes_a_; }
    uint32_t  strokes_b()    const { return strokes_b_; }
    Side      current_side() const { return side_fsm_.current_side(); }

private:
    void transition(State to, uint32_t now_ms);
    void handle_set_target  (const Tick& t);
    void handle_set_tolerance(const Tick& t);
    void handle_active      (const Tick& t);
    void handle_summary     (const Tick& t);
    void handle_bias_cal    (const Tick& t);
    void handle_resume_prompt(const Tick& t);

    State            state_                = State::BOOT;
    uint32_t         state_entered_ms_     = 0;
    uint32_t         last_activity_ms_     = 0;
    uint32_t         last_stroke_ms_       = 0;

    // Session state in-memory (mirrored to RTC RAM via session module at key transitions).
    float            target_deg_           = 17.0f;
    Tolerance        tol_                  = Tolerance::NORMAL;
    bool             buzzer_on_            = false;
    Vec3             g_ref_                = {0,0,-1};
    uint32_t         strokes_a_            = 0;
    uint32_t         strokes_b_            = 0;
    uint32_t         session_started_ms_   = 0;

    // SET_TARGET preset sub-state
    bool             in_preset_mode_       = false;
    PresetSelection  preset_selection_     = PresetSelection::P17;

    // Buzzer flash overlay state
    uint32_t         buzzer_flash_until_   = 0;
    bool             buzzer_flash_showing_ = false;

    // Submodule state
    MahonyFilter filter_;
    StrokeFSM    stroke_fsm_;
    SideFSM      side_fsm_;
    FaultCode    fault_code_           = FaultCode::NONE;
};
```

- [ ] **Step 13.2: Write failing tests**

Create `./test/test_app/test_app.cpp`:

```cpp
#include <unity.h>
#include "app.h"
#include "settings.h"
#include "session.h"

static void advance(App& a, uint32_t& t, uint32_t dt_ms, InputEvent ev = InputEvent::NONE,
                    Vec3 accel = {0,0,-1}, Vec3 gyro = {0,0,0})
{
    uint32_t end = t + dt_ms;
    bool emitted = false;
    while (t < end) {
        t += 10;
        App::Tick tick{t, (emitted ? InputEvent::NONE : ev), accel, gyro, FaultCode::NONE};
        emitted = true;
        a.on_tick(tick);
    }
}

void setUp(void) {
    // Reset settings & session between tests.
    settings::save_tolerance(Tolerance::NORMAL);
    settings::save_buzzer(false);
    settings::clear_first_boot();
    session::clear();
}
void tearDown(void) {}

void test_boot_without_session_goes_to_set_target(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);  // past splash (2s)
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_boot_with_session_goes_to_resume_prompt(void) {
    App a;
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 2100);
    TEST_ASSERT_EQUAL_INT((int)State::RESUME_PROMPT, (int)a.current());
}

void test_resume_prompt_a_confirms_active(void) {
    App a;
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_resume_prompt_b_starts_new_session(void) {
    App a;
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::B_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_resume_prompt_times_out_to_set_target(void) {
    App a;
    a.begin(true);
    uint32_t t = 0;
    advance(a, t, 2100);       // into RESUME_PROMPT
    advance(a, t, 5500);       // > 5s timeout
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_set_target_a_captures_and_advances(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_set_target_b_enters_preset_mode_and_cycles(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::B_SHORT); // enter preset mode, first preset is P12
    advance(a, t, 100, InputEvent::B_SHORT); // cycles to P15
    advance(a, t, 100, InputEvent::A_SHORT); // confirm P15
    TEST_ASSERT_EQUAL_FLOAT(15.0f, a.target_deg());
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_preset_cancel_returns_to_live_capture(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    // Cycle through all 5 presets then CANCEL
    for (int i = 0; i < 6; i++) advance(a, t, 100, InputEvent::B_SHORT); // end on CANCEL
    advance(a, t, 100, InputEvent::A_SHORT); // confirm CANCEL → exit preset mode
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
    // Subsequent A_SHORT should capture live, not select a preset.
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TOLERANCE, (int)a.current());
}

void test_tolerance_a_confirms_and_advances(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT); // capture
    advance(a, t, 100, InputEvent::A_SHORT); // confirm tolerance
    TEST_ASSERT_EQUAL_INT((int)State::ACTIVE, (int)a.current());
}

void test_active_long_a_goes_to_summary(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_LONG);
    TEST_ASSERT_EQUAL_INT((int)State::SUMMARY, (int)a.current());
}

void test_summary_a_starts_new_session(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_LONG);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_EQUAL_INT((int)State::SET_TARGET, (int)a.current());
}

void test_active_long_b_toggles_buzzer_persistently(void) {
    App a;
    a.begin(false);
    uint32_t t = 0;
    advance(a, t, 2100);
    advance(a, t, 100, InputEvent::A_SHORT);
    advance(a, t, 100, InputEvent::A_SHORT);
    TEST_ASSERT_FALSE(a.buzzer_on());
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_TRUE(a.buzzer_on());
    TEST_ASSERT_TRUE(settings::load_buzzer());
    advance(a, t, 100, InputEvent::B_LONG);
    TEST_ASSERT_FALSE(a.buzzer_on());
    TEST_ASSERT_FALSE(settings::load_buzzer());
}

void test_imu_fault_at_boot_goes_to_fault(void) {
    App a;
    a.begin(false);
    uint32_t t = 50;
    App::Tick tick{t, InputEvent::NONE, {0,0,-1}, {0,0,0}, FaultCode::E01_BEGIN_FAILED};
    a.on_tick(tick);
    TEST_ASSERT_EQUAL_INT((int)State::FAULT, (int)a.current());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_without_session_goes_to_set_target);
    RUN_TEST(test_boot_with_session_goes_to_resume_prompt);
    RUN_TEST(test_resume_prompt_a_confirms_active);
    RUN_TEST(test_resume_prompt_b_starts_new_session);
    RUN_TEST(test_resume_prompt_times_out_to_set_target);
    RUN_TEST(test_set_target_a_captures_and_advances);
    RUN_TEST(test_set_target_b_enters_preset_mode_and_cycles);
    RUN_TEST(test_preset_cancel_returns_to_live_capture);
    RUN_TEST(test_tolerance_a_confirms_and_advances);
    RUN_TEST(test_active_long_a_goes_to_summary);
    RUN_TEST(test_summary_a_starts_new_session);
    RUN_TEST(test_active_long_b_toggles_buzzer_persistently);
    RUN_TEST(test_imu_fault_at_boot_goes_to_fault);
    return UNITY_END();
}
```

- [ ] **Step 13.3: Run tests — fail compile**

Run: `pio test -e native -f test_app`
Expected: compile error, `app.cpp` doesn't exist.

- [ ] **Step 13.4: Create `src/app.cpp`**

```cpp
#include "app.h"
#include "angle.h"
#include "ui.h"
#include "feedback.h"
#include "settings.h"
#include "session.h"
#include <cmath>

static PresetSelection next_preset(PresetSelection p) {
    switch (p) {
        case PresetSelection::P12:    return PresetSelection::P15;
        case PresetSelection::P15:    return PresetSelection::P17;
        case PresetSelection::P17:    return PresetSelection::P20;
        case PresetSelection::P20:    return PresetSelection::P22;
        case PresetSelection::P22:    return PresetSelection::CANCEL;
        case PresetSelection::CANCEL: return PresetSelection::P12;
    }
    return PresetSelection::P12;
}

void App::begin(bool had_session_in_rtc_ram) {
    settings::begin();
    session::begin();
    filter_.begin(100.0f);
    filter_.set_bias(settings::load_gyro_bias());
    buzzer_on_ = settings::load_buzzer();
    tol_       = settings::load_tolerance();

    if (settings::is_first_boot()) {
        transition(State::BOOT, 0);
        // After splash, we'll route into BIAS_CAL in handle logic.
    } else if (had_session_in_rtc_ram) {
        // Pull session snapshot to restore numbers.
        const auto& s = session::state();
        target_deg_ = s.target_deg;
        tol_        = s.tolerance;
        g_ref_      = s.g_ref;
        strokes_a_  = s.strokes_A;
        strokes_b_  = s.strokes_B;
        session_started_ms_ = s.session_started_ms;
        transition(State::BOOT, 0);
    } else {
        transition(State::BOOT, 0);
    }
}

void App::transition(State to, uint32_t now_ms) {
    state_             = to;
    state_entered_ms_  = now_ms;
    last_activity_ms_  = now_ms;
    last_stroke_ms_    = now_ms;

    switch (to) {
        case State::BOOT:          ui::draw_boot(); break;
        case State::BIAS_CAL:      ui::draw_bias_cal(10); break;
        case State::SET_TARGET:
            in_preset_mode_  = false;
            preset_selection_ = PresetSelection::P12;
            break;
        case State::SET_TOLERANCE: break;
        case State::ACTIVE:
            stroke_fsm_.reset();
            side_fsm_.reset();
            if (session_started_ms_ == 0) session_started_ms_ = now_ms;
            {
                SessionState ss;
                ss.target_deg = target_deg_;
                ss.tolerance  = tol_;
                ss.g_ref      = g_ref_;
                ss.strokes_A  = strokes_a_;
                ss.strokes_B  = strokes_b_;
                ss.current_side = side_fsm_.current_side();
                ss.session_started_ms = session_started_ms_;
                session::mark_active(ss);
            }
            break;
        case State::SUMMARY: break;
        case State::FAULT:   ui::draw_fault(fault_code_); feedback::fault_led(); break;
        case State::RESUME_PROMPT: break;
        case State::SLEEP:   break;
    }
}

void App::handle_bias_cal(const Tick& t) {
    // Stay in BIAS_CAL for 10s of stillness. If motion, restart countdown.
    float mag = std::sqrt(t.accel_g.x*t.accel_g.x + t.accel_g.y*t.accel_g.y + t.accel_g.z*t.accel_g.z);
    if (std::fabs(mag - 1.0f) > 0.15f) {
        state_entered_ms_ = t.now_ms; // restart countdown
    }
    if (t.now_ms - state_entered_ms_ >= 10'000) {
        // In a real build we'd accumulate during BIAS_CAL; for now, use the last bias
        // written by the IMU module via settings (main.cpp calls imu::capture_gyro_bias
        // in the Arduino path before handing control to App).
        filter_.set_bias(settings::load_gyro_bias());
        settings::clear_first_boot();
        transition(State::SET_TARGET, t.now_ms);
    }
}

void App::handle_set_target(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        if (in_preset_mode_) {
            if (preset_selection_ == PresetSelection::CANCEL) {
                in_preset_mode_ = false;
            } else {
                target_deg_ = preset_degrees(preset_selection_);
                // Synthesize a g_ref consistent with the chosen angle (used only if user
                // never goes into ACTIVE and captures; at ACTIVE entry the filter's live
                // gravity becomes g_ref).
                g_ref_ = {std::cos(target_deg_ * (float)M_PI / 180.0f), 0.0f,
                          -std::sin(target_deg_ * (float)M_PI / 180.0f)};
                transition(State::SET_TOLERANCE, t.now_ms);
            }
        } else {
            // Capture live gravity from filter.
            g_ref_ = filter_.gravity();
            // Compute displayed capture angle purely for target_deg_ bookkeeping.
            float sinv = -g_ref_.z; // with n_back = (0,0,-1), dot = -g.z => sin(angle)
            if (sinv >  1.0f) sinv =  1.0f;
            if (sinv < -1.0f) sinv = -1.0f;
            target_deg_ = std::asin(sinv) * (180.0f / (float)M_PI);
            transition(State::SET_TOLERANCE, t.now_ms);
        }
    } else if (t.input == InputEvent::B_SHORT) {
        if (!in_preset_mode_) {
            in_preset_mode_   = true;
            preset_selection_ = PresetSelection::P12;
        } else {
            preset_selection_ = next_preset(preset_selection_);
        }
    }
}

void App::handle_set_tolerance(const Tick& t) {
    if (t.input == InputEvent::B_SHORT) {
        int v = (int)tol_;
        v = (v + 1) % 3;
        tol_ = (Tolerance)v;
    } else if (t.input == InputEvent::A_SHORT) {
        settings::save_tolerance(tol_);
        transition(State::ACTIVE, t.now_ms);
    }
}

void App::handle_active(const Tick& t) {
    // Update filter & derived gravity.
    filter_.update(t.gyro_dps, t.accel_g);
    Vec3 g_now = filter_.gravity();

    // Angle + classification.
    AngleResult ar = compute_angle(g_ref_, g_now);
    ColorState col = classify(ar, tolerance_degrees(tol_));

    // Stroke FSM
    bool in_tol = (col == ColorState::GREEN);
    uint32_t count_before = stroke_fsm_.stroke_count();
    stroke_fsm_.update(t.now_ms, in_tol);
    if (stroke_fsm_.stroke_count() > count_before) {
        if (side_fsm_.current_side() == Side::A) strokes_a_++;
        else                                      strokes_b_++;
        last_stroke_ms_ = t.now_ms;
    }

    // Side FSM
    float accel_mag = std::sqrt(t.accel_g.x*t.accel_g.x + t.accel_g.y*t.accel_g.y + t.accel_g.z*t.accel_g.z);
    float grav_dot_ref = g_now.x*g_ref_.x + g_now.y*g_ref_.y + g_now.z*g_ref_.z;
    side_fsm_.update(t.now_ms, accel_mag, grav_dot_ref);
    if (side_fsm_.consume_switch()) {
        // Side changed; stroke FSM should also reset to avoid carry-over noise.
        stroke_fsm_.reset();
    }

    // Inputs
    if (t.input == InputEvent::A_LONG) {
        transition(State::SUMMARY, t.now_ms);
        return;
    }
    if (t.input == InputEvent::B_SHORT) {
        side_fsm_.manual_toggle(t.now_ms);
        side_fsm_.consume_switch(); // acknowledge so downstream doesn't re-fire
        stroke_fsm_.reset();
    }
    if (t.input == InputEvent::B_LONG) {
        buzzer_on_ = !buzzer_on_;
        settings::save_buzzer(buzzer_on_);
        buzzer_flash_until_  = t.now_ms + 800;
        buzzer_flash_showing_ = true;
    } else if (buzzer_flash_showing_ && t.now_ms > buzzer_flash_until_) {
        buzzer_flash_showing_ = false;
    }

    // Feedback channels
    feedback::set_color(col);
    if (buzzer_on_ && col != ColorState::GREEN) {
        feedback::beep_out_of_tolerance();
    }

    // Render
    ui::ActiveView v{ col,
                      side_fsm_.current_side(),
                      strokes_a_, strokes_b_,
                      buzzer_flash_showing_, buzzer_on_ };
    ui::draw_active(v);

    last_activity_ms_ = t.now_ms;
}

void App::handle_summary(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        strokes_a_ = strokes_b_ = 0;
        session_started_ms_ = 0;
        session::clear();
        transition(State::SET_TARGET, t.now_ms);
    } else if (t.input == InputEvent::B_SHORT) {
        transition(State::SLEEP, t.now_ms);
    }
}

void App::handle_resume_prompt(const Tick& t) {
    if (t.input == InputEvent::A_SHORT) {
        transition(State::ACTIVE, t.now_ms);
    } else if (t.input == InputEvent::B_SHORT) {
        session::clear();
        strokes_a_ = strokes_b_ = 0;
        transition(State::SET_TARGET, t.now_ms);
    } else if (t.now_ms - state_entered_ms_ >= 5000) {
        session::clear();
        strokes_a_ = strokes_b_ = 0;
        transition(State::SET_TARGET, t.now_ms);
    }
}

void App::on_tick(const Tick& t) {
    if (t.imu_fault != FaultCode::NONE && state_ != State::FAULT) {
        fault_code_ = t.imu_fault;
        transition(State::FAULT, t.now_ms);
        return;
    }

    switch (state_) {
        case State::BOOT:
            if (t.now_ms - state_entered_ms_ >= 2000) {
                if (settings::is_first_boot()) transition(State::BIAS_CAL, t.now_ms);
                else if (session::has_session()) transition(State::RESUME_PROMPT, t.now_ms);
                else transition(State::SET_TARGET, t.now_ms);
            }
            break;
        case State::BIAS_CAL:      handle_bias_cal(t); break;
        case State::SET_TARGET:    handle_set_target(t); break;
        case State::SET_TOLERANCE: handle_set_tolerance(t); break;
        case State::ACTIVE:        handle_active(t); break;
        case State::SUMMARY:       handle_summary(t); break;
        case State::RESUME_PROMPT: handle_resume_prompt(t); break;
        case State::FAULT:         break; // no escape without power-cycle
        case State::SLEEP:         break; // not actually ticked; real sleep happens in main
    }
}
```

- [ ] **Step 13.5: Run tests — verify pass**

Run: `pio test -e native -f test_app`
Expected: all 13 tests pass. If `test_boot_with_session_goes_to_resume_prompt` fails, confirm `session::begin()` is idempotent and `setUp` properly clears via `session::clear()`.

- [ ] **Step 13.6: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 13.7: Commit**

```bash
git add src/app.h src/app.cpp test/test_app/
git -c commit.gpgsign=false commit -m "feat(app): state machine with event wiring + unit tests"
```

---

## Task 14: `main.cpp` — Arduino entry point & 100 Hz scheduler

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 14.1: Rewrite `src/main.cpp`**

```cpp
#ifndef UNIT_TEST
#include <Arduino.h>
#include <M5Unified.h>

#include "app.h"
#include "imu.h"
#include "input.h"
#include "settings.h"
#include "session.h"
#include "ui.h"
#include "feedback.h"
#include "power.h"

static App       g_app;
static InputFSM  g_input;

static bool      g_had_session_in_rtc = false;
static uint32_t  g_next_tick_ms       = 0;
constexpr uint32_t TICK_PERIOD_MS     = 10;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    ui::begin();
    feedback::begin();
    power::begin();
    session::begin();

    // If the session state in RTC RAM is marked active AND we woke from deep sleep,
    // route through RESUME_PROMPT. On a power-on reset, esp_sleep_get_wakeup_cause
    // returns ESP_SLEEP_WAKEUP_UNDEFINED and the RTC RAM is uninitialized — treat
    // that case as "no session."
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    g_had_session_in_rtc =
        (cause != ESP_SLEEP_WAKEUP_UNDEFINED) && session::has_session();

    FaultCode fc = imu::begin();

    // If this is a first boot, run the bias capture BEFORE handing control to the app.
    if (fc == FaultCode::NONE && settings::is_first_boot()) {
        ui::draw_bias_cal(10);
        Vec3 bias;
        if (imu::capture_gyro_bias(bias)) {
            settings::save_gyro_bias(bias);
            // clear_first_boot() is called inside App::handle_bias_cal after the 10s UI
            // countdown concludes; we don't clear here so the splash shows once more.
        }
    }

    g_app.begin(g_had_session_in_rtc);
    g_next_tick_ms = millis();
}

void loop() {
    uint32_t now = millis();
    if ((int32_t)(now - g_next_tick_ms) < 0) {
        delay(1);
        return;
    }
    g_next_tick_ms += TICK_PERIOD_MS;

    M5.update();
    bool a_pressed = M5.BtnA.isPressed();
    bool b_pressed = M5.BtnB.isPressed();
    InputEvent ev = g_input.update(now, a_pressed, b_pressed);

    Vec3 accel, gyro;
    FaultCode fault = FaultCode::NONE;
    if (!imu::read(accel, gyro)) {
        fault = FaultCode::E02_SELF_TEST_FAILED;
        accel = {0,0,-1};
        gyro  = {0,0,0};
    }

    App::Tick tick{now, ev, accel, gyro, fault};
    g_app.on_tick(tick);

    feedback::tick(now);

    // Idle detection & sleep
    if (power::check_idle(now, g_app.current(), /*last_activity_ms*/ now, /*last_stroke_ms*/ now)) {
        // In a later refinement: App should publish its own last_activity_ms + last_stroke_ms;
        // for v1, the App internally transitions to SUMMARY/SLEEP on stroke inactivity by
        // watching its own counters.
        power::enter_deep_sleep();
    }
    power::update_backlight(now, g_app.current(), now, now);
}
#endif
```

- [ ] **Step 14.2: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 14.3: Verify all tests still pass**

Run: `pio test -e native`
Expected: all previous tests still green.

- [ ] **Step 14.4: Commit**

```bash
git add src/main.cpp
git -c commit.gpgsign=false commit -m "feat(main): Arduino setup/loop with 100 Hz scheduler"
```

---

## Task 15: Refine `App` idle/stroke accounting for accurate sleep trigger

The MVP-correct idle sleep behavior for ACTIVE is "sleep after 5 min with no new strokes." Task 14 stubbed this with `now` everywhere. Plumbing real last-stroke timestamps through means App must expose them.

**Files:**
- Modify: `src/app.h` (add accessors)
- Modify: `src/main.cpp` (use them)

- [ ] **Step 15.1: Add accessors to `src/app.h`**

Add inside the public section of class `App`:

```cpp
uint32_t last_activity_ms() const { return last_activity_ms_; }
uint32_t last_stroke_ms()   const { return last_stroke_ms_; }
```

- [ ] **Step 15.2: Update `src/main.cpp`**

Replace the `power::check_idle` and `power::update_backlight` calls in `loop()`:

```cpp
    if (power::check_idle(now, g_app.current(),
                          g_app.last_activity_ms(),
                          g_app.last_stroke_ms())) {
        power::enter_deep_sleep();
    }
    power::update_backlight(now, g_app.current(),
                            g_app.last_activity_ms(),
                            g_app.last_stroke_ms());
```

- [ ] **Step 15.3: Verify target build**

Run: `pio run -e m5stick-c-plus`
Expected: clean.

- [ ] **Step 15.4: Verify tests still pass**

Run: `pio test -e native`
Expected: all green (no tests touch these accessors yet).

- [ ] **Step 15.5: Commit**

```bash
git add src/app.h src/main.cpp
git -c commit.gpgsign=false commit -m "refactor(main): use App's real last-activity/stroke timestamps for idle"
```

---

## Task 16: Hardware bring-up checklist document

Once hardware arrives, a structured walkthrough resolves the known risks documented in the spec.

**Files:**
- Create: `docs/superpowers/bringup/2026-04-23-hardware-bringup.md`

- [ ] **Step 16.1: Create the bring-up checklist**

Create file `./docs/superpowers/bringup/2026-04-23-hardware-bringup.md`:

```markdown
# Hardware Bring-Up — Sharpening Guide v1

Run once, in order, when the M5StickC Plus Watch Kit arrives. Each step has an expected result; record the actual observation. Any mismatch gates the next step until resolved.

## Pre-flight

- [ ] USB-C cable connected, `pio run -e m5stick-c-plus -t upload` flashes firmware successfully.
- [ ] Serial monitor shows no panic/reset loop.

## 1. I²C sanity (spec risk #3)

- [ ] Boot with fresh flash. Device should enter BIAS_CAL (first-boot flag set).
- [ ] If instead you see FAULT with code `E03`, the MPU6886 / AXP192 I²C address conflict is active on this unit. Workaround options:
  1. Try a different M5StickC Plus firmware baseline (erase flash via `esptool.py --chip esp32 erase_flash` then re-flash).
  2. Explicitly pin `M5.Imu.begin(Wire1)` with a secondary I²C bus if M5Unified supports it on this board revision.
  3. Fall back to direct-register access via `Wire` after manual mutex with AXP192 reads.
- [ ] If FAULT shows `E01`: `M5.Imu.isEnabled()` returned false — hardware defect or wrong board identity. Confirm `board = m5stick-c` in `platformio.ini`.

## 2. Gyro bias capture

- [ ] BIAS_CAL screen counts down 10 seconds while device sits motionless on the bench.
- [ ] Picking up the device restarts the countdown.
- [ ] After 10 s, device transitions to SET_TARGET.
- [ ] Power-cycle: confirm BIAS_CAL does NOT re-run (first-boot flag cleared).

## 3. Orientation convention validation (spec §4.3)

- [ ] Lay device screen-up on a level surface. From serial debug print: confirm accel Z reads ≈ −1.0 g. If it reads +1.0 g, flip the sign of `N_BACK` in `src/angle.h` from `{0,0,-1}` to `{0,0,1}` and re-verify.
- [ ] Tilt device 45° nose-up: accel X should go to roughly −0.7 g, Z to roughly −0.7 g.

## 4. Angle display smoke test

- [ ] In SET_TARGET, the live-angle readout should:
  - Be ~0° when device lies flat on the table.
  - Increase smoothly as you tilt the device up by rotating around the long axis.
  - Reach ~90° when device is vertical.

## 5. Capture + active feedback

- [ ] Capture at ~15° (hold device at 15° against a protractor or phone inclinometer, press A).
- [ ] Advance to NORMAL tolerance. Active screen should be green while held near capture angle.
- [ ] Tilt further up: screen should go RED.
- [ ] Tilt back down past capture: screen should go BLUE.
- [ ] Confirm the directional sign is correct (RED = higher angle than target). If reversed: flip the `N_BACK` sign constant. Re-run item 3.

## 6. Stroke count

- [ ] Hold within tolerance >300 ms, drift out >200 ms → stroke count increments by 1.
- [ ] A stroke done with <300 ms hold does not increment.
- [ ] A brief <200 ms wobble mid-pass does not end the stroke.

## 7. Side switch

- [ ] Peel the device off the test surface (or simulate with a ~1 g+ shake). Stick it back with the opposite orientation (180° flipped).
- [ ] Expected: one side switch event within ~0.5 s of the device settling. Other side's count becomes the active one.
- [ ] Pressing B during ACTIVE forces a manual side toggle. Test that an auto switch does NOT follow during the 2 s suppression window.

## 8. Buzzer toggle

- [ ] In ACTIVE, long-press B. Expected: "BUZZER ON" overlay briefly, then returns to normal.
- [ ] Cause an out-of-tolerance transition — expect an audible beep.
- [ ] Long-press B again — "BUZZER OFF" overlay; subsequent out-of-tolerance transitions produce no beep.
- [ ] Power-cycle: the previously-saved buzzer state persists.

## 9. Deep sleep + wake

- [ ] From SET_TARGET, wait 2 min with no input/motion. Expect backlight dim at 90 s, sleep at 120 s.
- [ ] Press power-key: device wakes to SET_TARGET. If it does NOT wake, the GPIO wake mask in `src/power.cpp` is wrong for this board revision — try `(1ULL << 37)` (BtnA IO) or consult `.pio/libdeps/.../M5Unified/src/utility/Power_Class.cpp` for the AXP192 IRQ routing on this board.
- [ ] From ACTIVE with at least one captured angle, long-press A → SUMMARY → wait 5 min with no strokes → sleep → press power-key → expect RESUME_PROMPT.

## 10. Mahony gain tuning (spec risk #1)

- [ ] Sharpen a real knife on a real stone for 2–3 minutes with the device attached.
- [ ] Compare observed green/red/blue transitions against expected sharpening motion.
- [ ] If the filter appears sluggish (color changes lag reality by >200 ms), raise `Kp` in `MahonyFilter::begin` (e.g., 0.5 → 1.0).
- [ ] If the filter appears jumpy (false color transitions during smooth strokes), lower `Kp` and/or raise `Ki` (e.g., 0 → 0.05).
- [ ] Record the final tuned `Kp` / `Ki` in a git commit and re-flash.

## 11. Stroke threshold tuning (spec risk #2)

- [ ] From a recorded real session, log `in_tolerance` bool with timestamps. Identify actual hold durations and gap durations.
- [ ] Adjust `StrokeFSM::IN_MIN_MS` / `OUT_MIN_MS` in `src/stroke.h` if observed ranges differ substantially from 300 ms / 200 ms. Re-run `pio test -e native -f test_stroke` to ensure the test fixtures still pass with the new thresholds.

## 12. Magnet mount (spec risk #5)

- [ ] Glue a neodymium magnet (≥10 mm x 5 mm N35 or stronger) to the back of the device with 5-min epoxy or high-strength double-sided tape.
- [ ] Attach to a typical kitchen knife flat. The device must not shift during normal sharpening motion.
- [ ] If slippage occurs, upgrade to a larger or stronger magnet, or add a thin rubber grip layer.

## Sign-off

Date hardware bring-up completed: ____________
Any outstanding spec risk unresolved: ____________
Any spec/plan amendment required as a result: ____________
```

- [ ] **Step 16.2: Commit**

```bash
git add docs/superpowers/bringup/
git -c commit.gpgsign=false commit -m "docs: hardware bring-up checklist for post-delivery validation"
```

---

## Self-review

- **Spec coverage:**
  - Section 3.1 states → Task 13 handles all transitions; Task 9 handles FAULT entry; Task 14 wires boot path.
  - Section 3.2 screens → Task 10 implements each.
  - Section 3.3 LED + buzzer → Task 11.
  - Section 3.4 persistent settings → Task 7.
  - Section 3.5 RTC RAM session → Task 8.
  - Section 4.1 loop → Task 14.
  - Section 4.2 Mahony → Task 5.
  - Section 4.3 angle math → Task 2.
  - Section 4.4 stroke FSM + side FSM → Tasks 3, 4.
  - Section 4.5 dirty-region rendering → Task 10.
  - Section 4.6 power management → Task 12 + Task 15.
  - Section 4.7 modules → all.
  - Section 5 build env → Task 1.
  - Section 6 testing → tasks 2, 3, 4, 5, 6, 13 provide desktop tests; Task 16 provides bring-up.
  - All of section 8 risks → referenced in Task 16.
  - Appendix 9 decisions → all implemented.
- **Placeholder scan:** none.
- **Type consistency:** `StrokeFSM::stroke_count()`, `SideFSM::current_side()`, `App::current()` used consistently across tasks.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-04-23-sharpening-guide-implementation.md`. Two execution options:

**1. Subagent-Driven (recommended)** — A fresh subagent is dispatched per task with no carryover context; I review between tasks. Faster iteration, better isolation per task.

**2. Inline Execution** — Execute tasks in this session using the executing-plans sub-skill; batch execution with checkpoints for review.

Which approach?
