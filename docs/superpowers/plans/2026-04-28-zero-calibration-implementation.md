# Zero-Angle Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the per-session `ZERO_CAL` state defined in `docs/superpowers/specs/2026-04-28-zero-calibration-design.md`, replacing the synthesized world-horizontal `g_ref` with two measured zeros (`g_zero_A`, `g_zero_B`) per blade side.

**Architecture:** New pure-logic module `zero_cal` (capture FSM + stillness gate + averager) unit-tested on native. Integration into the state machine via a new `State::ZERO_CAL` and substate enum. Existing modules (`angle`, `side`, `session`, `app`, `ui`, `imu`) get targeted edits — `angle.cpp`'s direction-sign trick (`N_BACK`) stays as-is. The synthesized preset `g_ref = {cos(r), 0, -sin(r)}` and the freehand `asin(-g_ref.z)` paths in `SET_TARGET` are removed.

**Tech Stack:** Same as the parent project — Arduino-ESP32, PlatformIO, Unity, M5Unified, Mahony AHRS (existing). No new `lib_deps`.

---

## File Structure

```
./
├── src/
│   ├── types.h                    # MODIFY: add State::ZERO_CAL, ZeroCalSubstate, ZeroCalCfg
│   ├── zero_cal.h / zero_cal.cpp  # NEW: pure capture FSM + stillness gate + averager
│   ├── angle.h / angle.cpp        # MODIFY: classify() signature
│   ├── session.h / session.cpp    # MODIFY: replace g_ref with g_zero_A, g_zero_B
│   ├── app.h / app.cpp            # MODIFY: state machine wiring + remove old g_ref paths
│   ├── side.cpp                   # NO CODE CHANGE: only call site moves to g_zero_A
│   ├── imu.h / imu.cpp            # MODIFY (if needed): expose raw_accel() bypass
│   ├── ui.h / ui.cpp              # MODIFY: draw_zero_cal_prompt + draw_zero_cal_progress
│   └── main.cpp                   # NO CODE CHANGE expected
├── test/
│   ├── test_angle/test_angle.cpp        # MODIFY: classifier signature
│   ├── test_zero_cal/test_zero_cal.cpp  # NEW: capture FSM + stillness + averager
│   └── test_app/test_app.cpp            # MODIFY: ZERO_CAL transitions, remove g_ref tests
└── docs/
    └── superpowers/
        ├── specs/2026-04-28-zero-calibration-design.md   # already committed
        ├── plans/2026-04-28-zero-calibration-implementation.md  # this file
        └── bringup/2026-04-23-hardware-bringup.md         # MODIFY: add §9.2 from spec
```

**Responsibilities:**
- `zero_cal.h/cpp` — pure C++. Runs offline against arbitrary sample streams. Three concerns: stillness gate (single-sample test), running averager (accumulator + reset), capture FSM (substates: WARMUP / AVERAGING / DONE). No Arduino dependencies.
- `angle.cpp` — `compute_angle` body unchanged. `classify()` becomes `classify(magnitude_deg, target_deg, tolerance_deg, direction_sign)`.
- `session.cpp` — RTC RAM struct loses `g_ref`, gains `g_zero_A` and `g_zero_B`.
- `app.cpp` — adds `handle_zero_cal()`. `handle_set_target()` simplified (target is just a scalar). `handle_active()` selects `g_zero_active` by side and uses it as the first arg to `compute_angle`. The side-FSM call site uses `dot(g_now, g_zero_A_)` for `grav_dot_ref` regardless of current side.
- `imu.cpp` — already exposes raw accel via `read(accel_g, gyro_dps)`. Reuse that; no new accessor needed.
- `ui.cpp` — two new render functions; reuses existing M5GFX layout conventions.

---

## Task 1: Add ZERO_CAL types

**Files:**
- Modify: `src/types.h`

- [ ] **Step 1.1: Add `State::ZERO_CAL` and the substate enum**

Edit `src/types.h` — add `ZERO_CAL` to the `State` enum after `SET_TOLERANCE`, and add a new `ZeroCalSubstate` enum.

```cpp
enum class State : uint8_t {
    BOOT,
    BIAS_CAL,
    SET_TARGET,
    SET_TOLERANCE,
    ZERO_CAL,                  // NEW
    ACTIVE,
    SUMMARY,
    FAULT,
    RESUME_PROMPT,
    SLEEP
};

enum class ZeroCalSubstate : uint8_t {
    PROMPT_A,
    CAPTURE_A,
    PROMPT_B,
    CAPTURE_B,
    DONE
};
```

- [ ] **Step 1.2: Build native to verify enum compiles**

Run: `pio run -e native`
Expected: build completes (no test changes yet, just enum addition).

- [ ] **Step 1.3: Commit**

```bash
git add src/types.h
git commit -m "feat(types): add State::ZERO_CAL and ZeroCalSubstate"
```

---

## Task 2: Pure stillness gate

**Files:**
- Create: `src/zero_cal.h`
- Create: `src/zero_cal.cpp`
- Create: `test/test_zero_cal/test_zero_cal.cpp`

- [ ] **Step 2.1: Write the failing test for the stillness gate**

Create `test/test_zero_cal/test_zero_cal.cpp`:

```cpp
#include <unity.h>
#include "zero_cal.h"

void setUp(void) {}
void tearDown(void) {}

void test_still_sample_passes_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.0f};   // 1g magnitude
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};   // no rotation
    TEST_ASSERT_TRUE(zero_cal::is_still_instant(accel, gyro));
}

void test_high_gyro_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.0f};
    Vec3 gyro  = {0.0f, 0.0f,  1.0f};   // 1 dps > 0.5 dps threshold
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

void test_off_gravity_magnitude_fails_gate(void) {
    Vec3 accel = {0.0f, 0.0f, -1.05f};  // |a| - 1g = 0.05 > 0.01g
    Vec3 gyro  = {0.0f, 0.0f,  0.0f};
    TEST_ASSERT_FALSE(zero_cal::is_still_instant(accel, gyro));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_still_sample_passes_gate);
    RUN_TEST(test_high_gyro_fails_gate);
    RUN_TEST(test_off_gravity_magnitude_fails_gate);
    return UNITY_END();
}
```

- [ ] **Step 2.2: Run the test — should fail (header missing)**

Run: `pio test -e native -f test_zero_cal`
Expected: FAIL — `zero_cal.h` not found.

- [ ] **Step 2.3: Create `src/zero_cal.h`**

```cpp
#pragma once
#include "types.h"

namespace zero_cal {

// Stillness thresholds (per spec 2026-04-28 §4):
//   |‖a‖ − 1.0g| < 0.01g
//   |gyro| < 0.5 dps
// Per-axis stddev gate (spec §4 third bullet) is deferred to v2 — the magnitude
// + gyro pair is sufficient for realistic bench sharpening setups. Revisit if
// hardware bring-up shows false-positive captures.
constexpr float STILL_ACCEL_MAG_TOL_G = 0.01f;
constexpr float STILL_GYRO_MAG_DPS    = 0.5f;

bool is_still_instant(Vec3 accel_g, Vec3 gyro_dps);

}  // namespace zero_cal
```

- [ ] **Step 2.4: Create `src/zero_cal.cpp`**

```cpp
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
```

- [ ] **Step 2.5: Run tests — should pass**

Run: `pio test -e native -f test_zero_cal`
Expected: 3 PASS.

- [ ] **Step 2.6: Commit**

```bash
git add src/zero_cal.h src/zero_cal.cpp test/test_zero_cal/test_zero_cal.cpp
git commit -m "feat(zero_cal): pure single-sample stillness gate"
```

---

## Task 3: Capture FSM with running averager

**Files:**
- Modify: `src/zero_cal.h`, `src/zero_cal.cpp`
- Modify: `test/test_zero_cal/test_zero_cal.cpp`

- [ ] **Step 3.1: Append failing tests for the capture FSM**

Add to `test/test_zero_cal/test_zero_cal.cpp` (before `main`):

```cpp
static Vec3 still_accel = {0.0f, 0.0f, -1.0f};
static Vec3 still_gyro  = {0.0f, 0.0f,  0.0f};
static Vec3 jitter_accel = {0.05f, 0.0f, -1.0f}; // ~0.05g lateral

void test_capture_completes_after_warmup_and_averaging(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    // 500ms warmup at 100 Hz = 50 ticks; 1s averaging = 100 ticks.
    for (int i = 0; i < 150; ++i) {
        fsm.update(still_accel, still_gyro);
    }
    TEST_ASSERT_TRUE(fsm.done());
    Vec3 result = fsm.result();
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, result.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, result.z);
}

void test_jitter_during_warmup_restarts(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 30; ++i) fsm.update(still_accel, still_gyro);   // partial warmup
    fsm.update(jitter_accel, still_gyro);                                // jitter -> restart
    for (int i = 0; i < 30; ++i) fsm.update(still_accel, still_gyro);   // partial again
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_jitter_during_averaging_restarts(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 50; ++i) fsm.update(still_accel, still_gyro);   // warmup done
    for (int i = 0; i < 30; ++i) fsm.update(still_accel, still_gyro);   // partial avg
    fsm.update(jitter_accel, still_gyro);                                // jitter -> restart
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}

void test_reset_clears_state(void) {
    zero_cal::CaptureFSM fsm;
    fsm.start();
    for (int i = 0; i < 150; ++i) fsm.update(still_accel, still_gyro);
    TEST_ASSERT_TRUE(fsm.done());
    fsm.start();
    TEST_ASSERT_FALSE(fsm.done());
    TEST_ASSERT_EQUAL(zero_cal::Phase::WARMUP, fsm.phase());
}
```

Add to `main`:

```cpp
RUN_TEST(test_capture_completes_after_warmup_and_averaging);
RUN_TEST(test_jitter_during_warmup_restarts);
RUN_TEST(test_jitter_during_averaging_restarts);
RUN_TEST(test_reset_clears_state);
```

- [ ] **Step 3.2: Run — should fail (CaptureFSM undefined)**

Run: `pio test -e native -f test_zero_cal`
Expected: FAIL with "no member `CaptureFSM`" or similar.

- [ ] **Step 3.3: Extend `src/zero_cal.h`**

Append to `src/zero_cal.h` (inside the `namespace zero_cal` block, before the closing brace):

```cpp
constexpr int WARMUP_TICKS    = 50;   // 500 ms at 100 Hz
constexpr int AVERAGING_TICKS = 100;  // 1000 ms at 100 Hz

enum class Phase : uint8_t { IDLE, WARMUP, AVERAGING, DONE };

class CaptureFSM {
public:
    void  start();
    void  update(Vec3 accel_g, Vec3 gyro_dps);
    bool  done()      const { return phase_ == Phase::DONE; }
    Phase phase()     const { return phase_; }
    int   warmup_remaining()    const;
    int   averaging_remaining() const;
    Vec3  result()    const;

private:
    void reset_to_warmup();

    Phase phase_           = Phase::IDLE;
    int   ticks_in_phase_  = 0;
    Vec3  accum_           = {0.0f, 0.0f, 0.0f};
    int   accum_count_     = 0;
    Vec3  result_          = {0.0f, 0.0f, 0.0f};
};
```

- [ ] **Step 3.4: Implement `CaptureFSM` in `src/zero_cal.cpp`**

Append to `src/zero_cal.cpp` (inside the namespace, after `is_still_instant`):

```cpp
void CaptureFSM::start() {
    reset_to_warmup();
}

void CaptureFSM::reset_to_warmup() {
    phase_          = Phase::WARMUP;
    ticks_in_phase_ = 0;
    accum_          = {0.0f, 0.0f, 0.0f};
    accum_count_    = 0;
    result_         = {0.0f, 0.0f, 0.0f};
}

void CaptureFSM::update(Vec3 accel_g, Vec3 gyro_dps) {
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) return;

    if (!is_still_instant(accel_g, gyro_dps)) {
        reset_to_warmup();
        return;
    }

    ++ticks_in_phase_;

    if (phase_ == Phase::WARMUP) {
        if (ticks_in_phase_ >= WARMUP_TICKS) {
            phase_          = Phase::AVERAGING;
            ticks_in_phase_ = 0;
            accum_          = {0.0f, 0.0f, 0.0f};
            accum_count_    = 0;
        }
        return;
    }

    // AVERAGING
    accum_.x += accel_g.x;
    accum_.y += accel_g.y;
    accum_.z += accel_g.z;
    ++accum_count_;

    if (ticks_in_phase_ >= AVERAGING_TICKS) {
        float n = (float)accum_count_;
        result_ = { accum_.x / n, accum_.y / n, accum_.z / n };
        phase_  = Phase::DONE;
    }
}

int CaptureFSM::warmup_remaining() const {
    if (phase_ != Phase::WARMUP) return 0;
    return WARMUP_TICKS - ticks_in_phase_;
}

int CaptureFSM::averaging_remaining() const {
    if (phase_ != Phase::AVERAGING) return 0;
    return AVERAGING_TICKS - ticks_in_phase_;
}

Vec3 CaptureFSM::result() const {
    return result_;
}
```

- [ ] **Step 3.5: Run — should pass**

Run: `pio test -e native -f test_zero_cal`
Expected: 7 PASS.

- [ ] **Step 3.6: Commit**

```bash
git add src/zero_cal.h src/zero_cal.cpp test/test_zero_cal/test_zero_cal.cpp
git commit -m "feat(zero_cal): capture FSM with warmup + averaging + jitter restart"
```

---

## Task 4: Update angle classifier signature

**Files:**
- Modify: `src/angle.h`
- Modify: `src/angle.cpp`
- Modify: `test/test_angle/test_angle.cpp`

The classifier currently takes `(AngleResult, tolerance_deg)`. The new signature decouples target from the magnitude that came out of `compute_angle`, since target is now scalar and the magnitude is `angle(g_now, g_zero_active)` rather than `angle(target_pose, g_now)`.

- [ ] **Step 4.1: Update `test_angle/test_angle.cpp` failing tests**

Open `test/test_angle/test_angle.cpp`. Replace any existing `classify(...)` calls with the new 4-argument signature, and append these new tests before `main`:

```cpp
void test_classify_in_tolerance_returns_green(void) {
    // magnitude 17.5°, target 17°, tol 1° -> within (16..18)
    ColorState c = classify(17.5f, 17.0f, 1.0f, +1);
    TEST_ASSERT_EQUAL(ColorState::GREEN, c);
}

void test_classify_above_tolerance_returns_red(void) {
    // magnitude 19°, target 17°, tol 1° -> above
    ColorState c = classify(19.0f, 17.0f, 1.0f, +1);
    TEST_ASSERT_EQUAL(ColorState::RED, c);
}

void test_classify_below_tolerance_returns_blue(void) {
    // magnitude 15°, target 17°, tol 1° -> below
    ColorState c = classify(15.0f, 17.0f, 1.0f, -1);
    TEST_ASSERT_EQUAL(ColorState::BLUE, c);
}

void test_classify_zero_direction_falls_back_to_green(void) {
    // magnitude 19°, dir=0 -> conservative GREEN per spec
    ColorState c = classify(19.0f, 17.0f, 1.0f, 0);
    TEST_ASSERT_EQUAL(ColorState::GREEN, c);
}
```

Add to `main`:

```cpp
RUN_TEST(test_classify_in_tolerance_returns_green);
RUN_TEST(test_classify_above_tolerance_returns_red);
RUN_TEST(test_classify_below_tolerance_returns_blue);
RUN_TEST(test_classify_zero_direction_falls_back_to_green);
```

Also: search for any existing `classify(r, tolerance_deg)` calls in this test file and rewrite them to the new signature using `r.degrees`, the test's target_deg, the tolerance, and `r.direction_sign`.

- [ ] **Step 4.2: Run — should fail (signature mismatch)**

Run: `pio test -e native -f test_angle`
Expected: build error — argument count mismatch.

- [ ] **Step 4.3: Update `src/angle.h`**

Replace the existing `classify` declaration:

```cpp
// Decide GREEN/BLUE/RED given the angle magnitude vs target ± tolerance.
// direction_sign disambiguates BLUE vs RED when out of tolerance.
// direction_sign == 0 => conservative GREEN fallback.
ColorState classify(float magnitude_deg,
                    float target_deg,
                    float tolerance_deg,
                    int   direction_sign);
```

- [ ] **Step 4.4: Update `src/angle.cpp`**

Replace the existing `classify` body:

```cpp
ColorState classify(float magnitude_deg,
                    float target_deg,
                    float tolerance_deg,
                    int   direction_sign) {
    float low  = target_deg - tolerance_deg;
    float high = target_deg + tolerance_deg;
    if (magnitude_deg >= low && magnitude_deg <= high) return ColorState::GREEN;
    if (direction_sign == 0)                            return ColorState::GREEN;
    if (direction_sign > 0)                             return ColorState::RED;
    return ColorState::BLUE;
}
```

- [ ] **Step 4.5: Run — should pass**

Run: `pio test -e native -f test_angle`
Expected: all PASS.

- [ ] **Step 4.6: Commit**

```bash
git add src/angle.h src/angle.cpp test/test_angle/test_angle.cpp
git commit -m "refactor(angle): classify takes magnitude/target/tolerance/sign"
```

---

## Task 5: Update SessionState

**Files:**
- Modify: `src/session.h`
- Modify: `src/session.cpp`

- [ ] **Step 5.1: Update `src/session.h`**

Replace the `SessionState` struct:

```cpp
struct SessionState {
    bool      active             = false;
    float     target_deg         = 0.0f;
    Tolerance tolerance          = Tolerance::NORMAL;
    Vec3      g_zero_A           = {0.0f, 0.0f, 0.0f};
    Vec3      g_zero_B           = {0.0f, 0.0f, 0.0f};
    uint32_t  strokes_A          = 0;
    uint32_t  strokes_B          = 0;
    Side      current_side       = Side::A;
    uint32_t  session_started_ms = 0;
};
```

- [ ] **Step 5.2: Search for `g_ref` references in session.cpp**

Run: `grep -n g_ref "./src/session.cpp"`

If there are any (e.g., field initializers in a static struct), update them to set `g_zero_A` and `g_zero_B` to `{0,0,0}` instead. Otherwise no change to session.cpp is needed.

- [ ] **Step 5.3: Build native to verify struct compiles**

Run: `pio run -e native`
Expected: build fails because `app.cpp` still references `s.g_ref`. That's expected — Task 6 fixes it. Do not commit yet.

---

## Task 6: Update App fields and `begin()`

**Files:**
- Modify: `src/app.h`
- Modify: `src/app.cpp`

- [ ] **Step 6.1: Update `src/app.h` private fields**

Open `src/app.h`. Replace the line `Vec3 g_ref_ = {0.0f, 0.0f, -1.0f};` with:

```cpp
    Vec3             g_zero_A_             = {0.0f, 0.0f, 0.0f};
    Vec3             g_zero_B_             = {0.0f, 0.0f, 0.0f};

    // ZERO_CAL substate machinery
    ZeroCalSubstate  zc_substate_          = ZeroCalSubstate::PROMPT_A;
    bool             zc_capture_running_   = false;
```

Add a new private method declaration alongside the other handlers:

```cpp
    void handle_zero_cal        (const Tick& t);
```

Add a `#include "zero_cal.h"` near the top with the other includes. Add a private member:

```cpp
    zero_cal::CaptureFSM zc_fsm_;
```

- [ ] **Step 6.2: Update `App::begin()` in `src/app.cpp`**

Find the `begin()` body (around lines 30–60 — search for `target_deg_         = s.target_deg;`) and replace `g_ref_ = s.g_ref;` (or its equivalent) with:

```cpp
            g_zero_A_           = s.g_zero_A;
            g_zero_B_           = s.g_zero_B;
```

Find the `mark_active` write-back (around line 70 — search for `ss.g_ref      = g_ref_;`) and replace with:

```cpp
            ss.g_zero_A   = g_zero_A_;
            ss.g_zero_B   = g_zero_B_;
```

- [ ] **Step 6.3: Defensive resume routing**

In `App::begin()`, after restoring fields, find where the resume-prompt path is selected. If both `g_zero_A_` and `g_zero_B_` are non-zero (any component non-zero) the resume into `RESUME_PROMPT` is fine. If either is the zero vector, force-route the resume into `ZERO_CAL` instead. Add this guard right before the existing resume transition. Use the helper:

```cpp
static inline bool is_zero_vec(Vec3 v) {
    return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
}
```

Place `is_zero_vec` at the top of `app.cpp` (after includes, before the class methods). Then in `begin`:

```cpp
    if (had_session_in_rtc_ram && session::has_session()) {
        // ... existing field restoration ...
        if (is_zero_vec(g_zero_A_) || is_zero_vec(g_zero_B_)) {
            transition(State::ZERO_CAL, 0);
            zc_substate_ = is_zero_vec(g_zero_A_) ? ZeroCalSubstate::PROMPT_A
                                                  : ZeroCalSubstate::PROMPT_B;
            return;
        }
        transition(State::RESUME_PROMPT, 0);
        return;
    }
```

(Adapt the surrounding code to fit the existing structure — the key additions are the `is_zero_vec` guard and the substate selection.)

- [ ] **Step 6.4: Build native — expect compile errors elsewhere**

Run: `pio run -e native`
Expected: build still fails because `handle_set_target` and `handle_active` still reference `g_ref_`. Task 7 and Task 8 fix those. No commit yet.

---

## Task 7: Strip g_ref synthesis from SET_TARGET

**Files:**
- Modify: `src/app.cpp`

The current `handle_set_target` synthesizes `g_ref_` two ways: from a preset using `{cos(r), 0, -sin(r)}`, and from freehand using `filter_.gravity()`. Both go away.

- [ ] **Step 7.1: Replace the body of `handle_set_target`**

Find `handle_set_target` (around line 110). Inside it, **delete** the synthesized preset block:

```cpp
                target_deg_ = preset_degrees(preset_selection_);
                // Synthesize a g_ref consistent with the chosen angle.
                float r = target_deg_ * (float)M_PI / 180.0f;
                g_ref_ = {std::cos(r), 0.0f, -std::sin(r)};
```

Replace it with:

```cpp
                target_deg_ = preset_degrees(preset_selection_);
```

Then **delete** the freehand block:

```cpp
            g_ref_ = filter_.gravity();
            float sinv = -g_ref_.z;
            // ... clamp and asin ...
            target_deg_ = std::asin(sinv) * (180.0f / (float)M_PI);
```

The freehand path is removed entirely. If `A_SHORT` was the trigger for freehand capture, change that branch to just transition forward to `SET_TOLERANCE` without capturing anything (target_deg keeps whatever value it had — default 17°).

After both deletions, search the rest of the function for any remaining reference to `g_ref_` and remove. The function should now only manipulate `target_deg_`, `preset_selection_`, and `in_preset_mode_`.

- [ ] **Step 7.2: Update the SET_TOLERANCE → next-state transition target**

Find the `transition(State::ACTIVE, ...)` call inside `handle_set_tolerance`. Change `State::ACTIVE` to `State::ZERO_CAL`. Also reset `zc_substate_ = ZeroCalSubstate::PROMPT_A` and `zc_fsm_` (call `zc_fsm_.start()` if appropriate — but per spec, capture only starts on the user pressing A, so leave the FSM idle and have `handle_zero_cal` start it on the press).

```cpp
    if (input == InputEvent::A_SHORT) {
        // ... save tolerance to NVS ...
        zc_substate_        = ZeroCalSubstate::PROMPT_A;
        zc_capture_running_ = false;
        transition(State::ZERO_CAL, t.now_ms);
    }
```

- [ ] **Step 7.3: Build native**

Run: `pio run -e native`
Expected: builds pass for all files except `handle_active` (and possibly `app.cpp`'s side FSM call site). Task 8 finishes those.

---

## Task 8: Wire ZERO_CAL handler

**Files:**
- Modify: `src/app.cpp`
- Modify: `test/test_app/test_app.cpp`

- [ ] **Step 8.1: Implement `handle_zero_cal`**

Add a new function in `src/app.cpp` (alongside the other `handle_*` functions):

```cpp
void App::handle_zero_cal(const Tick& t) {
    InputEvent input = t.input;

    // Long-press A aborts back to SET_TARGET (consistent with other screens).
    if (input == InputEvent::A_LONG) {
        zc_capture_running_ = false;
        transition(State::SET_TARGET, t.now_ms);
        return;
    }

    auto on_capture_done = [&](Vec3& dest, ZeroCalSubstate next) {
        dest                = zc_fsm_.result();
        zc_capture_running_ = false;
        zc_substate_        = next;
    };

    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_A:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_capture_running_ = true;
                zc_substate_        = ZeroCalSubstate::CAPTURE_A;
            }
            break;

        case ZeroCalSubstate::CAPTURE_A:
            zc_fsm_.update(t.accel_g, t.gyro_dps);
            if (zc_fsm_.done()) {
                on_capture_done(g_zero_A_, ZeroCalSubstate::PROMPT_B);
            }
            break;

        case ZeroCalSubstate::PROMPT_B:
            if (input == InputEvent::A_SHORT) {
                zc_fsm_.start();
                zc_capture_running_ = true;
                zc_substate_        = ZeroCalSubstate::CAPTURE_B;
            }
            break;

        case ZeroCalSubstate::CAPTURE_B:
            zc_fsm_.update(t.accel_g, t.gyro_dps);
            if (zc_fsm_.done()) {
                on_capture_done(g_zero_B_, ZeroCalSubstate::DONE);
                // Persist immediately and enter ACTIVE.
                SessionState ss;
                ss.active             = true;
                ss.target_deg         = target_deg_;
                ss.tolerance          = tol_;
                ss.g_zero_A           = g_zero_A_;
                ss.g_zero_B           = g_zero_B_;
                ss.strokes_A          = 0;
                ss.strokes_B          = 0;
                ss.current_side       = Side::A;
                ss.session_started_ms = t.now_ms;
                session::mark_active(ss);
                transition(State::ACTIVE, t.now_ms);
            }
            break;

        case ZeroCalSubstate::DONE:
            // Should not be reached — DONE triggers the transition above.
            break;
    }
}
```

- [ ] **Step 8.2: Add `ZERO_CAL` to the dispatch switch**

Find the main switch in `App::on_tick` (around line 250 — `case State::SET_TARGET: handle_set_target(t); break;`). Add:

```cpp
        case State::ZERO_CAL:      handle_zero_cal(t); break;
```

- [ ] **Step 8.3: Add a test for the ZERO_CAL flow**

Open `test/test_app/test_app.cpp`. Add a test that:
1. Drives the app from SET_TOLERANCE → A_SHORT → ZERO_CAL.
2. Sends A_SHORT → CAPTURE_A.
3. Feeds 150 still-sample ticks → expect transition to PROMPT_B with `g_zero_A_` populated.
4. Sends A_SHORT → CAPTURE_B.
5. Feeds 150 still-sample ticks → expect transition to ACTIVE with `g_zero_B_` populated.

```cpp
void test_zero_cal_two_capture_flow_advances_to_active(void) {
    App app;
    app.begin(/*had_session_in_rtc_ram=*/false);

    auto tick_with = [](uint32_t ms, InputEvent ev, Vec3 a, Vec3 g) {
        return App::Tick{ms, ev, a, g, FaultCode::NONE};
    };

    Vec3 still_a = {0.0f, 0.0f, -1.0f};
    Vec3 still_g = {0.0f, 0.0f,  0.0f};

    // Simulate getting through BIAS_CAL/SET_TARGET/SET_TOLERANCE — implementation
    // detail. Test helpers in this file should already cover that pre-amble; if
    // not, drive ticks until app.current() == State::ZERO_CAL. (Reuse any
    // existing helper named e.g. drive_to_state(app, State::ZERO_CAL).)
    drive_to_state(app, State::ZERO_CAL);

    // PROMPT_A: press A
    app.on_tick(tick_with(1000, InputEvent::A_SHORT, still_a, still_g));

    // 150 still ticks => CAPTURE_A done, advance to PROMPT_B
    for (int i = 0; i < 150; ++i) {
        app.on_tick(tick_with(1000 + (i+1)*10, InputEvent::NONE, still_a, still_g));
    }
    TEST_ASSERT_EQUAL(State::ZERO_CAL, app.current());
    // (Add an accessor in app.h for zc_substate_ if you want to assert it; otherwise
    // skip and rely on the next-step transition as the proof.)

    // PROMPT_B: press A
    app.on_tick(tick_with(3000, InputEvent::A_SHORT, still_a, still_g));

    // 150 still ticks => CAPTURE_B done, advance to ACTIVE
    for (int i = 0; i < 150; ++i) {
        app.on_tick(tick_with(3000 + (i+1)*10, InputEvent::NONE, still_a, still_g));
    }
    TEST_ASSERT_EQUAL(State::ACTIVE, app.current());
}
```

If `drive_to_state` doesn't exist, write inline ticks to walk the FSM from `BOOT` to `ZERO_CAL` (BIAS_CAL is skipped on non-first boot — the test starts with `had_session_in_rtc_ram=false` and a fresh NVS, so check the existing test patterns for the correct preamble). If existing tests already have a helper, reuse it.

Add to `main` in this test file:

```cpp
RUN_TEST(test_zero_cal_two_capture_flow_advances_to_active);
```

- [ ] **Step 8.4: Add a test-only accessor for `zc_substate_`**

In `src/app.h`, in the `public:` test-only accessors block (alongside `target_deg()`, `tolerance()`, etc.):

```cpp
    ZeroCalSubstate  zero_cal_substate() const { return zc_substate_; }
    Vec3             g_zero_a()          const { return g_zero_A_; }
    Vec3             g_zero_b()          const { return g_zero_B_; }
```

- [ ] **Step 8.5: Run native tests**

Run: `pio test -e native`
Expected: all PASS, including the new ZERO_CAL flow test.

- [ ] **Step 8.6: Commit**

```bash
git add src/app.h src/app.cpp src/session.h src/zero_cal.h src/zero_cal.cpp \
        test/test_app/test_app.cpp test/test_zero_cal/test_zero_cal.cpp
git commit -m "feat(app): wire ZERO_CAL state with two-capture flow"
```

---

## Task 9: Update `handle_active` to use `g_zero_active` and side-FSM polarity

**Files:**
- Modify: `src/app.cpp`

- [ ] **Step 9.1: Replace the angle compute call in `handle_active`**

Find `handle_active` (around line 153). Locate the line `AngleResult ar = compute_angle(g_ref_, g_now);`. Replace it with:

```cpp
    Vec3 g_zero_active = (side_fsm_.current_side() == Side::A) ? g_zero_A_ : g_zero_B_;
    AngleResult ar = compute_angle(g_zero_active, g_now);
```

- [ ] **Step 9.2: Update the classify call**

Find any `classify(ar, tolerance_deg)` call below it. Replace with:

```cpp
    ColorState color = classify(ar.degrees,
                                target_deg_,
                                tolerance_degrees(tol_),
                                ar.direction_sign);
```

- [ ] **Step 9.3: Replace `grav_dot_ref` source**

Find the line `float grav_dot_ref = g_now.x*g_ref_.x + g_now.y*g_ref_.y + g_now.z*g_ref_.z;` (around line 170). Replace with:

```cpp
    float grav_dot_ref = g_now.x*g_zero_A_.x + g_now.y*g_zero_A_.y + g_now.z*g_zero_A_.z;
```

(Always uses `g_zero_A_` — see spec §5.1: it's the polarity anchor for the side FSM, not the magnitude reference.)

- [ ] **Step 9.4: Search for any other lingering `g_ref_` references**

Run: `grep -n "g_ref_" "./src/app.cpp"`
Expected: no matches. If any remain, decide context-by-context (likely safe to delete or replace with the corresponding `g_zero_*`).

- [ ] **Step 9.5: Run native tests**

Run: `pio test -e native`
Expected: all PASS.

- [ ] **Step 9.6: Build target firmware**

Run: `pio run -e m5stick-c-plus`
Expected: build succeeds. Address any compile errors in UI / hardware modules from referenced symbols (next task covers UI; if anything else surfaces, fix inline).

- [ ] **Step 9.7: Commit**

```bash
git add src/app.cpp src/app.h
git commit -m "feat(app): handle_active uses g_zero_active for angle, g_zero_A_ for side polarity"
```

---

## Task 10: ZERO_CAL UI screens

**Files:**
- Modify: `src/ui.h`
- Modify: `src/ui.cpp`
- Modify: `src/app.cpp` (call sites)

- [ ] **Step 10.1: Add new UI declarations to `src/ui.h`**

Add to `src/ui.h`:

```cpp
namespace ui {
    // step: 1 or 2 (which side); retry: true if last attempt failed stillness gate
    void draw_zero_cal_prompt(int step, bool retry);
    // remaining_ms: ticks remaining in current capture window (warmup or averaging combined)
    void draw_zero_cal_progress(int remaining_ms);
}
```

- [ ] **Step 10.2: Implement renderers in `src/ui.cpp`**

Add to `src/ui.cpp`:

```cpp
#ifndef UNIT_TEST

void ui::draw_zero_cal_prompt(int step, bool retry) {
    auto& d = M5.Display;
    d.fillScreen(TFT_BLACK);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setTextSize(2);
    d.setCursor(8, 8);
    d.printf("ZERO CAL  %d/2", step);

    d.setTextSize(2);
    d.setCursor(8, 50);
    if (step == 1) d.print("Lay knife flat");
    else           d.print("Flip; lay flat");

    d.setCursor(8, 90);
    d.print("Press A");

    d.setCursor(8, 130);
    d.print("Hold still");

    if (retry) {
        d.setTextColor(TFT_RED, TFT_BLACK);
        d.setCursor(8, 180);
        d.setTextSize(3);
        d.print("HOLD STILL");
    }
}

void ui::draw_zero_cal_progress(int remaining_ms) {
    auto& d = M5.Display;
    d.fillScreen(TFT_BLACK);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setTextSize(3);
    d.setCursor(8, 60);
    d.print("Hold still");
    d.setTextSize(4);
    d.setCursor(8, 120);
    d.printf("%0.1fs", remaining_ms / 1000.0f);
}

#endif
```

(The `#ifndef UNIT_TEST` matches the existing pattern in `ui.cpp` where Arduino-coupled code is excluded from the native build.)

- [ ] **Step 10.3: Call the new UI from `app.cpp`**

In `App::handle_zero_cal`, after the substate logic, call the right renderer. The simplest form: render at the end of every tick. Add at the end of the function:

```cpp
#ifndef UNIT_TEST
    int total_capture_ms_remaining =
        zc_fsm_.warmup_remaining() * 10 +
        zc_fsm_.averaging_remaining() * 10;

    // "retry" cue: detect a recent gate failure by the FSM dropping back to WARMUP
    // mid-CAPTURE phase. We can detect this from outside via phase_ + ticks_in_phase.
    // For v1 simplicity, show retry only if WARMUP and we've been in capture for >0 ticks.
    bool retry = false;  // v1: don't show retry cue. Add later if needed.

    switch (zc_substate_) {
        case ZeroCalSubstate::PROMPT_A:  ui::draw_zero_cal_prompt(1, retry); break;
        case ZeroCalSubstate::CAPTURE_A: ui::draw_zero_cal_progress(total_capture_ms_remaining); break;
        case ZeroCalSubstate::PROMPT_B:  ui::draw_zero_cal_prompt(2, retry); break;
        case ZeroCalSubstate::CAPTURE_B: ui::draw_zero_cal_progress(total_capture_ms_remaining); break;
        case ZeroCalSubstate::DONE:      break;
    }
#endif
```

- [ ] **Step 10.4: Build target firmware**

Run: `pio run -e m5stick-c-plus`
Expected: succeeds.

- [ ] **Step 10.5: Run native tests (sanity)**

Run: `pio test -e native`
Expected: all PASS (UI code is excluded from native via `#ifndef UNIT_TEST`).

- [ ] **Step 10.6: Commit**

```bash
git add src/ui.h src/ui.cpp src/app.cpp
git commit -m "feat(ui): ZERO_CAL prompt and progress screens"
```

---

## Task 11: Bring-up checklist additions

**Files:**
- Modify: `docs/superpowers/bringup/2026-04-23-hardware-bringup.md`

- [ ] **Step 11.1: Append a Zero-Cal validation section**

Open `docs/superpowers/bringup/2026-04-23-hardware-bringup.md`. Append at the end:

```markdown

## Zero-Calibration Validation (added 2026-04-28)

Run after the basic IMU/UI/buttons checks pass.

1. **Repeatability** — capture 5 zeros in a row on the same physical pose without moving the device. Compare each captured `g_zero` against the first using `acos(dot)`. Expected: angular spread between any two captures is `< 0.5°`. If wider, the stillness gate is too loose for the bench environment — tighten thresholds in `zero_cal.h`.
2. **Cross-side correctness** — mount the device on a real blade, lay flat on a stone for side A, capture; flip to side B, lay flat, capture. Tilt to roughly the target angle (use a protractor); confirm the displayed angle is within ±1° of the protractor reading on both sides. Repeat with a tilted stone (place a coin under one edge of the stone): displayed angle should still match the protractor.
3. **Stillness-gate retry** — start CAPTURE_A, then tap the device gently mid-window. The FSM should drop back to WARMUP and display the prompt anew. The capture must not complete with bad data.
4. **Resume after sleep** — complete a full ZERO_CAL, enter ACTIVE, force sleep (long-press A then idle timeout), wake. Expected: skip ZERO_CAL, resume ACTIVE with the previously-captured zeros intact.
5. **Battery-pull recovery** — power-cycle (battery pull) after a successful ZERO_CAL. Expected: RTC RAM clears, next boot routes through ZERO_CAL again.

If any step fails, capture serial logs (`pio device monitor -b 115200`) and update `Known risks` in `docs/superpowers/specs/2026-04-28-zero-calibration-design.md` with the observed failure mode before adjusting thresholds.
```

- [ ] **Step 11.2: Commit**

```bash
git add docs/superpowers/bringup/2026-04-23-hardware-bringup.md
git commit -m "docs(bringup): zero-cal validation checklist"
```

---

## Task 12: Final integration sweep

**Files:**
- (read-only) all modified files

- [ ] **Step 12.1: Search for stale `g_ref` references**

Run:
```
grep -rn "g_ref" ~/Documents/AI/Sharpner\ Guide/src \
                 ~/Documents/AI/Sharpner\ Guide/test
```

Expected: zero matches in `src/`. Test files may reference it in old commented-out tests — clean any of those up. Commit if anything was found and removed:

```bash
git add -A
git commit -m "chore: remove stray g_ref references"
```

- [ ] **Step 12.2: Full native test run**

Run: `pio test -e native`
Expected: all suites PASS — `test_angle`, `test_zero_cal`, `test_app`, `test_stroke`, `test_side`, `test_filter`, `test_input`.

- [ ] **Step 12.3: Full target build**

Run: `pio run -e m5stick-c-plus`
Expected: build succeeds with no warnings introduced by this work. Compare flash/RAM usage against the previous main commit (just informational — no hard limit).

- [ ] **Step 12.4: CodeRabbit review**

Run: `coderabbit review --plain`
Address any issues flagged in zero-cal-related files. Loop until clean.

- [ ] **Step 12.5: Final commit if anything changed**

```bash
git add -A
git commit -m "chore: final review fixes for zero-cal"
```

---

## Verification checklist (run before declaring done)

- [ ] `grep -rn "g_ref" src/ test/` returns zero matches
- [ ] `pio test -e native` — all PASS
- [ ] `pio run -e m5stick-c-plus` — builds clean
- [ ] `coderabbit review --plain` — clean
- [ ] `git log --oneline` — atomic commits, conventional messages, all co-authored
- [ ] Spec §11 risks reviewed; any in-tree TODOs left for hardware bring-up validation are flagged in the bring-up doc, not in code comments
