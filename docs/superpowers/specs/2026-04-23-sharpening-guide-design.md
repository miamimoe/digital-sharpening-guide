# Sharpening Guide — Design Spec

**Date:** 2026-04-23
**Status:** Draft for approval
**Target hardware:** M5StickC Plus Watch Kit (ordered, not yet in hand)

## 1. Summary

An electronic sharpening angle guide. A small ESP32-based device magnetically attaches to a knife blade's flat side. The user sets a target sharpening angle (either by capturing the current blade angle against the stone or selecting from common presets), then sharpens on a whetstone as normal. The device fills its color LCD with a **solid green** background while the user holds the target angle within tolerance, **solid blue** if the angle drops below tolerance (signalling "raise the spine"), and **solid red** if the angle rises above tolerance (signalling "lower the spine"). It also auto-counts strokes per side of the blade — central to balanced sharpening — detecting side switches automatically when the user peels the device off one flat and reattaches it to the other.

**Reference product:** SharpAl HoldBubble (concept similar; we add stroke counting and directional tri-color feedback).

## 2. Hardware

**Kit:** M5StickC Plus Watch Kit (ordered, delivery pending).

| Component | Part | Role |
|---|---|---|
| MCU | ESP32-PICO | Main processor, ~240 MHz dual-core |
| IMU | MPU6886 | 6-axis accel + gyro, I²C |
| Display | ST7789V2 | 1.14" color LCD, 135×240, SPI |
| PMIC | AXP192 | Battery mgmt, LDO3 controls backlight |
| RTC | BM8563 | Real-time clock (unused in v1) |
| Mic | SPM1423 | PDM mic (unused in v1) |
| Battery | LiPo, ~120 mAh | ~1–2 hr active, much longer idle |
| Buttons | 2 user (A main, B side) + power | Primary UX |
| LED | 1 red | Peripheral-vision feedback |
| Buzzer | Piezo | Opt-in audible feedback |

**Mounting:** a neodymium magnet glued directly to the back of the device body. No kit mount in the loop (brick/watch strap/wall mount all unused for this project).

## 3. Product behavior

### 3.1 States

```
BOOT
 ↓ (splash 1s)
SET_TARGET ──────────────────────┐
 │ (A=capture or B=preset→A)     │
 ↓                               │
SET_TOLERANCE                    │ (wake path)
 │ (B cycles, A confirms)        │
 ↓                               │
ACTIVE ← RESUME_PROMPT ←─ SLEEP ─┘
 │ (long-press A)                ↑
 ↓                               │
SUMMARY                          │
 │ (A=new session → SET_TARGET)  │
 └───────────────────────────────┘

Any state + 2 min idle + no motion → SLEEP
BOOT + IMU init fail → FAULT (halt, requires power cycle)
```

### 3.2 Screens

All screens render on a 135×240 color LCD.

**BOOT** — 2-second splash: "SHARPENING GUIDE", version number. On the first boot after firmware flash (detected via an NVS flag), BOOT is extended into **BIAS_CAL**: display "Hold still — calibrating" with a 10-second countdown while capturing gyro bias. If the device moves during BIAS_CAL (accel magnitude deviation > threshold), the countdown restarts. On completion, bias is written to NVS and the first-boot flag is cleared. Subsequent boots skip straight from splash to SET_TARGET.

**SET_TARGET** — shows live angle (gravity-derived) as a large number. `A`: capture current angle and advance to SET_TOLERANCE. `B`: enter preset mode.
- Preset mode: `B` cycles **12° / 15° / 17° / 20° / 22° / CANCEL**. `A` on a numeric preset confirms and advances to SET_TOLERANCE. `A` on `CANCEL` returns to live-capture mode (the top-level SET_TARGET screen).

**SET_TOLERANCE** — displays current preset (default from NVS; on very-first boot, defaults to **NORMAL ±2°**). `B` cycles **TIGHT ±1° / NORMAL ±2° / EASY ±3°**. `A` confirms, persists to NVS, and advances to ACTIVE.

**ACTIVE** — full-screen solid color (blue / green / red) based on angle state. Overlay on top of color:
- Tiny legend strip at the top, drawn once on entry: three small colored squares (blue, green, red) with text labels "LOW · OK · HIGH" beside them. No emoji (M5GFX default font does not render color emoji).
- Current side's stroke count, large (center/lower)
- Other side's stroke count, small (corner), prefixed with side label (e.g., `B: 11`)
- No live angle number — the color is the primary signal.

Color rules:
- In tolerance → **green**.
- Angle < target − tolerance → **blue** ("raise spine / increase angle").
- Angle > target + tolerance → **red** ("lower spine / decrease angle").

Transitions / controls:
- Long-press `A` (≥800 ms) → SUMMARY.
- `B` short-press → manual side toggle (override).
- Long-press `B` (≥800 ms) → toggle buzzer on/off, persist to NVS. Brief on-screen confirmation ("BUZZER ON" / "BUZZER OFF", 800 ms) overlays the color background, then returns to normal render.
- Motion-spike + settle with inverted gravity sign → auto side switch (Section 4.4).
- Any `B` press suppresses auto-side-switch for 2 seconds (prevents double-toggle when the user manually overrides during a peel/settle window).

**SUMMARY** — final stats for the ended session:
- Target angle
- Tolerance setting
- Strokes Side A
- Strokes Side B
- Session duration (MM:SS)
- `A`: start new session → SET_TARGET.
- `B`: go to SLEEP without starting a new session (stats preserved in RTC RAM, so wake → RESUME_PROMPT will show them again).

**FAULT** — shown if IMU fails to initialize at boot. Displays:
- "IMU FAULT"
- Error code (e.g., `E01` for `begin()` returned false, `E02` for self-test failure, `E03` for WHO_AM_I mismatch — latter is the most likely indicator of the documented MPU6886/AXP192 I²C address conflict).
- Instruction: "Power-cycle to retry"
Red LED is solid-on to provide peripheral confirmation. Device does not silently proceed.

**RESUME_PROMPT** — shown on wake from sleep **only if the device entered sleep from ACTIVE** (or from SUMMARY with preserved stats, entered via `B` in SUMMARY). If sleep was entered from SET_TARGET or SET_TOLERANCE or BOOT, wake goes directly to SET_TARGET. 5-second countdown:
- `A` → resume ACTIVE with same target, tolerance, and stroke counts from RTC RAM.
- `B` or timeout → new session → SET_TARGET.

**SLEEP** — display off, LDO3 off, ESP32 deep sleep. Wake via AXP192 power-key.

### 3.3 Feedback channels (beyond the LCD color)

- **Red LED** — mirrors the HIGH state only (on when screen is red; off otherwise). Provides peripheral-vision cue when eyes are on the blade. Also: solid-on in FAULT state.
- **Buzzer** — persisted via NVS. Toggled at runtime by long-pressing `B` in ACTIVE (see 3.2). Short beep on transition into out-of-tolerance (either blue or red). No beep returning to green. Default off on first-boot.
- No haptic (no vibration motor in this kit).

### 3.4 Persistent settings (NVS, ESP32 Preferences)

| Key | Values | Default | Set by |
|---|---|---|---|
| `tolerance` | `TIGHT` / `NORMAL` / `EASY` | `NORMAL` | `A` confirm in SET_TOLERANCE |
| `buzzer` | `0` / `1` | `0` (off) | Long-press `B` in ACTIVE |
| `gyro_bias_x/y/z` | int16 triplet | zeros | Written on BIAS_CAL completion |
| `first_boot` | `0` / `1` | `1` (first-boot) | Cleared after BIAS_CAL completes |

All persist across power cycles.

### 3.5 Session state (RTC RAM, survives deep sleep, lost on battery pull)

| Field | Purpose |
|---|---|
| `session_active` | Is there a session to resume? Set on ACTIVE entry, cleared on SUMMARY dismiss via `A` (new session). |
| `target_angle`, `tolerance_preset` | Current session settings |
| `g_ref` (3 floats) | Captured reference gravity |
| `strokes_A`, `strokes_B` | Stroke counts per side |
| `current_side` | `A` or `B` |
| `session_started_ms` | For duration calculation |

`session_active == true` at wake → route to RESUME_PROMPT. `false` → route to SET_TARGET.

## 4. Technical design

### 4.1 Loop architecture

Single-threaded cooperative loop at **100 Hz** (10 ms tick). No FreeRTOS tasks. Per tick:

```
1. imu.sample()          // raw gyro + accel
2. filter.update()       // Mahony AHRS
3. angle.compute()       // degrees from g_ref
4. stroke.update()       // in/out hysteresis
5. side.update()         // peel/settle detection
6. app.tick()            // state machine transitions
7. if tick % 5 == 0:
     ui.render()         // dirty-region, 20 Hz
     feedback.render()   // LED, buzzer edges
8. power.check_idle()
```

Display renders at **20 Hz** (every 5th tick) to stay under SPI bandwidth.

### 4.2 Orientation filter

**Mahony AHRS** (public-domain, ~80 LOC). Fuses gyro and accelerometer to produce a stable gravity-direction estimate robust against translational acceleration during strokes (which a naive complementary filter cannot handle at ±1° precision).

Gains (`Kp`, `Ki`) are placeholders at defaults; **must be tuned on hardware against a reference inclinometer during actual sharpening motion** — this is a known risk that can only be resolved post-delivery.

### 4.3 Angle computation (orientation-agnostic)

At capture time (either via "capture" mode or preset confirm), snapshot the current gravity unit vector `g_ref` (3 components in device frame, from the Mahony filter output).

**Magnitude of deviation:**

```
θ = acos( dot(g_ref, g_now) ) * 180/π     // unsigned degrees from reference
```

Axis-label-free. Works regardless of how the user rotated the device when sticking it to the blade.

**Direction of deviation** (for blue vs red): exploits the fact that the magnet geometry fixes the device's back surface against the blade's flat. The device's local axis perpendicular to the back (`n_back`, a fixed unit vector in device frame) always points into the blade. On M5StickC Plus, the IMU is soldered such that the screen is on the opposite face from the back; by the M5Stack schematic, `n_back = (0, 0, -1)` in the MPU6886's body frame (i.e., the negative local Z-axis). The exact sign is validated by a simple bring-up test: place the device flat on a table, screen up, and confirm `g·n_back < 0` (gravity's Z-component is −1 g when Z points up); the code flips the `n_back` sign at a single compile-time constant if the empirical convention differs.

Using signed `g·n_back` (NOT absolute value), the relationship to sharpening angle θ_s is:

```
g · n_back = sin(θ_s)    (monotonic for θ_s ∈ (−90°, 90°), which covers all practical sharpening)
```

This follows because the blade's flat is perpendicular to `n_back`, and the sharpening angle is by definition the tilt between the flat and the stone (gravity-perpendicular).

Directional signal:

```
α_ref = dot(g_ref, n_back)
α_now = dot(g_now, n_back)
δ     = α_now − α_ref
```

- `δ > 0` → angle **increased** → **RED** (lower the spine)
- `δ < 0` → angle **decreased** → **BLUE** (raise the spine)
- Only evaluated when `θ > tolerance`; otherwise → **GREEN**

**Orientation-agnostic under device rotation:** rotating the device around `n_back` (i.e., spinning it in the plane of the blade flat when mounting) leaves `g · n_back` unchanged, so both magnitude `θ` and direction `δ` are invariant under that rotation. The user does not have to think about which way is "up" when sticking the device on.

### 4.4 Stroke and side detection

**Stroke definition:** one stroke = one complete "held at target angle" period, bookended by a departure. The user's act of lifting the blade from the stone, or deliberately drifting off angle between passes, produces the OUT window that ends a stroke. Passes that never enter tolerance (i.e., the user never held within ± tolerance for ≥ 300 ms) are **by design not counted** — the entire point of the counter is to count *good* strokes per side.

**Stroke counter FSM** (per-side):
- `OUT_TOL`: initial state. Track how long angle has been outside tolerance.
- Enter `IN_TOL` when angle has been within tolerance for ≥ **300 ms** continuously.
- Re-enter `OUT_TOL` when angle has been outside tolerance for ≥ **200 ms** continuously (hysteresis).
- **A stroke is counted on each `IN_TOL → OUT_TOL` transition.**
- Edge case: an unbroken `IN_TOL` period longer than 3 seconds still counts as exactly one stroke, counted on its eventual exit. Documented in code as the expected behavior — users won't hit this in practice since lifting the blade at the end of each pass naturally produces an OUT window.
- The 300 ms / 200 ms thresholds are initial estimates; tuning against real sharpening sessions post-delivery is a known risk.

**Side detection — motion-spike + gravity-sign:**
- **Spike event:** accelerometer magnitude deviates from 1 g by more than **0.5 g peak** during a 100 ms window. Peels of the magnet from the blade are sharp/fast events; this must not require a long sustained high-magnitude period.
- **Settle event:** after a spike, accel magnitude returns to within 0.1 g of 1 g and stays stable for ≥ **500 ms**.
- On settle, compute `sign(dot(g_now, g_ref))`:
  - **Negative** (flipped) → auto side switch. Counter for the other side becomes the active one (both counters persist; neither resets mid-session).
  - **Positive** (same side) → no change.
- **Timeout:** if no settle occurs within 5 seconds of a spike, reset detection (classify as ongoing handling; the user can still reach settle later and trigger switch).
- **B-press suppression:** after any `B` press in ACTIVE, auto-side-switch is suppressed for 2 seconds (prevents double-toggle with a manual override that coincides with handling motion).

**Manual override:** short-press `B` in ACTIVE swaps current side regardless of sensor state.

### 4.5 Display rendering

- **Library:** M5GFX (bundled transitively with `m5stack/M5Unified`, the current standard library for M5Stack boards).
- **Dirty-region only**: never do full-frame redraws during steady-state ACTIVE.
  - Background color (blue/green/red) → redrawn **only on state change** (blue↔green↔red).
  - Stroke count numbers → redrawn **only on increment or side switch**.
  - Side label → redrawn on side switch.
  - Legend strip → drawn once on entry to ACTIVE, never redrawn.
- Full frame (135×240 × 16 bpp ≈ 64 KB) at 27 MHz SPI ≈ 19 ms, which exceeds the 10 ms loop tick. Dirty-region keeps steady-state renders well under the 20 Hz budget. State-transition ticks that require a full-screen fill (e.g., blue→red) consume up to ~20 ms; those ticks are allowed to skip the next loop iteration's sensor sample without user-visible effect (10 Hz effective sample during that single transition; the Mahony filter tolerates a single sample dropout).

### 4.6 Power management

Idle/sleep criteria differ by state:

| State | Dim backlight at | Enter SLEEP at |
|---|---|---|
| BOOT / BIAS_CAL | never (short-lived) | never |
| SET_TARGET | 90 s no-motion | 120 s no-motion |
| SET_TOLERANCE | 60 s no-motion | 90 s no-motion |
| ACTIVE | 3 min no-new-strokes | **5 min no-new-strokes** (motion-agnostic — user may pause between passes to check edge, wet stone, etc. without losing the session) |
| SUMMARY | 60 s | 90 s |
| RESUME_PROMPT | n/a | on 5 s timeout |
| FAULT | never | never (user must power-cycle) |

Sleep sequence:
1. Call `M5.Axp.SetLDO3(false)` to cut backlight power (otherwise backlight stays lit during deep sleep and drains the battery — documented M5StickC Plus-specific behavior).
2. Save session state to RTC RAM if `state == ACTIVE` at sleep entry (see 3.5).
3. `esp_deep_sleep_start()` with wake source set to AXP192 power-key interrupt (not a raw GPIO) — documented M5StickC Plus-specific requirement.

On wake:
- Read RTC RAM `session_active` flag.
- `true` → RESUME_PROMPT (session still in RTC RAM).
- `false` → SET_TARGET.
- RTC RAM is lost on battery pull or on AXP192 power-off (long power-key hold); that is acceptable — session preservation is a convenience, not a promise.

### 4.7 Module breakdown

Logical modules (separate files in `src/`):

| Module | Responsibility |
|---|---|
| `imu` | MPU6886 read + 10-second still-gyro bias capture at first boot (NVS) |
| `filter` | Mahony AHRS; outputs quaternion + gravity unit vector |
| `angle` | Pure function: `(g_ref, g_now) → (degrees, direction_sign)` — uses compile-time `n_back` constant. No state. |
| `stroke` | In/out hysteresis FSM per side |
| `side` | Spike/settle detection + gravity-sign compare + manual override |
| `ui` | M5GFX render; pure "draw current state" — no logic |
| `input` | Debounce + short/long press detection on A and B |
| `feedback` | LED + buzzer "show current tolerance state" |
| `power` | Idle detection, backlight dim/sleep, wake plumbing |
| `settings` | NVS accessor for tolerance + buzzer |
| `app` | State machine, event wiring, mode transitions |

Gravity-sign tracking lives in `side`, not `angle`, to keep `angle` a pure function.

## 5. Build & firmware environment

- **Framework:** Arduino.
- **Build:** PlatformIO.
- **`platformio.ini` env:** `m5stick-c-plus`.
- **Key library:** `m5stack/M5Unified` (bundles M5GFX and the board HAL; M5Unified is the current standard and supersedes the older per-board `M5StickCPlus` package). Mahony AHRS implementation inlined (public-domain reference implementation, ~100 LOC including the fast-inverse-sqrt helper).
- **Minimum toolchain:** Arduino-ESP32 core 2.x, PlatformIO 6.x.

## 6. Testing

### 6.1 Desktop unit tests (no hardware required)

All FSM tests use a pseudo-sample stream: a text fixture file of `(t_ms, ax, ay, az, gx, gy, gz)` rows fed into the module under test, asserting expected event emissions.

- **`angle.compute()`**: feed canned `g_ref` + `g_now` pairs, assert θ degrees (within 0.01°) and direction sign. Include coverage for: zero-deviation (θ=0, δ=0), ±5° deviation both signs, edge cases near the tolerance boundaries.
- **Stroke FSM**: feed synthetic `(t_ms, in_tolerance_bool)` sequences, assert expected stroke counts. Include coverage for: canonical 3-stroke session, sub-300 ms in-tolerance blips (must not count), sub-200 ms out-of-tolerance wobbles (must not re-arm), the > 3 s unbroken in-tolerance edge case (counts as 1), back-to-back valid strokes.
- **Side FSM**: feed synthetic `(t_ms, accel_magnitude, gravity_sign)` sequences, assert expected side-switch emissions. Include coverage for: peel/flip/settle (switches), peel/no-flip/settle (doesn't switch), peel/no-settle-within-5s (reset), B-press suppression window (suppresses auto-switch for 2 s).

### 6.2 On-hardware validation (once kit arrives)

- **IMU-on-I²C sanity:** confirm MPU6886 `begin()` succeeds and returns non-zero samples — rules out the known potential AXP192 I²C address conflict depending on shipped firmware.
- **Power-key wake:** validate `esp_deep_sleep_start()` → power-key press → re-entry at the expected app state.
- **Mahony tuning:** bench-test with real sharpening motion against a reference inclinometer; tune `Kp` / `Ki`.
- **Stroke-gap thresholds:** record a real sharpening session, measure actual in/out timing distributions, tune 300 / 200 ms.
- **Backlight-on-sleep:** confirm LDO3 cut correctly dims the display on deep sleep.

### 6.3 Manual QA walkthrough

Once firmware is up, walk each state machine transition by hand with expected observable behavior documented step-by-step (written alongside the spec when hardware is in hand).

## 7. Scope — explicitly out of v1

- BLE / phone companion app
- Session logging / history beyond the current session
- OTA updates
- Battery state-of-charge display
- Per-stroke analytics (timing, avg deviation)
- Custom preset entry via button dial-in (use capture mode instead)
- Haptic feedback (no motor in this kit)
- Recalibration UI beyond first-boot gyro bias

All of the above are candidates for v2 once the v1 loop is validated with real sharpening sessions.

## 8. Known risks to resolve on hardware

1. **Mahony gain tuning** — empirical, no substitute for real motion on real hardware.
2. **Stroke hysteresis thresholds (300 / 200 ms)** — initial guesses; will need adjustment after measuring real sessions.
3. **MPU6886 / AXP192 I²C conflict** — documented community issue dependent on shipped M5 firmware; confirm on first boot.
4. **AXP192 deep-sleep wake behavior** — board-specific errata reported in community; validate the full sleep → wake round trip.
5. **Magnet on device back** — how well it grips a typical kitchen-knife flat; mounting method (adhesive choice) may need iteration.

## 9. Appendix — decisions locked during brainstorm

1. Target-angle input: capture + presets (**12° / 15° / 17° / 20° / 22° / CANCEL**); A on CANCEL returns to live capture
2. Axis detection: orientation-agnostic (`sin(θ) = dot(g, n_back)` — signed, monotonic)
3. Tolerance: 3 user-selectable presets at session start (±1° / ±2° / ±3°), persisted in NVS
4. Mounting: magnet glued directly to device back
5. Feedback: tri-color screen (blue/green/red) + red LED (on in HIGH and FAULT) + opt-in buzzer (toggle via long-press B in ACTIVE, persisted)
6. Stroke detection: auto, in/out hysteresis with 300 ms / 200 ms min durations. Sloppy passes that never enter tolerance don't count (by design).
7. Side switch: auto via peel-spike (>0.5 g peak in 100 ms) + settle + gravity-sign compare; manual override on B short-press; B-press suppression 2 s to prevent double-toggle
8. Session lifecycle: explicit, long-press A in ACTIVE ends; idle-sleep criterion in ACTIVE = 5 min with no new strokes (motion-agnostic). B in SUMMARY = sleep without starting new session
9. Wake behavior: RESUME_PROMPT only shown if sleep entered from ACTIVE (or SUMMARY with preserved stats); otherwise wake → SET_TARGET. 5 s resume-prompt timeout → new session
10. Active screen: solid-color full-background + large current-side count + small other-side count. No live angle number. No emoji — text + colored primitives only.
11. First-boot: BIAS_CAL state captures gyro bias (10 s still) before first use; persisted to NVS
12. Framework: Arduino + PlatformIO + `m5stack/M5Unified`
13. Persistence: tolerance, buzzer, gyro bias, first-boot flag in NVS; session state in RTC RAM
14. FAULT state on IMU init failure: displays error code (E01/E02/E03), red LED solid, requires power-cycle
15. Out of scope v1: BLE, logging/history, companion app, OTA, battery SoC display, per-stroke analytics, custom preset dial-in, recalibration UI beyond first-boot
