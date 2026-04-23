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

**BOOT** — 1-second splash: "SHARPENING GUIDE", version number.

**SET_TARGET** — shows live angle (gravity-derived) as a large number. `A`: capture current angle and advance to SET_TOLERANCE. `B`: enter preset mode.
- Preset mode: `B` cycles **12° / 15° / 17° / 20° / 22°**. `A` confirms selection and advances to SET_TOLERANCE.

**SET_TOLERANCE** — displays current preset (default from NVS). `B` cycles **TIGHT ±1° / NORMAL ±2° / EASY ±3°**. `A` confirms and advances to ACTIVE.

**ACTIVE** — full-screen solid color (blue / green / red) based on angle state. Overlay on top of color:
- Tiny legend at top: `🔵LOW  🟢OK  🔴HIGH`
- Current side's stroke count, large (center/lower)
- Other side's stroke count, small (corner), prefixed with side label (e.g., `B: 11`)
- No live angle number — the color is the primary signal.

Color rules:
- In tolerance → **green**.
- Angle < target − tolerance → **blue** ("raise spine / increase angle").
- Angle > target + tolerance → **red** ("lower spine / decrease angle").

Transitions:
- Long-press `A` (≥800 ms) → SUMMARY.
- `B` short-press → manual side toggle (override).
- Motion-spike + settle with inverted gravity sign → auto side switch (Section 4.4).

**SUMMARY** — final stats for the ended session:
- Target angle
- Tolerance setting
- Strokes Side A
- Strokes Side B
- Session duration (MM:SS)
- `A`: start new session → SET_TARGET.

**FAULT** — shown if IMU fails to initialize at boot. Static error message. User must power-cycle. Device does not silently proceed.

**RESUME_PROMPT** — shown on wake from sleep if a session was in progress (had a target angle captured). 5-second countdown:
- `A` → resume ACTIVE with same target, tolerance, and stroke counts.
- `B` or timeout → new session → SET_TARGET.

**SLEEP** — display off, LDO3 off, ESP32 deep sleep. Wake via AXP192 power-key.

### 3.3 Feedback channels (beyond the LCD color)

- **Red LED** — mirrors the HIGH state only (on when screen is red; off otherwise). Provides peripheral-vision cue when eyes are on the blade.
- **Buzzer** — opt-in via NVS setting. Short beep on transition into out-of-tolerance (either blue or red). No beep returning to green. Default off.
- No haptic (no vibration motor in this kit).

### 3.4 Persistent settings (NVS, ESP32 Preferences)

| Key | Values | Default |
|---|---|---|
| `tolerance` | `TIGHT` / `NORMAL` / `EASY` | `NORMAL` |
| `buzzer` | `0` / `1` | `0` (off) |

Both persist across power cycles. Set via their respective cycling UI; saved on confirm.

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

At capture time (either via "capture" mode or preset confirm), snapshot the current gravity unit vector `g_ref` (3 components).

Live sharpening angle:

```
θ = acos( dot(g_ref, g_now) )   // radians, convert to degrees
```

Completely axis-label-free. Works regardless of how the user oriented the device when sticking it to the blade.

**Direction of deviation** (for blue vs red): at capture, also snapshot which local IMU axis is most aligned with the expected "tilt" direction (the axis most orthogonal to gravity at the moment of capture, projected into the device frame). The sign of the current gravity vector's projection onto that axis, compared to capture, tells us whether the angle increased (red) or decreased (blue).

### 4.4 Stroke and side detection

**Stroke counter** — finite state machine per-side:
- `IN_TOL`: current angle within target ± tolerance for ≥ **300 ms**.
- `OUT_TOL`: outside tolerance for ≥ **200 ms**.
- A stroke is counted on the `IN_TOL → OUT_TOL` transition.
- Edge case: an unbroken `IN_TOL` period > 3 seconds still counts as exactly one stroke (counted on its eventual exit). Documented in code.
- The 300 ms / 200 ms thresholds are initial estimates; tuning against real sharpening sessions post-delivery is a known risk.

**Side detection** — motion-spike + gravity-sign:
- **Spike event:** accelerometer magnitude deviates from 1 g by more than a threshold (e.g., > 0.5 g for ≥ 500 ms). This indicates the device is being handled (peeled off, moved).
- **Settle event:** after a spike, gravity magnitude returns to ~1 g and stays stable for ≥ 500 ms.
- On settle, compute `sign(dot(g_now, g_ref))`:
  - **Negative** (flipped) → auto side switch. Other side's counter becomes current.
  - **Positive** (same side) → no change (user picked up and set back down).
- **Timeout:** if no settle occurs within 2 seconds of a spike, ignore the spike (classify as ongoing handling).
- Current side's stroke counter is not reset on switch; we persist both counters for the session.

**Manual override:** short-press `B` in ACTIVE swaps current side regardless of sensor state.

### 4.5 Display rendering

- **Library:** M5GFX via the official `M5StickCPlus` Arduino package.
- **Dirty-region only**: never do full-frame redraws.
  - Background color (blue/green/red) → redrawn **only on state change**.
  - Stroke count numbers → redrawn **only on increment**.
  - Side label → redrawn on side switch.
  - Legend strip → drawn once on entry to ACTIVE.
- Full frame (135×240 × 16bpp = 64 KB) at 27 MHz SPI ≈ 19 ms. Dirty-region keeps rendering well under budget at 20 Hz.

### 4.6 Power management

- Motion idle detection: IMU accel magnitude deviation < small threshold for 90 s → dim backlight to 10 %. Continued idle for another 30 s → enter SLEEP.
- Before `esp_deep_sleep_start()`: **`M5.Axp.SetLDO3(false)`** to cut backlight power (otherwise backlight stays on and drains battery).
- Wake source: AXP192 power-key interrupt (not raw GPIO) — known M5StickC Plus-specific requirement.
- On wake, check for an in-progress session in RAM (preserved via RTC RAM or rehydrated from last-written NVS) → route to RESUME_PROMPT or SET_TARGET accordingly.

### 4.7 Module breakdown

Logical modules (separate files in `src/`):

| Module | Responsibility |
|---|---|
| `imu` | MPU6886 read + 10-second still-gyro bias capture at first boot (NVS) |
| `filter` | Mahony AHRS; outputs quaternion + gravity unit vector |
| `angle` | Pure function: `(g_ref, g_now, axis_hint) → (degrees, direction_sign)` |
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
- **Key libraries:** `m5stack/M5StickCPlus`, `M5GFX` (bundled transitively). Mahony AHRS implementation inlined (public domain, ~80 LOC).
- **Minimum toolchain:** Arduino-ESP32 core 2.x, PlatformIO 6.x.

## 6. Testing

### 6.1 Desktop unit tests (no hardware required)

- `angle.compute()`: feed canned `g_ref` + `g_now` pairs, assert degrees and direction sign.
- Stroke FSM: feed synthetic in/out-of-tolerance event sequences, assert expected counts including the >3 s unbroken edge case.
- Side FSM: feed synthetic spike/settle/sign sequences including the 2 s handling timeout case.

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

1. Target-angle input: capture + presets
2. Axis detection: orientation-agnostic (gravity-vector dot product)
3. Tolerance: 3 user-selectable presets at session start (±1° / ±2° / ±3°)
4. Mounting: magnet glued directly to device back
5. Feedback: tri-color screen (blue/green/red) + red LED + opt-in buzzer
6. Presets: 12° / 15° / 17° / 20° / 22°
7. Stroke detection: auto, in/out hysteresis with min durations
8. Side switch: auto via peel/settle + gravity sign; manual override on B
9. Session lifecycle: explicit, long-press A ends; auto-sleep on idle
10. Wake behavior: 5-second resume prompt, default to new session
11. Active screen: solid-color full-background + large current-side count + small other-side count
12. Framework: Arduino + PlatformIO + M5StickCPlus library
13. Persistence: tolerance + buzzer in NVS
14. Out of scope v1: BLE, logging, companion app, OTA
