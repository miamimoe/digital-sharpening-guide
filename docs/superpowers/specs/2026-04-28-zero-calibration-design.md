# Zero-Angle Calibration — Design Spec

**Date:** 2026-04-28
**Status:** Draft for approval
**Related:** `2026-04-23-sharpening-guide-design.md` (parent design)

## 1. Problem

The current firmware's reference gravity vector (`g_ref_`) is either:
- synthesized from a preset assuming the blade is perfectly horizontal in world frame (`{cos(r), 0, -sin(r)}`), or
- captured live from `filter_.gravity()` with the displayed angle derived as `asin(-g_ref.z)`.

Both paths bake a hidden assumption: that the world horizontal coincides with the stone surface and the magnet is mounted perfectly perpendicular to the blade flat. Neither is true in practice. Two error sources stack:

1. **Magnet-mount offset** — the magnet is glued by hand; the device back is not exactly perpendicular to the blade flat.
2. **Stone tilt** — the whetstone may sit on a non-horizontal bench, holder, or sink bridge.

Either source can put the displayed angle several degrees off the true blade-to-stone angle. With `NORMAL ±2°` tolerance, even a 1° offset materially distorts the green/red feedback that is the device's primary purpose.

## 2. Solution overview

Add a per-session `ZERO_CAL` state that captures the real reference geometry by physically demonstrating "blade flat on stone" — once per side. All angle math is then expressed relative to those captured zeros instead of a synthesized world frame.

The synthesized preset path and the freehand `asin(-g_ref.z)` path in `SET_TARGET` are **removed**. Target is reduced to a scalar angle in degrees; reference geometry comes from `g_zero_A` / `g_zero_B`.

## 3. State machine change

### 3.1 New transitions

```
SET_TOLERANCE
 │ (A=confirm)
 ↓
ZERO_CAL  ←── new state
 │ (two captures: side A then side B)
 ↓
ACTIVE
```

`ZERO_CAL` is inserted between `SET_TOLERANCE` and `ACTIVE` on the cold-start path. The wake path (`SLEEP → RESUME_PROMPT → ACTIVE`) bypasses `ZERO_CAL` because `g_zero_A` / `g_zero_B` survive in RTC RAM. New sessions started via `SUMMARY → A → SET_TARGET → ... → ZERO_CAL` re-capture both zeros from scratch.

### 3.2 ZERO_CAL substates

```
ZERO_CAL
├── PROMPT_A   — "Lay flat on stone (side A). Press A, hold still."
├── CAPTURE_A  — 500 ms warm-up + 1 s averaging window with stillness gate
├── PROMPT_B   — "Flip knife. Lay flat (side B). Press A, hold still."
├── CAPTURE_B  — same as CAPTURE_A
└── done → ACTIVE
```

Failure handling (per capture):
- If the stillness gate fails any time during warm-up or averaging, the substate restarts the warm-up automatically with the prompt "Hold still" (no manual retry button).
- No timeout. The user holds still until the gate passes; the device waits.
- Long-press `A` at any prompt aborts back to `SET_TARGET` (consistent with existing back-out conventions).

## 4. Capture procedure

For each zero (A and B):

1. **Wait for `A` press** at the prompt.
2. **Warm-up (500 ms):** sample raw accelerometer at 100 Hz, do not record. This lets any motion from the button press settle.
3. **Averaging window (1 s):** continue sampling raw accel at 100 Hz (100 samples). Record running sum and per-axis sum-of-squares for stddev.
4. **Stillness gate (continuous, evaluated each tick during both warm-up and averaging):**
   - `|‖a‖ − 1.0g| < 0.01g` (gravity magnitude tolerance, ~10× tighter than `side.cpp`'s 0.1g)
   - per-axis stddev `< 0.005g` over a trailing 200 ms window
   - `|gyro| < 0.5 dps` (catches slow drift the accel-magnitude check misses)
5. **On gate failure** at any sample: discard accumulator, restart at step 2 with "Hold still" prompt.
6. **On completion:** `g_zero = mean(samples)`, normalized. Persist to RTC RAM `SessionState`.

**Why bypass Mahony:** with `kp = 0.5` the filter has a ~2 s time constant; 1 s of averaging on filter output bakes in transients. When the device is genuinely still (gate passed), the raw accelerometer **is** the gravity vector — Mahony is needed for moving readings, not stationary ones.

## 5. Runtime math

### 5.1 Active reference selection

`g_zero_active` is selected by the side FSM's current side:
- `Side::A` → `g_zero_A`
- `Side::B` → `g_zero_B`

The side FSM (`side.cpp`) detects auto side switches via the `grav_dot_ref` polarity flip (negative on side A means gravity has crossed perpendicular to the reference). It needs a **fixed** reference vector whose polarity is unambiguous across sides — *not* `g_zero_active`, which would always be positive by construction. With this design, `grav_dot_ref = dot(g_now, g_zero_A)` permanently (g_zero_A is the polarity anchor). Side A: positive. Side B: negative. Existing FSM logic in `side.cpp` is unchanged. The angle classifier separately uses `g_zero_active` (selected by side) for magnitude.

### 5.2 Angle classifier

`compute_angle(g_zero_active, g_now)` returns:
- **magnitude:** `acos(dot(u_zero, u_now))` in degrees, unchanged from the current implementation.
- **direction sign:** retains the existing `N_BACK = {0, 0, -1}` device-frame projection trick. This works correctly across both sides because `N_BACK` is in body frame (rotates with the device when the knife flips), and the projection differential `dot(u_now, N_BACK) − dot(u_zero, N_BACK)` is sign-symmetric across the flip.

Classifier (`classify` function):
- `magnitude_deg ∈ [target_deg − tolerance_deg, target_deg + tolerance_deg]` → GREEN.
- `magnitude_deg < target_deg − tolerance_deg` → BLUE (raise spine).
- `magnitude_deg > target_deg + tolerance_deg` → RED (lower spine).
- direction sign disambiguates BLUE vs. RED when magnitude lies outside tolerance; the existing zero-direction safety fallback to GREEN is preserved.

### 5.3 Removed paths

- Preset synthesis `g_ref_ = {cos(r), 0, -sin(r)}` (deleted from `app.cpp::handle_set_target`).
- Freehand derivation `target_deg_ = asin(-g_ref.z) * 180/π` (deleted).

`SET_TARGET` now stores only a scalar `target_deg_`. Both preset selection and "freehand" become numeric: presets pick from `{12, 15, 17, 20, 22}`; freehand mode is removed (it depended on the world-horizontal assumption — there is no longer any meaningful "capture current angle" without a zero).

## 6. Persistence

### 6.1 RTC RAM (`SessionState`)

Two fields added to `SessionState` in `src/session.h`:

```cpp
Vec3 g_zero_A = {0.0f, 0.0f, 0.0f};
Vec3 g_zero_B = {0.0f, 0.0f, 0.0f};
```

`g_ref` is **removed** from `SessionState` (replaced by these two).

`App::begin` restores both fields from `session::state()` on wake. If either is the zero vector (incomplete cal), the resume path force-routes to `ZERO_CAL` instead of `ACTIVE` — defensive, even though the wake path normally has both populated.

### 6.2 NVS

No NVS persistence for zeros. They are session-scoped: a new stone, a different mount, a re-glue all invalidate them. Persisting across power cycles risks silently using a stale zero from days ago.

## 7. UI text

| Screen | Text |
|---|---|
| `ZERO_CAL` PROMPT_A | "ZERO CAL  1/2"<br>"Lay knife flat on stone."<br>"Press A and hold still." |
| `ZERO_CAL` CAPTURE_A | "Hold still… 1.0s" (counts down) |
| `ZERO_CAL` retry | "Hold still" (red text) |
| `ZERO_CAL` PROMPT_B | "ZERO CAL  2/2"<br>"Flip knife. Lay flat."<br>"Press A and hold still." |
| `ZERO_CAL` CAPTURE_B | "Hold still… 1.0s" (counts down) |

Layout follows the existing `SET_TARGET` / `SET_TOLERANCE` text conventions (tiny header, large body). No new font work needed.

## 8. Files affected

| File | Change |
|---|---|
| `src/types.h` | No change (Vec3 reused). |
| `src/session.h` | Add `g_zero_A`, `g_zero_B`. Remove `g_ref`. |
| `src/session.cpp` | Update RTC field layout; bump struct version sentinel if applicable. |
| `src/app.h` | Add `State::ZERO_CAL`. Replace `g_ref_` field with `g_zero_A_`, `g_zero_B_`. Add ZERO_CAL substate enum + working accumulators. |
| `src/app.cpp` | New `handle_zero_cal()`. Remove synthesized preset / freehand g_ref code in `handle_set_target()`. Update `handle_active()` to select `g_zero_active` by side and pass to `compute_angle`. Update `grav_dot_ref` computation in side-FSM call site. |
| `src/angle.h` / `src/angle.cpp` | Function signature unchanged: still `compute_angle(g_ref, g_now)` — the meaning of the first argument is now `g_zero_active`, not a synthesized target vector. The classifier signature changes: `classify` now takes `(magnitude_deg, target_deg, tolerance_deg, direction_sign)` instead of `(AngleResult, tolerance_deg)`. |
| `src/imu.cpp` / `src/imu.h` | Expose a `raw_accel()` accessor returning the most recent accelerometer sample directly from the MPU6886 read (no Mahony, no low-pass, no scaling beyond the existing g-units conversion). If the IMU module already exposes raw accel, no new function is needed — the spec only requires that the capture path can read raw accel without going through the Mahony filter. |
| `src/ui.cpp` / `src/ui.h` | Add `draw_zero_cal_prompt(step, retry)` and `draw_zero_cal_progress(remaining_ms)`. |
| `src/filter.cpp` / `src/filter.h` | No change — Mahony is bypassed for capture, not modified. |
| `src/side.cpp` | No change — already keys off `grav_dot_ref` opaquely; only the upstream reference vector changes. |
| `src/stroke.cpp` | No change. |
| `src/feedback.cpp` | No change. |
| `src/power.cpp` | No change. |
| `src/main.cpp` | No change beyond what `app.cpp` requires. |
| Native tests | Add `test_zero_cal` suite for the capture state machine + averaging logic. Update `test_angle` if classifier signature changes. |

## 9. Testing

### 9.1 Unit (native)

- **`test_zero_cal`** — capture FSM with mocked sample stream:
  - happy path (still samples → completes in 1.5 s).
  - jitter during warm-up restarts warm-up.
  - jitter during averaging discards and restarts.
  - capture A then capture B both populate correct fields.
  - long-press A aborts back to SET_TARGET.
- **`test_angle`** — adapt to new classifier signature; magnitude correctness against `g_zero` for arbitrary `g_zero` orientations (not just world-vertical).
- **Stillness gate boundary cases** — exactly at `0.01g` magnitude error, exactly at `0.5 dps` gyro magnitude.

### 9.2 Hardware bring-up additions

Add to `docs/superpowers/bringup/2026-04-23-hardware-bringup.md`:
- ZERO_CAL repeatability: capture 5 zeros in a row on the same physical pose; confirm angular spread between captured vectors `< 0.5°`.
- Cross-side correctness: capture A and B on a real blade against a real stone; verify in-tolerance angle near target_deg on each side.
- Recovery: induce stillness-gate failure (tap the device) and confirm "Hold still" retry path.

## 10. Out of scope (v1)

- NVS persistence of zeros (rejected — see §6.2).
- Re-zero shortcut during ACTIVE (long-press B re-cal). Possible v2 if missed in practice.
- Single-zero shortcut (capture A only, infer B by negation). Rejected — fragile against mount asymmetry.
- Auto-learned `tilt_axis` for direction sign. Rejected — `N_BACK` is sufficient and orientation-symmetric across side flips.
- Tilting-the-target ("set angle by physical demo") — rejected as redundant with the preset path now that the magnitude reference is physical.

## 11. Risks to validate on hardware

1. **Mahony warm-up at ZERO_CAL entry** — even though we bypass Mahony for the capture itself, the filter is still updating in the main loop. If a side switch triggers some downstream Mahony reset, validate that ACTIVE entry doesn't show a transient bad reading.
2. **Stillness threshold realism** — `0.01g` magnitude tolerance plus `0.005g` per-axis stddev plus `0.5 dps` gyro is tight. May be unachievable on a sharpening setup if the bench has any vibration. If gate failures are common in real use, loosen progressively (first relax stddev, then magnitude, last gyro).
3. **Gyro-bias-already-corrected assumption** — the existing 10 s `BIAS_CAL` runs on first boot only. If the device has been sitting for weeks since flash, gyro bias may have drifted. The `0.5 dps` gate catches gross drift but not slow constant offsets. Hardware bring-up should validate that BIAS_CAL output remains valid across long idle.
4. **Magnet placement asymmetry** — if the magnet is glued at a slight offset, `g_zero_A` and `g_zero_B` will not be exact negatives of each other in body frame. This is **fine** — the design captures both independently and does not assume any relationship between them. Risk is only that the user might mount the magnet in a position where one side cannot lay fully flat (e.g., handle interferes); document in bring-up.

## 12. Open questions

None. All major UX and architectural decisions resolved during brainstorming + QA review pass.
