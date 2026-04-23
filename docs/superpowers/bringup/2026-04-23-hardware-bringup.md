# Hardware Bring-Up — Sharpening Guide v1

Run once, in order, when the M5StickC Plus Watch Kit arrives. Each step has an expected observation — record actuals; any mismatch gates the next step until resolved.

## Pre-flight

- [ ] Cable connects; `pio run -e m5stick-c-plus -t upload` flashes firmware without error
- [ ] Serial monitor at 115200 baud shows no ESP32 panic / reset loop
- [ ] Battery charge LED visible when USB-C is connected

## 1. I²C sanity (spec risk #3 — MPU6886/AXP192 conflict)

- [ ] Fresh flash → device enters BIAS_CAL screen (first-boot flag was set)
- [ ] If instead you see FAULT with code:
  - `E01` — `M5.Imu.isEnabled()` returned false. Wrong `board` in `platformio.ini`, hardware defect, or incompatible M5Unified firmware baseline. Try `esptool.py --chip esp32 erase_flash` + re-flash.
  - `E02` — first 10 `read()` attempts all returned false. Possible IMU self-test failure; inspect serial log for I²C errors.
  - `E03` — `getType()` returned something other than `m5::imu_mpu6886`. Most likely indicator of the MPU6886/AXP192 I²C address conflict. Workarounds to try:
    1. Erase flash and re-flash with a different M5Unified firmware version (try 0.2.10 then 0.2.14)
    2. Examine `.pio/libdeps/.../M5Unified/src/utility/IMU_Class.cpp` for how it distinguishes the two chips
    3. Worst case: pin `M5.Imu.begin(Wire1)` and physically split the buses

## 2. Gyro bias capture (first boot only)

- [ ] BIAS_CAL screen shows "Hold still — Calibrating" with countdown
- [ ] Leaving device on bench → countdown reaches 0 after 10 seconds
- [ ] Picking up the device → countdown restarts
- [ ] Sustained motion > 60 seconds → capture times out (`capture_gyro_bias` returns false), app proceeds with zero-bias gracefully
- [ ] After success, power-cycle: BIAS_CAL does NOT re-run (first-boot flag cleared in NVS)

## 3. Orientation convention (spec §4.3 — `n_back` sign validation)

- [ ] Lay device screen-up on a level surface. Enter SET_TARGET. Add a one-off serial log dump of `filter_.gravity()` — look for z ≈ −1.0 g.
- [ ] If z ≈ +1.0 g: flip the sign of `N_BACK` in `src/angle.h` from `{0,0,-1}` to `{0,0,1}`, rebuild, re-verify.
- [ ] Tilt device 45° nose-up: X goes to ≈ −0.7 g, Z goes to ≈ −0.7 g.

## 4. Angle display smoke test

- [ ] In SET_TARGET, the live angle readout:
  - Shows ~0° flat on the table
  - Smoothly increases as you tilt device up around its long axis
  - Reaches ~90° at vertical
- [ ] Readout update rate feels smooth (~20 Hz; not laggy, not jumpy)

## 5. Capture + tri-color feedback (spec §4.3 directional signal)

- [ ] Hold device at ~15° against a protractor or phone inclinometer, press A → advances to SET_TOLERANCE
- [ ] Confirm NORMAL (±2°) → advances to ACTIVE
- [ ] Screen is **GREEN** while held near capture angle
- [ ] Tilt further up (angle increases past target + tolerance) → **RED**
- [ ] Tilt back down past capture - tolerance → **BLUE**
- [ ] Directional colors correct? RED = "too high", BLUE = "too low"? If reversed, flip sign of `N_BACK` and re-run item 3

## 6. Stroke counter (spec §4.4 — hysteresis)

- [ ] Hold within tolerance > 300 ms, drift out > 200 ms → count increments by 1
- [ ] Quick < 300 ms tap-into-green → no count
- [ ] Wobble < 200 ms mid-pass → does not end stroke
- [ ] Record 10 real sharpening strokes → count should be ~10 (allow ±1)
- [ ] If count is consistently low, lower `StrokeFSM::IN_MIN_MS` below 300 ms; if high, raise it

## 7. Side switch (spec §4.4 — peel/settle)

- [ ] Peel device off test surface (or shake ~1.5g+), reattach rotated 180° → side switch within ~0.5 s of re-settle
- [ ] Peel and place back in same orientation → no switch
- [ ] B short-press in ACTIVE → manual side toggle
- [ ] Pressing B during peel (within 2 s) → no auto-switch (suppression works)
- [ ] Peel + sustained motion > 5 s (no settle) → FSM resets without switching

## 8. Buzzer toggle (spec §3.3, §3.2 ACTIVE)

- [ ] ACTIVE, long-press B (≥800 ms) → "BUZZER ON" overlay appears, persists ~800 ms, then clears
- [ ] Tilt out of tolerance → audible beep
- [ ] Long-press B again → "BUZZER OFF" — subsequent tilts are silent
- [ ] Power-cycle → buzzer state persists (NVS)

## 9. Deep sleep + wake (spec §4.6; board-revision risk #4)

- [ ] From SET_TARGET, don't touch device for 2 min → backlight dims at 90 s, enters SLEEP at 120 s
- [ ] Press power-key → device wakes. If wake fails:
  - Try `(1ULL << 37)` in `src/power.cpp` (BtnA GPIO on some revs)
  - Try `(1ULL << 39)` (BtnB)
  - Consult M5Unified's `Power_Class` source for the AXP192 IRQ routing on this board
- [ ] In ACTIVE with a captured angle + at least one stroke, long-press A → SUMMARY → wait 5 min no strokes → SLEEP → wake → expect RESUME_PROMPT
- [ ] RESUME_PROMPT + A → resumes with counts intact
- [ ] RESUME_PROMPT + 5 s timeout → new SET_TARGET (stats cleared)

## 10. Mahony gain tuning (spec risk #1)

- [ ] Attach device magnetically to a real knife's flat
- [ ] Sharpen for 2–3 min on a real stone with real motion
- [ ] If color transitions visibly LAG the actual angle (>200 ms delay), raise `kp` from 0.5 to 1.0 in `src/filter.h` / `filter.cpp`
- [ ] If transitions are JUMPY during smooth strokes (flicker between green/red), lower `kp` to 0.3 and/or raise `ki` from 0 to 0.05
- [ ] Record final tuned `kp` / `ki` in a commit with message `tune(filter): real-hardware kp/ki`

## 11. Stroke threshold tuning (spec risk #2)

- [ ] Add a serial log dumping the stroke FSM state transitions during a 60-second sharpening session
- [ ] Analyze actual IN/OUT durations from log
- [ ] Adjust `StrokeFSM::IN_MIN_MS` / `OUT_MIN_MS` in `src/stroke.h` if the 300/200 ms guesses don't match
- [ ] Re-run `pio test -e native -f test_stroke` — all 9 tests must still pass with the new thresholds

## 12. Magnet mount (spec risk #5)

- [ ] Attach neodymium magnet (≥10 mm × 5 mm N35 or stronger) to device back — try 5-min epoxy first, fall back to high-strength VHB tape
- [ ] Mount to typical kitchen-knife flat, sharpen, confirm no shift/slip
- [ ] If slip: upgrade magnet strength or add a thin silicone/rubber grip layer between magnet and blade

## 13. Battery-life measurement

- [ ] Full charge to ~4.2 V
- [ ] Run in ACTIVE continuously with full backlight — measure time until auto-sleep or voltage < 3.3 V
- [ ] Expected: 45–90 min (120 mAh cell, full-screen color fill + 100 Hz IMU is energy-hungry)
- [ ] If < 45 min: check the efficiency audit doc for unchecked wins (lower sample rate, deeper dim, etc.)

## Sign-off

Date hardware bring-up completed: ____________
Any outstanding spec risk unresolved: ____________
Amendments required to spec or code: ____________
