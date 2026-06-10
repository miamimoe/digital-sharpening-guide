# Digital Sharpening Guide

Firmware that turns an **M5StickC Plus** (ESP32-PICO + MPU6886 IMU + 1.14" LCD + AXP192 PMIC) into a knife-sharpening angle guide. A neodymium magnet glued to the back sticks the device to the blade's flat; the screen then gives live, glanceable feedback while you sharpen:

- 🟩 **Green** — you're holding the target angle within tolerance
- 🟦 **Blue** — angle too low, raise the spine
- 🟥 **Red** — angle too high, lower the spine

Plus automatic stroke counting per side, an optional out-of-tolerance buzzer, and a session summary.

## How it works

- **Edge-axis bevel measurement.** A two-step zero calibration (lay the blade flat on the stone, then raise the spine to your angle) captures both a flat reference and the cutting-edge hinge axis. The bevel angle is measured as rotation *about that axis*, so tip-to-heel skew doesn't inflate the reading, and one calibration serves both blade faces.
- **Mahony AHRS filter** fuses gyro + accelerometer at 50 Hz, with per-session gyro-bias capture and a snap-to-raw recovery when the device is verifiably still.
- **Motion-based stroke counting.** Passes are detected as horizontal linear-acceleration peaks (hysteresis + refractory interval) while on-angle — not from angle-dwell timing.
- **Battery-aware.** Deep sleep on idle with the session preserved in RTC RAM (wake resumes where you left off), screen dimming, 80 MHz CPU, and a one-click sleep/wake power key.

## Controls

| Button | Short press | Long press |
|---|---|---|
| A (front) | confirm / re-zero in session | end session |
| B (side) | cycle option / switch blade side | toggle buzzer |
| Power key | sleep / wake | 6 s = hard power-off |

## Build & flash

Built with [PlatformIO](https://platformio.org/) on the Arduino framework; the only dependency is M5Unified.

```bash
pio run -e m5stick-c-plus              # build
pio test -e native                     # unit tests (pure-logic modules, desktop)
pio run -e m5stick-c-plus -t upload    # flash
```

## Layout

```
src/        firmware modules — app state machine, angle math, Mahony filter,
            stroke/side/input FSMs, zero-cal capture, UI, power, persistence
test/       native (desktop) unit tests for the pure-logic modules
docs/       design spec, implementation plan, hardware bring-up checklist
```

Status: hardware-validated; stroke/filter thresholds still being tuned against real sharpening sessions.
