# Contributing

Thanks for taking an interest! This is a small, solo hobby project, but contributions are genuinely welcome.

## The most useful thing right now: tuning data

The stroke counter and angle filter are tuned from a handful of sessions. If you sharpen with it, please open an [issue](https://github.com/miamimoe/digital-sharpening-guide/issues) with:

- **Hand-counted strokes vs. what the device reported** (per side).
- Your **knife, stone, and target angle**.
- Anything that felt laggy, jumpy, or wrong.

Real numbers from real stones beat any amount of bench guessing.

## Code contributions

- The pure-logic modules (`angle`, `stroke`, `side`, `filter`, `input`, `zero_cal`) are unit-tested on the desktop. **Add/adjust tests** when you change them:
  ```bash
  pio test -e native
  ```
- Hardware-coupled modules (`imu`, `ui`, `feedback`, `power`) are validated against the checklist in [`docs/superpowers/bringup/`](docs/superpowers/bringup/), not unit tests.
- Build before you push:
  ```bash
  pio run -e m5stick-c-plus
  ```
- Keep it lean. This firmware deliberately avoids BLE, logging, and a companion app. If you're adding a dependency, justify the flash/RAM cost.
- Commit style: `type(scope): summary` (e.g. `fix(stroke): ...`, `feat(ui): ...`, `docs: ...`).

## Hardware notes

Targets the original **M5StickC Plus** (ESP32-PICO + MPU6886 + ST7789V2 + AXP192). Ports to other M5 sticks (e.g. Plus2) are welcome but untested — expect the power and IMU paths to need work.

By contributing, you agree your contributions are licensed under the project's [MIT License](LICENSE).
