# Multi-Board Support: M5StickC Plus2 + M5StickS3

**Date:** 2026-06-26
**Status:** Design — approved for planning
**Author:** Moe Fayed (with Claude)

## Why

The v0.1.0 release (M5StickC Plus) went public and got real traction. The top hardware
request is to run on newer sticks. We will add two more boards as first-class targets
while keeping the proven Plus firmware untouched.

## Scope decisions (locked)

- **Targets:** M5StickC Plus (existing, frozen) + **M5StickC Plus2** + **M5StickS3**, all first-class.
- **Structure:** **one codebase**, divergent hardware code **guarded per-board at compile time** so the
  Plus build stays byte-for-byte what it is today. No fork.
- **Release:** all three published as **equal/stable** in the browser flasher.
- **Preservation guarantee:** the original is already frozen independently as git tag `v0.1.0`
  and the flashable binary `docs/firmware/sharpening-guide-v0.1.0.bin`. Nothing here can alter those.

## Out of scope (YAGNI)

- No BLE, logging, companion app, or battery-level UI (unchanged repo policy).
- No runtime multi-board single-binary. Each board gets its own compiled binary, selected by
  PlatformIO env. (Plus and Plus2 are different ESP32 boards with different power hardware; S3 is a
  different silicon family entirely — one binary cannot serve all three regardless.)
- No new behavior. A sharpening guide does the same thing on every board; only the hardware glue differs.

## Hardware delta (research summary)

"M5Stack stick S3" is ambiguous in the wild — it can mean either the Plus2 (plain ESP32, *not* S3)
or the genuine ESP32-S3 stick. We support both.

| Concern | M5StickC **Plus** (baseline) | M5StickC **Plus2** | **M5StickS3** |
|---|---|---|---|
| SoC | ESP32-PICO-D4 | ESP32-PICO-V3-02 (plain ESP32) | ESP32-S3-PICO-1-N8R8 (S3) |
| Flash / PSRAM | 4 MB / none | 8 MB / 2 MB | 8 MB / 8 MB octal |
| IMU | MPU6886 | MPU6886 (same) | **BMI270** |
| Power mgmt | AXP192 (I²C) | **none** — HOLD pin G4 | **M5PM1** PMIC |
| Display | ST7789V2 135×240 | ST7789V2 135×240 | ST7789P3 135×240 |
| Audio | passive buzzer (G2) | passive buzzer | **ES8311 codec + speaker** |
| Red LED | G10 | G19 | G19 |
| Power button | AXP192 IRQ → GPIO35 | Button C on GPIO35 | M5PM1 power button |

The pin/PMIC details the research flagged as uncertain on the S3 (exact button GPIOs, M5PM1 vs AXP2101
labeling) are **deliberately sidestepped**: buttons go through `M5.BtnA/BtnB/BtnPWR` and power through
`M5.Power`, both auto-detected by M5Unified. We only touch one raw GPIO (the LED).

## Architecture

### Why the port is feasible at all

The codebase is already M5Unified-first and cleanly split:

- **Pure-logic modules** (`angle`, `stroke`, `side`, `filter`, `input`, and the `app` state machine):
  no hardware references, unit-tested on desktop. **These do not change.** Future stroke/Mahony
  threshold tuning therefore lands once and benefits all three boards.
- **Hardware-coupled modules** (`imu`, `ui`, `feedback`, `power`): Arduino-only. Audited, only three
  spots are actually wired to Plus-specific hardware.

### The only three divergent spots

1. **`power.cpp` — deep-sleep / wake.** Today: AXP192 register writes + `esp_sleep_enable_ext0_wakeup`
   on GPIO35. This is the one substantial change.
2. **`imu.cpp:27` — `getType() == m5::imu_mpu6886` gate.** Would FAULT on the S3's BMI270.
3. **`feedback.cpp:7` — `LED_PIN = 10`.** Different GPIO on Plus2/S3.

`feedback`'s audio path (`M5.Speaker.tone`) already works on the S3 codec — no change. `ui` uses
`M5.Display` abstractions — no change. `imu`'s `getAccel/getGyro` are board-agnostic — no change beyond
the type gate.

### Board selection mechanism

A single header `src/board.h` defines, per build, one of three board variants from a PlatformIO
build flag, plus the per-board constants the three spots need:

```cpp
// Exactly one of these is defined by the active env's build_flags:
//   -D SG_BOARD_PLUS   -D SG_BOARD_PLUS2   -D SG_BOARD_S3
// board.h validates exactly-one-defined (#error otherwise) and exposes:
namespace board {
    constexpr int  led_pin();          // 10 (Plus) | 19 (Plus2/S3), active LOW
    constexpr bool has_axp192();       // true only on Plus
    enum class Variant { PLUS, PLUS2, S3 };
    constexpr Variant variant();
    bool imu_type_ok(m5::imu_t t);     // mpu6886 (Plus/Plus2) | bmi270 (S3)
}
```

- **Compile-time guards**, not runtime branches, are what give the "Plus binary byte-identical"
  guarantee: when building the Plus env, no Plus2/S3 code is compiled in. The Plus branch in every
  guarded spot is the **verbatim** current code.
- **Runtime safety assert (belt-and-suspenders):** at boot, each binary checks `M5.getBoard()` matches
  its compiled variant; on mismatch it shows a clear "wrong firmware for this device" FAULT screen
  instead of misbehaving. This protects a user who flashes the S3 binary onto a Plus2, etc.

### `power.cpp` per board

`enter_deep_sleep()` and the wake handling branch on the compiled variant:

- **Plus (AXP192):** unchanged — verbatim current logic (clear AXP IRQ regs 0x44–0x4D, EXT0 on GPIO35).
- **Plus2 (no PMIC):** ensure the G4 power-latch is held at boot so the device stays on; sleep via
  `esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0)` + `M5.Power.deepSleep()`. No AXP register I/O.
- **S3 (M5PM1):** sleep/wake through `M5.Power.deepSleep()` and M5Unified's S3 power-button wake; no
  raw register or EXT0 pin assumptions. Exact wake wiring confirmed against the installed M5Unified
  version during implementation.

The pure-logic half of `power.cpp` (`config_for`, `check_idle`, `update_backlight`) is unit-tested and
unchanged.

### `setup()` / `main.cpp`

`M5.begin(cfg)` already auto-detects the board and brings up the right IMU/display/power/speaker
drivers. The only `main.cpp` change: the `M5.Power.getKeyState()` IRQ-consume call is Plus/AXP-specific
(M5Unified documents `getKeyState()` as "for devices with AXP192/AXP2101") and is guarded with
`board::has_axp192()`.

### Docs verification (Context7, `/m5stack/m5unified`, 2026-06-26)

Confirmed against current M5Unified docs:

- **`m5::imu_bmi270` is a real `imu_t` enum value** (alongside `imu_mpu6886`, `imu_none`, etc.). The S3's
  BMI270 is auto-detected and read through the same `M5.Imu.getAccel/getGyro` calls — no IMU code change
  beyond relaxing the type gate.
- **`M5.getBoard()` returns `m5::board_t`** at runtime — the wrong-firmware safety assert is feasible.
- **`M5.Power.deepSleep(microseconds)`, `powerOff()`, `lightSleep()`, `timerSleep()` exist** across the
  family; `getKeyState()` is documented as AXP192/AXP2101-only (confirms the `has_axp192()` guard).

Pinned during the first implementation task (version-sensitive, verified against the *installed*
M5Unified header, not assumed): the exact `m5::board_t` constants for Plus2 and S3 (expected
`board_M5StickCPlus2` / `board_M5StickS3`), the minimum M5Unified version carrying StickS3 support, and
the S3 deep-sleep wake source (EXT0 pin vs. M5Unified-managed power-button wake).

## Build targets

`platformio.ini` gains two envs; the existing one is left functionally identical (add only its
`-D SG_BOARD_PLUS` selector):

| Env | `board` | Key flags | M5Unified |
|---|---|---|---|
| `m5stick-c-plus` (existing) | `m5stick-c` | `-D SG_BOARD_PLUS` (+ today's flags) | current pin |
| `m5stick-c-plus2` (new) | `m5stick-c` | `-D SG_BOARD_PLUS2 -D BOARD_HAS_PSRAM` | bumped |
| `m5stick-s3` (new) | S3 board (exact id pinned in plan) | `-D SG_BOARD_S3` + octal-PSRAM flags | bumped |
| `native` (existing) | native | unchanged | n/a |
| `diag` (existing) | extends plus | unchanged | n/a |

- **M5Unified version:** bump from `^0.2.14` to a release that includes StickS3 board support + the S3
  `powerOff`/detection fixes. Exact minimum version pinned in the implementation plan after verifying
  against the M5Unified changelog (Context7 / GitHub releases). The Plus env keeps building against the
  same (newer) version — verified non-regressing, since M5Unified is backward compatible for the Plus.
- **S3 board id:** M5Stack does not ship an official PlatformIO board JSON for the StickS3 as of mid-2026.
  The plan will either point at a generic ESP32-S3 board def (8 MB flash, octal PSRAM) or add a vendored
  `boards/m5stick-s3.json`. Decided in the first implementation task.

## Browser flasher (`docs/`)

- `manifest.json` → one manifest per board (ESP-Web-Tools needs `chipFamily` to match: `ESP32` for
  Plus/Plus2, `ESP32-S3` for S3). Use separate manifests + a device picker on the page.
- `docs/firmware/` gains the Plus2 and S3 binaries alongside the preserved v0.1.0 Plus binary.
- `index.html` → add a small device selector (Plus / Plus2 / S3) that swaps the manifest the
  install button points at; generalize the copy from "M5StickC Plus" to the three devices.
- Version each binary in its filename (`sharpening-guide-<board>-vX.Y.Z.bin`).

## Testing strategy

**Honest ceiling: neither maintainer has the Plus2 or S3 hardware.** "Tested" here means the maximum
verifiable without the device. We will NOT claim on-device validation we cannot perform.

What we verify:

1. **Native unit suite** (`pio test -e native`) stays green — the pure-logic brain is unchanged, so this
   proves no regression in angle/stroke/side/filter/app logic.
2. **All firmware envs compile** — `pio run -e m5stick-c-plus`, `-e m5stick-c-plus2`, `-e m5stick-s3`
   all build clean with `-Wall -Wswitch-enum`. This is the build-verification gate.
3. **Plus non-regression** — confirm the `m5stick-c-plus` build is unchanged vs v0.1.0 (the guarded
   Plus branches are verbatim; spot-check the binary / map). Plus is also still validated on real
   hardware (the maintainer's device).
4. **Line-by-line review against datasheets** — each board's `power.cpp` branch, LED pin, and IMU type
   reviewed against the M5Stack docs/pinouts during the QA pipeline.

What we cannot verify (must be stated in README + flasher): real-device angle colors, buzzer/speaker
audibility, sleep/wake, and magnet grip on Plus2/S3.

## QA pipeline

Per repo policy, every implementation task runs: current-docs research → implementer subagent → spec +
code-quality review subagent → CodeRabbit CLI → fix loop until green. The maintainer is AFK during
execution; review fixes auto-iterate.

## Risks & open items

1. **No Plus2/S3 hardware for on-device test** — mitigated by build gate + datasheet review + runtime
   wrong-firmware FAULT guard; disclosed honestly at release.
2. **M5Unified version bump touches the Plus build** — mitigated: Plus stays on the same newer library,
   compile + native tests must stay green, binary spot-checked for non-regression.
3. **S3 PlatformIO board definition** — no official JSON; resolved in first impl task (generic S3 def or
   vendored board file), pinned explicitly.
4. **S3 power/sleep wiring uncertainty** — sidestepped by routing through `M5.Power`/M5Unified rather
   than raw registers; exact API confirmed against the pinned M5Unified version.
5. **BMI270 data-ready freshness** — `main.cpp` treats a `false` from `imu::read()` as "no fresh sample
   this tick" and only faults after 25 consecutive misses. The MPU6886's data-ready cadence is known;
   BMI270's may differ. Behavior is benign either way (logic only acts on fresh samples), but the IMU
   fault threshold is reviewed for the S3 during implementation. No on-device confirmation possible.
6. **Threshold tuning still open** (existing risk) — unchanged and now shared across all three boards.
