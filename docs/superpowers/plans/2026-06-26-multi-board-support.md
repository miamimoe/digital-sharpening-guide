# M5StickC Plus2 + M5StickS3 Multi-Board Support — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the M5StickC Plus2 and M5StickS3 as first-class flash targets in one codebase, without altering the proven M5StickC Plus build.

**Architecture:** One source tree. A new `src/board.h` selects a board variant from a per-env build flag and exposes the few constants that differ. The three hardware-coupled spots (`power.cpp` sleep/wake, `imu.cpp` IMU-type gate, `feedback.cpp` LED pin) branch on that variant at **compile time**, so the Plus build compiles zero new-board code and stays byte-identical. Pure-logic modules are untouched.

**Tech Stack:** C++ (gnu++11-constrained), Arduino-ESP32, M5Unified, PlatformIO, Unity (native tests), ESP Web Tools (browser flasher).

## Global Constraints

- **Toolchain forces `-std=gnu++11`** even when gnu++17 is set: **no digit separators** (`90'000`), **no structured bindings**. Plain literals and explicit struct decomposition only.
- **`-Wswitch-enum` is on:** every `switch` over an `enum class` must handle all enumerators (or end with `__builtin_unreachable()` where appropriate).
- **No silent regression to the Plus:** the `m5stick-c-plus` env's compiled output must remain functionally identical to v0.1.0. Each guarded Plus branch is the **verbatim** current code.
- **Keep it lean (YAGNI):** no new deps, no new runtime features. M5Unified is the abstraction layer — prefer its APIs over raw registers/GPIO (the only raw GPIO is the LED).
- **M5Unified APIs confirmed (Context7, `/m5stack/m5unified`):** `m5::imu_bmi270` and `m5::imu_mpu6886` are valid `imu_t` values; `M5.getBoard()` returns `m5::board_t`; `M5.Power.deepSleep([us])` exists family-wide; `getKeyState()` is AXP192/AXP2101-only.
- **PlatformIO board ids:** Plus/Plus2 use `board = m5stick-c`; the S3 board id is resolved in Task 2.
- **Commit trailer on every commit:** `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. Commit type/scope format: `<type>(<scope>): <summary>`. Never `--no-verify`.
- **Build/test commands:** `pio` is at `/Users/moefayed/.local/bin/pio`.
  - Native tests: `pio test -e native`
  - Build a target: `pio run -e <env>`

---

### Task 1: `board.h` board-abstraction header

**Files:**
- Create: `src/board.h`
- Test: `test/test_board/test_board.cpp`
- Modify: `platformio.ini` (add `-D SG_BOARD_PLUS` to the `m5stick-c-plus` env's `build_flags`)

**Interfaces:**
- Produces: `board::Variant` (`enum class { PLUS, PLUS2, S3 }`), `board::variant() -> Variant`, `board::led_pin() -> int`, `board::has_axp192() -> bool`. All `constexpr`, no M5Unified dependency (so the header is native-compilable). Under `UNIT_TEST` with no `SG_BOARD_*` set, defaults to `PLUS`.

- [ ] **Step 1: Write the failing test**

`test/test_board/test_board.cpp`:
```cpp
#include <unity.h>
#include "board.h"

void setUp(void) {}
void tearDown(void) {}

// Native build defines no SG_BOARD_* flag, so board.h must default to PLUS.
void test_native_default_is_plus(void) {
    TEST_ASSERT_TRUE(board::variant() == board::Variant::PLUS);
}

void test_plus_led_pin_is_10(void) {
    TEST_ASSERT_EQUAL_INT(10, board::led_pin());
}

void test_plus_has_axp192(void) {
    TEST_ASSERT_TRUE(board::has_axp192());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_native_default_is_plus);
    RUN_TEST(test_plus_led_pin_is_10);
    RUN_TEST(test_plus_has_axp192);
    return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `/Users/moefayed/.local/bin/pio test -e native -f test_board`
Expected: FAIL — `board.h` not found / does not compile.

- [ ] **Step 3: Write `src/board.h`**

```cpp
#pragma once

// Board-variant selection. Exactly one SG_BOARD_* macro must be defined by the
// active PlatformIO env's build_flags. Under native unit tests no board flag is
// set, so default to PLUS to keep shared code compilable.
#if defined(UNIT_TEST) \
    && !defined(SG_BOARD_PLUS) && !defined(SG_BOARD_PLUS2) && !defined(SG_BOARD_S3)
#define SG_BOARD_PLUS
#endif

#if (defined(SG_BOARD_PLUS) + defined(SG_BOARD_PLUS2) + defined(SG_BOARD_S3)) != 1
#error "board.h: define exactly one of SG_BOARD_PLUS / SG_BOARD_PLUS2 / SG_BOARD_S3"
#endif

namespace board {

enum class Variant { PLUS, PLUS2, S3 };

constexpr Variant variant() {
#if defined(SG_BOARD_PLUS)
    return Variant::PLUS;
#elif defined(SG_BOARD_PLUS2)
    return Variant::PLUS2;
#else
    return Variant::S3;
#endif
}

// Red status LED GPIO (active LOW). Plus routes it to GPIO10; Plus2 and S3 to GPIO19.
constexpr int led_pin() {
#if defined(SG_BOARD_PLUS)
    return 10;
#else
    return 19;
#endif
}

// True only on the original Plus, whose PMIC is the AXP192 (I2C). The Plus2 has
// no PMIC (HOLD-pin power latch); the S3 uses the M5PM1. Used to gate AXP-only code.
constexpr bool has_axp192() {
#if defined(SG_BOARD_PLUS)
    return true;
#else
    return false;
#endif
}

} // namespace board
```

- [ ] **Step 4: Add the board selector to the existing Plus env**

In `platformio.ini`, under `[env:m5stick-c-plus]` `build_flags`, add `-D SG_BOARD_PLUS` alongside the existing `-D ARDUINO_M5StickC_Plus` line.

- [ ] **Step 5: Run test to verify it passes**

Run: `/Users/moefayed/.local/bin/pio test -e native -f test_board`
Expected: PASS (3 tests).

- [ ] **Step 6: Verify the Plus firmware still builds**

Run: `/Users/moefayed/.local/bin/pio run -e m5stick-c-plus`
Expected: SUCCESS.

- [ ] **Step 7: Commit**

```bash
git add src/board.h test/test_board/test_board.cpp platformio.ini
git commit -m "feat(board): add compile-time board-variant abstraction header

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: PlatformIO envs for Plus2 + S3

**Files:**
- Modify: `platformio.ini`
- Possibly create: `boards/m5stick-s3.json` (only if no suitable stock S3 board exists)

**Interfaces:**
- Produces: build envs `m5stick-c-plus2` and `m5stick-s3` that compile the current source under `SG_BOARD_PLUS2` / `SG_BOARD_S3`.

- [ ] **Step 1: Verify M5Unified version + S3 board id (current-docs research)**

Use Context7 (`/m5stack/m5unified`) and the PlatformIO registry / `pio boards m5stick` to determine:
- the minimum **M5Unified** version that includes StickS3 board support (newer than `0.2.14`);
- the exact `m5::board_t` constants for Plus2 and S3 (expected `board_M5StickCPlus2`, `board_M5StickS3`) — record them for Task 5;
- whether a stock PlatformIO S3 board (e.g. an `esp32-s3` devkit def with 8 MB flash / octal PSRAM) is acceptable, or a vendored `boards/m5stick-s3.json` is needed.
  Run: `/Users/moefayed/.local/bin/pio boards 2>/dev/null | grep -i s3 | head -40`

- [ ] **Step 2: Add the Plus2 env**

Append to `platformio.ini`:
```ini
[env:m5stick-c-plus2]
platform = espressif32@^6.5.0
board = m5stick-c
framework = arduino
upload_speed = 1500000
monitor_speed = 115200
build_flags =
    -D CORE_DEBUG_LEVEL=1
    -D SG_BOARD_PLUS2
    -D BOARD_HAS_PSRAM
    -std=gnu++17
    -Wall -Wswitch-enum
    -ffile-prefix-map=${platformio.core_dir}=pio
    -fmacro-prefix-map=${platformio.core_dir}=pio
lib_deps =
    m5stack/M5Unified@^0.2.14   ; bump to the StickS3-capable version found in Step 1
```

- [ ] **Step 3: Add the S3 env**

Append to `platformio.ini` (board id / PSRAM flags filled from Step 1):
```ini
[env:m5stick-s3]
platform = espressif32@^6.7.0   ; S3 needs a platform rev with S3-PICO support; pin from Step 1
board = <s3-board-id-from-step-1>
framework = arduino
upload_speed = 1500000
monitor_speed = 115200
build_flags =
    -D CORE_DEBUG_LEVEL=1
    -D SG_BOARD_S3
    -D BOARD_HAS_PSRAM
    -std=gnu++17
    -Wall -Wswitch-enum
    -ffile-prefix-map=${platformio.core_dir}=pio
    -fmacro-prefix-map=${platformio.core_dir}=pio
lib_deps =
    m5stack/M5Unified@^0.2.14   ; same StickS3-capable version as Plus2
```

- [ ] **Step 4: Bump the Plus env's M5Unified to match**

Set the `m5stack/M5Unified` version in `[env:m5stick-c-plus]` to the same StickS3-capable version, so all three envs share one library version.

- [ ] **Step 5: Build all three firmware envs**

Run:
```bash
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus2
/Users/moefayed/.local/bin/pio run -e m5stick-s3
```
Expected: all three SUCCESS. (At this point Plus2/S3 still run Plus-specific hardware behavior — that is corrected in Tasks 3–5. This step only proves the build matrix and the new envs are valid.)

- [ ] **Step 6: Confirm native tests still pass**

Run: `/Users/moefayed/.local/bin/pio test -e native`
Expected: all suites PASS.

- [ ] **Step 7: Commit**

```bash
git add platformio.ini boards/ 2>/dev/null; git add platformio.ini
git commit -m "build: add m5stick-c-plus2 and m5stick-s3 build envs

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: `feedback.cpp` — board-driven LED pin

**Files:**
- Modify: `src/feedback.cpp`

**Interfaces:**
- Consumes: `board::led_pin()` from Task 1.

- [ ] **Step 1: Replace the hardcoded LED pin**

In `src/feedback.cpp`, add `#include "board.h"` after `#include "feedback.h"`, then replace:
```cpp
    constexpr int      LED_PIN        = 10;     // M5StickC Plus red LED GPIO (active LOW)
```
with:
```cpp
    constexpr int      LED_PIN        = board::led_pin();  // red LED GPIO by board (active LOW)
```
Leave everything else (BEEP_HZ, BEEP_MS, BUZZER_VOLUME, all functions) unchanged — `M5.Speaker.tone` already works on the S3 codec speaker.

- [ ] **Step 2: Build all three envs**

Run:
```bash
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus2
/Users/moefayed/.local/bin/pio run -e m5stick-s3
```
Expected: all SUCCESS. On Plus, `board::led_pin()` is `constexpr 10` — identical to the literal it replaced.

- [ ] **Step 3: Confirm native tests still pass**

Run: `/Users/moefayed/.local/bin/pio test -e native`
Expected: PASS (feedback is stubbed under UNIT_TEST; this guards against an accidental include break).

- [ ] **Step 4: Commit**

```bash
git add src/feedback.cpp
git commit -m "feat(feedback): drive status-LED pin from board variant

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: `imu.cpp` — accept the BMI270 on the S3

**Files:**
- Modify: `src/imu.cpp:23-30` (the WHO_AM_I gate)

**Interfaces:**
- Consumes: `board::variant()`, `board::Variant` from Task 1.

- [ ] **Step 1: Replace the IMU-type gate**

In `src/imu.cpp`, add `#include "board.h"` inside the `#ifndef UNIT_TEST` block (after `#include <M5Unified.h>`), then replace:
```cpp
    // WHO_AM_I mismatch indicator: M5Unified's auto-detected type should be mpu6886
    // on an M5StickC Plus. Any other value means either a different board was
    // detected, or the documented AXP192/MPU6886 I²C conflict kept the IMU chip
    // from answering the type query.
    if (M5.Imu.getType() != m5::imu_mpu6886) {
        return FaultCode::E03_WHO_AM_I_MISMATCH;
    }
```
with:
```cpp
    // The expected auto-detected IMU is board-specific: MPU6886 on Plus/Plus2,
    // BMI270 on the S3. A mismatch means a different board was detected, the wrong
    // firmware was flashed, or (Plus only) the AXP192/MPU6886 I²C conflict kept the
    // IMU from answering the type query.
    m5::imu_t got = M5.Imu.getType();
    bool type_ok = (board::variant() == board::Variant::S3)
                       ? (got == m5::imu_bmi270)
                       : (got == m5::imu_mpu6886);
    if (!type_ok) {
        return FaultCode::E03_WHO_AM_I_MISMATCH;
    }
```

- [ ] **Step 2: Build all three envs**

Run:
```bash
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus2
/Users/moefayed/.local/bin/pio run -e m5stick-s3
```
Expected: all SUCCESS. On Plus/Plus2 the gate is `got == m5::imu_mpu6886` — identical behavior to before.

- [ ] **Step 3: Confirm native tests still pass**

Run: `/Users/moefayed/.local/bin/pio test -e native`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add src/imu.cpp
git commit -m "feat(imu): accept BMI270 on S3, keep MPU6886 on Plus/Plus2

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 5: `power.cpp` + `main.cpp` — per-board sleep/wake & wrong-firmware guard

**Files:**
- Modify: `src/power.cpp` (the `#ifndef UNIT_TEST` `enter_deep_sleep()`)
- Modify: `src/main.cpp` (`setup()`)

**Interfaces:**
- Consumes: `board::variant()`, `board::has_axp192()` from Task 1.

**Background:** M5Unified's `deepSleep()` arms its own `_wakeupPin` on the Plus2 (and manages the S3's M5PM1 power button), but NOT on the AXP192 Plus. So the Plus must keep arming EXT0 on GPIO35 itself; Plus2/S3 just call `deepSleep()`.

- [ ] **Step 1: Branch `enter_deep_sleep()` by board**

In `src/power.cpp`, add `#include "board.h"` inside the `#ifndef UNIT_TEST` region, then replace the body of the non-test `enter_deep_sleep()` (the AXP register writes + `esp_sleep_enable_ext0_wakeup` + `M5.Power.deepSleep()`) with:
```cpp
[[noreturn]] void enter_deep_sleep() {
    M5.Display.setBrightness(0);
    switch (board::variant()) {
        case board::Variant::PLUS:
            // M5Unified's deepSleep() does NOT arm a wake source on the AXP192
            // Plus, so do it ourselves: EXT0 on GPIO35 (the AXP192 IRQ line,
            // active-low) — a short power-key press asserts it and wakes the chip.
            // Clear all pending AXP IRQ flags first (write-1-to-clear), else the
            // line may already be low and the device would wake instantly.
            M5.Power.Axp192.writeRegister8(0x44, 0xFF);
            M5.Power.Axp192.writeRegister8(0x45, 0xFF);
            M5.Power.Axp192.writeRegister8(0x46, 0xFF);
            M5.Power.Axp192.writeRegister8(0x47, 0xFF);
            M5.Power.Axp192.writeRegister8(0x4D, 0xFF);
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
            break;
        case board::Variant::PLUS2:
        case board::Variant::S3:
            // M5Unified arms the board's own wake source inside deepSleep() for
            // these (Plus2 HOLD/power button; S3 M5PM1 power button). Nothing to
            // arm by hand, and there is no AXP192 IRQ to clear.
            break;
    }
    M5.Power.deepSleep();  // Display.sleep() + esp_deep_sleep_start(); wake stays armed
    while (true) {}        // deepSleep() does not return
}
```
(The Plus path above is the verbatim original logic, now inside the `PLUS` case.)

- [ ] **Step 2: Guard the AXP key-state consume in `setup()`**

In `src/main.cpp` `setup()`, add `#include "board.h"` with the other includes, then wrap the existing key-state consume:
```cpp
    // Consume any pending AXP192 power-key press (reads + clears the IRQ flag).
    // The press that woke us from deep sleep would otherwise surface as a
    // BtnPWR click on the first M5.update() and immediately put us back to sleep.
    M5.Power.getKeyState();
```
with:
```cpp
    // Consume any pending power-key press that woke us, so it doesn't surface as a
    // BtnPWR click on the first M5.update() and bounce us straight back to sleep.
    // getKeyState() is AXP192/AXP2101-only; the Plus2/S3 power buttons are handled
    // through M5.BtnPWR, so only the AXP192 Plus needs this.
    if (board::has_axp192()) {
        M5.Power.getKeyState();
    }
```

- [ ] **Step 3: Add the wrong-firmware runtime guard in `setup()`**

In `src/main.cpp` `setup()`, immediately after `M5.begin(cfg);` and the `setCpuFrequencyMhz(80);` line, insert:
```cpp
    // Safety: refuse to run if this binary was flashed onto a different board than
    // it was built for (e.g. the S3 build onto a Plus2). Each board has a distinct
    // sleep/wake + IMU path, so running the wrong one would misbehave silently.
    {
        m5::board_t expected;
        switch (board::variant()) {
            case board::Variant::PLUS:  expected = m5::board_t::board_M5StickCPlus;  break;
            case board::Variant::PLUS2: expected = m5::board_t::board_M5StickCPlus2; break;
            case board::Variant::S3:    expected = m5::board_t::board_M5StickS3;     break;
        }
        if (M5.getBoard() != expected) {
            M5.Display.setRotation(1);
            M5.Display.fillScreen(TFT_RED);
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(6, 10);
            M5.Display.print("WRONG FIRMWARE");
            M5.Display.setTextSize(1);
            M5.Display.setCursor(6, 40);
            M5.Display.print("for this device.");
            M5.Display.setCursor(6, 56);
            M5.Display.print("Flash the build that");
            M5.Display.setCursor(6, 68);
            M5.Display.print("matches your stick.");
            while (true) { delay(1000); }
        }
    }
```
**Verify the three `m5::board_t` constant names against the installed M5Unified header** (`~/.platformio/.../M5Unified/src/M5Unified.hpp` or via Context7). If a name differs (e.g. `board_M5StickCPlus2` spelling), use the header's exact spelling.

- [ ] **Step 4: Build all three envs**

Run:
```bash
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus2
/Users/moefayed/.local/bin/pio run -e m5stick-s3
```
Expected: all SUCCESS, no `-Wswitch-enum` warnings (both new switches cover all three `Variant` cases).

- [ ] **Step 5: Confirm native tests still pass**

Run: `/Users/moefayed/.local/bin/pio test -e native`
Expected: PASS (power's pure-logic functions and the `enter_deep_sleep()` UNIT_TEST stub are unchanged).

- [ ] **Step 6: Commit**

```bash
git add src/power.cpp src/main.cpp
git commit -m "feat(power): per-board deep-sleep/wake and wrong-firmware guard

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 6: Browser flasher — per-board manifests, binaries, device picker

**Files:**
- Create: `docs/manifest-plus.json`, `docs/manifest-plus2.json`, `docs/manifest-s3.json`
- Modify: `docs/index.html`
- Modify: `docs/manifest.json` (keep as the Plus manifest for backward-compatible links, or repoint)
- Add: `docs/firmware/sharpening-guide-plus2-v0.2.0.bin`, `docs/firmware/sharpening-guide-s3-v0.2.0.bin`, `docs/firmware/sharpening-guide-plus-v0.2.0.bin`

**Interfaces:** none (web).

- [ ] **Step 1: Produce merged, flashable binaries for each board**

ESP Web Tools flashes a single image at offset 0. Build, then merge each env's parts with esptool (mirrors how v0.1.0 was produced). For each env, locate the build dir under `.pio/build/<env>/` and merge:
```bash
PIO=/Users/moefayed/.local/bin/pio
$PIO run -e m5stick-c-plus
$PIO run -e m5stick-c-plus2
$PIO run -e m5stick-s3
# Merge (flash args come from each build's flasher_args.json; chip differs for S3):
python3 -m esptool --chip esp32   merge_bin -o docs/firmware/sharpening-guide-plus-v0.2.0.bin   @.pio/build/m5stick-c-plus/flasher_args.json
python3 -m esptool --chip esp32   merge_bin -o docs/firmware/sharpening-guide-plus2-v0.2.0.bin  @.pio/build/m5stick-c-plus2/flasher_args.json
python3 -m esptool --chip esp32s3 merge_bin -o docs/firmware/sharpening-guide-s3-v0.2.0.bin     @.pio/build/m5stick-s3/flasher_args.json
```
If `@flasher_args.json` form is unavailable, merge with explicit offsets from that file (bootloader/partitions/boot_app0/firmware). Keep the existing `docs/firmware/sharpening-guide-v0.1.0.bin` in place untouched.

- [ ] **Step 2: Write per-board manifests**

`docs/manifest-plus.json` (chipFamily ESP32):
```json
{
  "name": "Digital Sharpening Guide (M5StickC Plus)",
  "version": "0.2.0",
  "funding_url": "https://github.com/miamimoe/digital-sharpening-guide",
  "new_install_prompt_erase": true,
  "builds": [
    { "chipFamily": "ESP32", "parts": [ { "path": "firmware/sharpening-guide-plus-v0.2.0.bin", "offset": 0 } ] }
  ]
}
```
`docs/manifest-plus2.json` — identical but name `(M5StickC Plus2)` and `path` `firmware/sharpening-guide-plus2-v0.2.0.bin`.
`docs/manifest-s3.json` — name `(M5StickS3)`, **`"chipFamily": "ESP32-S3"`**, `path` `firmware/sharpening-guide-s3-v0.2.0.bin`.

- [ ] **Step 3: Add a device picker to `index.html`**

Replace the single `<esp-web-install-button manifest="manifest.json">` block with a 3-way selector that swaps the manifest. Update the intro copy from "M5StickC Plus" to "M5StickC Plus, Plus2, or M5StickS3". Minimal pattern:
```html
<div class="panel">
  <h2>1 · Pick your device</h2>
  <select id="device" style="width:100%;padding:10px;border-radius:8px;background:#0a0c0f;color:var(--text);border:1px solid var(--line);">
    <option value="manifest-plus.json">M5StickC Plus</option>
    <option value="manifest-plus2.json">M5StickC Plus2</option>
    <option value="manifest-s3.json">M5StickS3</option>
  </select>
</div>
<div class="cta">
  <esp-web-install-button id="installer" manifest="manifest-plus.json">
    <button slot="activate" class="btn-fallback">⚡ Flash it now</button>
    <span slot="unsupported" class="unsupported">Your browser can't flash over USB. Open this page in desktop Chrome, Edge, or Opera.</span>
    <span slot="not-allowed" class="unsupported">Flashing needs a secure (https) connection.</span>
  </esp-web-install-button>
</div>
<script>
  const sel = document.getElementById('device');
  const inst = document.getElementById('installer');
  sel.addEventListener('change', () => { inst.setAttribute('manifest', sel.value); });
</script>
```

- [ ] **Step 4: Verify the flasher page locally**

Open `docs/index.html` and confirm: the selector renders, switching options updates the install button's `manifest` attribute (DOM inspector), and all three manifest files + their referenced `.bin` files exist on disk.
Run: `ls -la docs/firmware/ docs/manifest-*.json`
Expected: three v0.2.0 bins + three manifests present.

- [ ] **Step 5: Commit**

```bash
git add docs/
git commit -m "feat(flasher): add Plus2 + S3 firmware and device picker

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 7: Docs — README, bring-up, CLAUDE.md (honest test status + tester call)

**Files:**
- Modify: `README.md`
- Modify: `CLAUDE.md`
- Modify: `docs/superpowers/bringup/2026-04-23-hardware-bringup.md` (add Plus2/S3 bring-up checklist)

**Interfaces:** none.

- [ ] **Step 1: Update README hardware/flash sections**

In `README.md`: change device references from "M5StickC Plus" to list all three supported boards; in the flashing section note the device picker. Add a short **"Board support status"** note, verbatim intent:
> Supported boards: **M5StickC Plus** (validated on real hardware), **M5StickC Plus2** and **M5StickS3** (compile-verified and code-reviewed against the M5Stack datasheets, but **not yet confirmed on real hardware** — if you have one, please flash it and open an issue with results). All three share one codebase.

- [ ] **Step 2: Update CLAUDE.md**

In `CLAUDE.md`: update the "What this is" / hardware-status lines to state the firmware targets Plus, Plus2, and S3 from one board-guarded codebase (`src/board.h`); note the build envs `m5stick-c-plus`, `m5stick-c-plus2`, `m5stick-s3`; add to "Known risks to validate on real hardware" that Plus2/S3 are unvalidated on-device.

- [ ] **Step 3: Add a Plus2/S3 bring-up checklist**

In the bring-up doc, add a section listing the first-boot checks a community tester should run on Plus2/S3: device boots to SET TARGET (no WRONG FIRMWARE screen), IMU not faulted, angle colors respond to tilt, buzzer/speaker audible, power-key sleep then wake resumes the session, magnet grip.

- [ ] **Step 4: Final full verification**

Run:
```bash
/Users/moefayed/.local/bin/pio test -e native
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus
/Users/moefayed/.local/bin/pio run -e m5stick-c-plus2
/Users/moefayed/.local/bin/pio run -e m5stick-s3
```
Expected: native PASS; all three builds SUCCESS.

- [ ] **Step 5: Commit**

```bash
git add README.md CLAUDE.md docs/superpowers/bringup/
git commit -m "docs: document Plus2 + S3 support and on-device test status

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Self-Review (completed)

- **Spec coverage:** board-guard mechanism → Task 1; build matrix + M5Unified bump + S3 board id → Task 2; LED pin → Task 3; BMI270 gate → Task 4; per-board sleep/wake + `getKeyState` guard + wrong-firmware assert → Task 5; flasher per-board manifests/picker/binaries → Task 6; honest release labeling + README/CLAUDE/bring-up → Task 7. Plus byte-identical guarantee enforced by verbatim Plus branches + native regression gate in every task.
- **Placeholder scan:** the only deferred values (M5Unified version, S3 board id, exact `m5::board_t` names) are explicit Task-2/Task-5 verification steps with the lookup commands given — not vague TODOs.
- **Type consistency:** `board::variant()`, `board::Variant::{PLUS,PLUS2,S3}`, `board::led_pin()`, `board::has_axp192()` defined in Task 1 and used with those exact names in Tasks 3–5. `m5::imu_bmi270`/`imu_mpu6886` and `M5.getBoard()`/`m5::board_t` confirmed via Context7.
