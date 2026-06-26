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

// Red status LED GPIO (active LOW). Plus routes it to GPIO10, Plus2 to GPIO19.
// The StickS3 has no user red LED, and its GPIO19 is the USB D- line — driving
// it would clobber USB-CDC — so the S3 reports -1 (no status LED). Callers must
// check has_status_led() before using this. The full-screen LCD color remains
// the primary feedback on every board; the LED is only a secondary cue.
constexpr int led_pin() {
#if defined(SG_BOARD_PLUS)
    return 10;
#elif defined(SG_BOARD_PLUS2)
    return 19;
#else
    return -1;
#endif
}

// Whether this board has a usable discrete status LED (false on the S3).
constexpr bool has_status_led() { return led_pin() >= 0; }

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
