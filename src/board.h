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
