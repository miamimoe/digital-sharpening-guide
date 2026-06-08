#pragma once
#include <cstdint>

// Main cooperative-loop period. Single source of truth for the loop rate (50 Hz);
// modules that count ticks-as-duration (zero_cal, app countdowns) derive from it.
constexpr uint32_t kLoopTickMs = 20;

struct Vec3 {
    float x;
    float y;
    float z;
};

enum class State : uint8_t {
    BOOT,
    BIAS_CAL,
    SET_TARGET,
    SET_TOLERANCE,
    ZERO_CAL,                  // NEW
    ACTIVE,
    REZERO,                    // re-capture current side's zero from within ACTIVE
    SUMMARY,
    FAULT,
    RESUME_PROMPT,
    SLEEP
};

enum class ZeroCalSubstate : uint8_t {
    PROMPT_FLAT,     // "lay flat on stone"
    CAPTURE_FLAT,
    PROMPT_RAISE,    // "raise the spine to your angle" (reveals the edge axis)
    CAPTURE_RAISE,
    DONE
};

enum class Side : uint8_t { A, B };

enum class Tolerance : uint8_t { TIGHT, NORMAL, EASY };

enum class ColorState : uint8_t { GREEN, BLUE, RED };

enum class InputEvent : uint8_t {
    NONE,
    A_SHORT,
    A_LONG,
    B_SHORT,
    B_LONG
};

enum class PresetSelection : uint8_t {
    P12,
    P15,
    P17,
    P20,
    P22,
    CANCEL
};

enum class FaultCode : uint8_t {
    NONE = 0,
    E01_BEGIN_FAILED = 1,
    E02_SELF_TEST_FAILED = 2,
    E03_WHO_AM_I_MISMATCH = 3
};

inline float tolerance_degrees(Tolerance t) {
    // Human-achievable green-zone half-widths (a person, not a robot, holding a
    // knife with live feedback). ±1° was unrealistic; these are the realistic set.
    switch (t) {
        case Tolerance::TIGHT:  return 2.0f;
        case Tolerance::NORMAL: return 3.0f;
        case Tolerance::EASY:   return 5.0f;
    }
    __builtin_unreachable();
}

inline float preset_degrees(PresetSelection p) {
    switch (p) {
        case PresetSelection::P12:    return 12.0f;
        case PresetSelection::P15:    return 15.0f;
        case PresetSelection::P17:    return 17.0f;
        case PresetSelection::P20:    return 20.0f;
        case PresetSelection::P22:    return 22.0f;
        case PresetSelection::CANCEL: return 0.0f;
    }
    __builtin_unreachable();
}
