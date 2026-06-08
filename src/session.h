#pragma once
#include "types.h"
#include <cstdint>

// Bump SESSION_VERSION whenever the field layout below changes, so a firmware
// update that leaves a stale struct in RTC RAM is detected and discarded instead
// of read as garbage. magic/version MUST stay the first two fields.
constexpr uint32_t SESSION_MAGIC   = 0x53475A31;  // "SGZ1"
constexpr uint16_t SESSION_VERSION = 1;

struct SessionState {
    uint32_t  magic              = SESSION_MAGIC;
    uint16_t  version            = SESSION_VERSION;
    bool      active             = false;
    float     target_deg         = 0.0f;
    Tolerance tolerance          = Tolerance::NORMAL;
    Vec3      g_zero_A           = {0.0f, 0.0f, 0.0f};
    Vec3      g_zero_B           = {0.0f, 0.0f, 0.0f};
    uint32_t  strokes_A          = 0;
    uint32_t  strokes_B          = 0;
    Side      current_side       = Side::A;
    uint32_t  session_started_ms = 0;
};

namespace session {
    void          begin();
    SessionState& state();
    bool          has_session();
    void          mark_active(const SessionState& s);
    void          clear();
}
