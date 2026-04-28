#pragma once
#include "types.h"
#include <cstdint>

struct SessionState {
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
