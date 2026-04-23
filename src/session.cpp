#include "session.h"

#ifdef UNIT_TEST
namespace {
    SessionState g_state;
}
namespace session {
    void          begin() {}
    SessionState& state() { return g_state; }
    bool          has_session() { return g_state.active; }
    void          mark_active(const SessionState& s) { g_state = s; g_state.active = true; }
    void          clear() { g_state = SessionState{}; g_state.active = false; }
}
#else
#include <Arduino.h>
RTC_DATA_ATTR static SessionState g_state;
RTC_DATA_ATTR static bool         g_state_valid = false;

namespace session {
    void begin() {
        if (!g_state_valid) {
            g_state = SessionState{};
            g_state_valid = true;
        }
    }
    SessionState& state() { return g_state; }
    bool has_session()    { return g_state.active; }
    void mark_active(const SessionState& s) {
        g_state = s;
        g_state.active = true;
    }
    void clear() {
        g_state = SessionState{};
        g_state.active = false;
    }
}
#endif
