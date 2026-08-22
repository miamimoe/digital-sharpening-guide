#include "settings.h"

#ifdef UNIT_TEST
namespace {
    Tolerance g_tol    = Tolerance::NORMAL;
    bool      g_buzz   = false;
    bool      g_steady = true;
    Vec3      g_bias   = {0.0f, 0.0f, 0.0f};
    SessionRecord g_hist[kSessionHistoryMax];
    int           g_hist_count = 0;
}
namespace settings {
    void begin() {}
    Tolerance load_tolerance()            { return g_tol; }
    void      save_tolerance(Tolerance t) { g_tol = t; }
    bool      load_buzzer()               { return g_buzz; }
    void      save_buzzer(bool on)        { g_buzz = on; }
    bool      load_steady()               { return g_steady; }
    void      save_steady(bool on)        { g_steady = on; }
    int  load_session_history(SessionRecord* out, int max) {
        int n = (g_hist_count < max) ? g_hist_count : max;
        for (int i = 0; i < n; i++) out[i] = g_hist[i];
        return n;
    }
    void push_session_record(const SessionRecord& r) {
        for (int i = kSessionHistoryMax - 1; i > 0; i--) g_hist[i] = g_hist[i-1];
        g_hist[0] = r;
        if (g_hist_count < kSessionHistoryMax) ++g_hist_count;
    }
    void clear_session_history() { g_hist_count = 0; }
    Vec3      load_gyro_bias()            { return g_bias; }
    void      save_gyro_bias(Vec3 b)      { g_bias = b; }
}
#else
#include <Preferences.h>
namespace {
    Preferences prefs;
    constexpr const char* NS = "sharpguide";
}
namespace settings {
    void begin() {
        prefs.begin(NS, false);
    }
    Tolerance load_tolerance() {
        return static_cast<Tolerance>(prefs.getUChar("tol", (uint8_t)Tolerance::NORMAL));
    }
    void save_tolerance(Tolerance t) {
        prefs.putUChar("tol", (uint8_t)t);
    }
    bool load_buzzer()               { return prefs.getBool("buzz", false); }
    void save_buzzer(bool on)        { prefs.putBool("buzz", on); }
    bool load_steady()               { return prefs.getBool("steady", true); }
    void save_steady(bool on)        { prefs.putBool("steady", on); }
    // Gyro bias is stored as a single NVS blob (one entry instead of three float
    // keys). Old bx/by/bz keys may remain orphaned in NVS; that is acceptable.
    Vec3 load_gyro_bias() {
        Vec3 v{0.0f, 0.0f, 0.0f};
        if (prefs.getBytesLength("bias") == sizeof(Vec3)) {
            prefs.getBytes("bias", &v, sizeof(Vec3));
        }
        return v;
    }
    void save_gyro_bias(Vec3 b) {
        prefs.putBytes("bias", &b, sizeof(Vec3));
    }

    // Session history lives as one fixed-size blob plus a count, so a push is a
    // single NVS write. Written once at session end — never per stroke, which
    // would burn through the flash's write endurance for no benefit.
    namespace {
        struct HistBlob {
            uint8_t       count = 0;
            SessionRecord rec[kSessionHistoryMax];
        };
        bool read_hist(HistBlob& h) {
            if (prefs.getBytesLength("hist") != sizeof(HistBlob)) return false;
            prefs.getBytes("hist", &h, sizeof(HistBlob));
            if (h.count > kSessionHistoryMax) return false;   // corrupt — ignore
            return true;
        }
    }

    int load_session_history(SessionRecord* out, int max) {
        HistBlob h;
        if (!read_hist(h)) return 0;
        int n = (h.count < max) ? h.count : max;
        for (int i = 0; i < n; i++) out[i] = h.rec[i];
        return n;
    }

    void push_session_record(const SessionRecord& r) {
        HistBlob h;
        if (!read_hist(h)) h = HistBlob{};
        for (int i = kSessionHistoryMax - 1; i > 0; i--) h.rec[i] = h.rec[i-1];
        h.rec[0] = r;
        if (h.count < kSessionHistoryMax) ++h.count;
        prefs.putBytes("hist", &h, sizeof(HistBlob));
    }

    void clear_session_history() {
        prefs.remove("hist");
    }
}
#endif
