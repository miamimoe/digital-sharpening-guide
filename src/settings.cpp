#include "settings.h"

#ifdef UNIT_TEST
namespace {
    Tolerance g_tol   = Tolerance::NORMAL;
    bool      g_buzz  = false;
    Vec3      g_bias  = {0.0f, 0.0f, 0.0f};
}
namespace settings {
    void begin() {}
    Tolerance load_tolerance()            { return g_tol; }
    void      save_tolerance(Tolerance t) { g_tol = t; }
    bool      load_buzzer()               { return g_buzz; }
    void      save_buzzer(bool on)        { g_buzz = on; }
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
}
#endif
