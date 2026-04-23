#include "settings.h"

#ifdef UNIT_TEST
namespace {
    Tolerance g_tol   = Tolerance::NORMAL;
    bool      g_buzz  = false;
    Vec3      g_bias  = {0.0f, 0.0f, 0.0f};
    bool      g_first = true;
}
namespace settings {
    void begin() {}
    Tolerance load_tolerance()            { return g_tol; }
    void      save_tolerance(Tolerance t) { g_tol = t; }
    bool      load_buzzer()               { return g_buzz; }
    void      save_buzzer(bool on)        { g_buzz = on; }
    Vec3      load_gyro_bias()            { return g_bias; }
    void      save_gyro_bias(Vec3 b)      { g_bias = b; }
    bool      is_first_boot()             { return g_first; }
    void      clear_first_boot()          { g_first = false; }
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
    Vec3 load_gyro_bias() {
        Vec3 v;
        v.x = prefs.getFloat("bx", 0.0f);
        v.y = prefs.getFloat("by", 0.0f);
        v.z = prefs.getFloat("bz", 0.0f);
        return v;
    }
    void save_gyro_bias(Vec3 b) {
        prefs.putFloat("bx", b.x);
        prefs.putFloat("by", b.y);
        prefs.putFloat("bz", b.z);
    }
    bool is_first_boot()       { return prefs.getBool("first", true); }
    void clear_first_boot()    { prefs.putBool("first", false); }
}
#endif
