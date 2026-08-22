#pragma once
#include "types.h"

namespace settings {
    void begin();

    Tolerance  load_tolerance();
    void       save_tolerance(Tolerance t);

    bool       load_buzzer();
    void       save_buzzer(bool on);

    // Steady mode: accel-reference smoothing + display/colour hysteresis.
    // Defaults ON — it is the better behaviour; the setting exists so a tester can
    // A/B it against the old response on one device without reflashing.
    bool       load_steady();
    void       save_steady(bool on);

    Vec3       load_gyro_bias();
    void       save_gyro_bias(Vec3 bias_dps);
}
