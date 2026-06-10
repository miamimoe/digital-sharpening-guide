#pragma once
#include "types.h"

namespace settings {
    void begin();

    Tolerance  load_tolerance();
    void       save_tolerance(Tolerance t);

    bool       load_buzzer();
    void       save_buzzer(bool on);

    Vec3       load_gyro_bias();
    void       save_gyro_bias(Vec3 bias_dps);
}
