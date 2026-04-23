#include "feedback.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace {
    constexpr int      LED_PIN        = 10;     // M5StickC Plus red LED GPIO (active LOW)
    constexpr float    BEEP_HZ        = 2000.0f;
    constexpr uint32_t BEEP_MS        = 80;
}

namespace feedback {

void begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // HIGH = LED off
}

void set_color(ColorState c) {
    digitalWrite(LED_PIN, (c == ColorState::RED) ? LOW : HIGH);
}

void fault_led() {
    digitalWrite(LED_PIN, LOW); // solid on
}

void beep_out_of_tolerance() {
    M5.Speaker.tone(BEEP_HZ, BEEP_MS);
}

} // namespace feedback

#else
// Native stubs
namespace feedback {
    void begin() {}
    void set_color(ColorState) {}
    void fault_led() {}
    void beep_out_of_tolerance() {}
}
#endif
