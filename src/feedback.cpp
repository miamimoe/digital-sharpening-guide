#include "feedback.h"
#include "board.h"

#ifndef UNIT_TEST
#include <M5Unified.h>

namespace {
    constexpr int      LED_PIN        = board::led_pin();  // red LED GPIO by board (active LOW)
    constexpr float    BEEP_HZ        = 2000.0f;
    constexpr uint32_t BEEP_MS        = 150;    // long enough to clearly hear on the GPIO2 buzzer
    constexpr uint8_t  BUZZER_VOLUME  = 255;    // max — the passive buzzer can't be overdriven
}

namespace feedback {

void begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // HIGH = LED off
    // M5.begin(cfg.internal_spk=true) already started the buzzer; just make sure
    // it's audible. The default volume is low and easy to miss on this buzzer.
    M5.Speaker.setVolume(BUZZER_VOLUME);
}

void set_color(ColorState c) {
    digitalWrite(LED_PIN, (c == ColorState::RED) ? LOW : HIGH);
}

void fault_led() {
    digitalWrite(LED_PIN, LOW); // solid on
}

void beep_out_of_tolerance() {
    if (!M5.Speaker.isEnabled()) return;
    M5.Speaker.tone(BEEP_HZ, BEEP_MS);
}

void beep_confirm() {
    if (!M5.Speaker.isEnabled()) return;
    // Two quick ascending notes — distinct from the single out-of-tolerance beep,
    // and an immediate "the buzzer works" cue when the user turns it on. The
    // second tone passes stop_current_sound=false so it queues after the first
    // instead of cutting it off (default tone() stops the playing sound).
    M5.Speaker.tone(1500.0f, 90);
    M5.Speaker.tone(2200.0f, 120, -1, false);
}

} // namespace feedback

#else
// Native stubs
namespace feedback {
    void begin() {}
    void set_color(ColorState) {}
    void fault_led() {}
    void beep_out_of_tolerance() {}
    void beep_confirm() {}
}
#endif
