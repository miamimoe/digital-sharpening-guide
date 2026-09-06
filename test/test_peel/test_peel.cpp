#include <unity.h>
#include "peel.h"

void setUp(void) {}
void tearDown(void) {}

static void drive(PeelFSM& p, uint32_t& t, uint32_t dt_ms, float a_mag, float gyro) {
    uint32_t end = t + dt_ms;
    while (t < end) {
        t += 10;
        p.update(t, a_mag, gyro);
    }
}

void test_stroke_like_motion_does_not_fire(void) {
    PeelFSM p;
    uint32_t t = 0;
    // 0.18 g horizontal on a 1 g vector is |a-1| ≈ 0.016 — a real stroke.
    for (int i = 0; i < 20; i++) {
        drive(p, t, 80, 1.02f, 25.0f);
        drive(p, t, 320, 1.00f, 0.0f);
    }
    TEST_ASSERT_FALSE(p.consume_settled());
}

void test_peel_then_settle_fires_once(void) {
    PeelFSM p;
    uint32_t t = 0;
    drive(p, t, 250, 1.60f, 0.0f);    // airborne
    drive(p, t, 400, 1.00f, 0.0f);    // settle
    TEST_ASSERT_TRUE(p.consume_settled());
    TEST_ASSERT_FALSE(p.consume_settled());
}

void test_short_twitch_does_not_fire(void) {
    PeelFSM p;
    uint32_t t = 0;
    drive(p, t, 80, 1.60f, 0.0f);     // < AIR_REQUIRED_MS
    drive(p, t, 400, 1.00f, 0.0f);
    TEST_ASSERT_FALSE(p.consume_settled());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_stroke_like_motion_does_not_fire);
    RUN_TEST(test_peel_then_settle_fires_once);
    RUN_TEST(test_short_twitch_does_not_fire);
    return UNITY_END();
}
