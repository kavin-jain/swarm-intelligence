// Wiring check + calibration run. Flash it, put the robot on the arena floor under the
// camera, and (optionally) run `satellite.py --measure <id>` to read off vmax/wheel_base.
// Sequence, repeating: left fwd, left back, right fwd, right back, both fwd 2 s, spin 2 s.
#include <Arduino.h>
#include "robot_config.h"

static void pwm_setup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch; ledcAttach(pin, PWM_FREQ, 8);
#else
    ledcSetup(ch, PWM_FREQ, 8); ledcAttachPin(pin, ch);
#endif
}
static void pwm_write(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch; ledcWrite(pin, duty);
#else
    (void)pin; ledcWrite(ch, duty);
#endif
}
static void motor(int dir, int duty, int en, int ch, int in1, int in2, bool invert) {
    if (invert) dir = -dir;
    digitalWrite(in1, dir > 0); digitalWrite(in2, dir < 0);
    pwm_write(en, ch, dir ? duty : 0);
}
static void step(const char* what, int l, int r, int ms) {
    Serial.println(what);
    motor(l, 255, PIN_L_EN, 0, PIN_L_IN1, PIN_L_IN2, INVERT_L);
    motor(r, 255, PIN_R_EN, 1, PIN_R_IN1, PIN_R_IN2, INVERT_R);
    delay(ms);
    motor(0, 0, PIN_L_EN, 0, PIN_L_IN1, PIN_L_IN2, INVERT_L);
    motor(0, 0, PIN_R_EN, 1, PIN_R_IN1, PIN_R_IN2, INVERT_R);
    delay(800);
}

void setup() {
    Serial.begin(115200);
    for (int p : {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    pwm_setup(PIN_L_EN, 0); pwm_setup(PIN_R_EN, 1);
    delay(2000);
}

void loop() {
    step("LEFT wheel forward   (if it spins backwards: INVERT_L 1)", 1, 0, 1000);
    step("LEFT wheel backward", -1, 0, 1000);
    step("RIGHT wheel forward  (if it spins backwards: INVERT_R 1)", 0, 1, 1000);
    step("RIGHT wheel backward", 0, -1, 1000);
    step("BOTH forward 2 s     (robot should go straight; measure vmax)", 1, 1, 2000);
    step("SPIN left 2 s        (measure wheel base)", -1, 1, 2000);
    // Deadband: ramp the left wheel up and note the first duty where it moves -> PWM_MIN.
    Serial.println("RAMP left wheel 40..255: note the duty where it starts turning = PWM_MIN");
    for (int d = 40; d <= 255; d += 5) {
        Serial.printf("  duty %d\n", d);
        motor(1, d, PIN_L_EN, 0, PIN_L_IN1, PIN_L_IN2, INVERT_L);
        delay(150);
    }
    motor(0, 0, PIN_L_EN, 0, PIN_L_IN1, PIN_L_IN2, INVERT_L);
    delay(3000);
}
