// Robot firmware: radio + motors around core/brain.h. All decisions happen in the brain.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "brain.h"
#include "robot_config.h"

#ifndef ROBOT_ID
#error "Build with -DROBOT_ID=n (see platformio.ini); it must match the robot's ArUco marker."
#endif

using namespace swarm;

static Tuning tuning() {
    Tuning t;
    t.vmax = CAL_VMAX; t.wheel_base = CAL_WHEEL_BASE; t.robot_radius = CAL_ROBOT_RADIUS;
    t.carry = GRIPPER_PIN >= 0;   // no gripper fitted: push loads instead of carrying them
    return t;
}
static Brain brain(ROBOT_ID, tuning());   // static: the planner grid is ~20 KB, too big for the stack

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static QueueHandle_t snap_q, hb_q;
static volatile int8_t estop_cmd = -1;    // set from the radio callback, applied in loop()

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t*, const uint8_t* d, int n) {
#else
static void on_recv(const uint8_t*, const uint8_t* d, int n) {
#endif
    // Runs in the WiFi task: decode and hand over, never touch the brain here.
    if (n < 1) return;
    if (d[0] == MSG_SNAPSHOT) { Snapshot s; if (decode(d, n, s)) xQueueOverwrite(snap_q, &s); }
    else if (d[0] == MSG_HEARTBEAT) { Heartbeat h; if (decode(d, n, h)) xQueueSend(hb_q, &h, 0); }
    else if (d[0] == MSG_ESTOP && n >= 2) estop_cmd = d[1] ? 1 : 0;
}

// ---- motors --------------------------------------------------------------------------
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
static void motor(float cmd, int en, int ch, int in1, int in2, bool invert) {
    if (invert) cmd = -cmd;
    if (fabsf(cmd) < 0.02f) { digitalWrite(in1, LOW); digitalWrite(in2, LOW); pwm_write(en, ch, 0); return; }
    digitalWrite(in1, cmd > 0 ? HIGH : LOW);
    digitalWrite(in2, cmd > 0 ? LOW : HIGH);
    // Map |cmd| onto [PWM_MIN, 255] so small commands still overcome the gearbox's stiction.
    pwm_write(en, ch, PWM_MIN + (int)(fminf(fabsf(cmd), 1.0f) * (255 - PWM_MIN)));
}
static void drive(float l, float r) {
    motor(l, PIN_L_EN, 0, PIN_L_IN1, PIN_L_IN2, INVERT_L);
    motor(r, PIN_R_EN, 1, PIN_R_IN1, PIN_R_IN2, INVERT_R);
}

// Gripper: peak-and-hold PWM on LEDC channel 2 (20 kHz: above hearing, fine for a MOSFET).
static void gripper(bool on, uint32_t now) {
    if (GRIPPER_PIN < 0) return;
    static bool was = false; static uint32_t since = 0;
    if (on && !was) since = now;
    was = on;
    pwm_write(GRIPPER_PIN, 2, !on ? 0 : now - since < GRIP_PEAK_MS ? 255 : GRIP_HOLD_DUTY);
}

static uint16_t battery_mv() {
    if (PIN_BATT < 0) return 0;
    return (uint16_t)(analogReadMilliVolts(PIN_BATT) * BATT_DIVIDER);
}

static void blink(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) { digitalWrite(PIN_LED, HIGH); delay(on_ms); digitalWrite(PIN_LED, LOW); delay(off_ms); }
}

static void radio_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); for (;;) blink(1, 50, 50); }
    esp_now_register_recv_cb(on_recv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

static void send_heartbeat(uint32_t now) {
    uint8_t buf[16];
    int n = encode(brain.heartbeat(now, battery_mv()), buf, sizeof buf);
    esp_now_send(BCAST, buf, n);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);
    for (int p : {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    pwm_setup(PIN_L_EN, 0); pwm_setup(PIN_R_EN, 1);
    drive(0, 0);
    if (GRIPPER_PIN >= 0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(GRIPPER_PIN, 20000, 8);
#else
        ledcSetup(2, 20000, 8); ledcAttachPin(GRIPPER_PIN, 2);
#endif
        pwm_write(GRIPPER_PIN, 2, 0);
    }
    snap_q = xQueueCreate(1, sizeof(Snapshot));
    hb_q = xQueueCreate(16, sizeof(Heartbeat));
    radio_setup();
    Serial.printf("robot %d up, MAC %s, channel %d\n", ROBOT_ID, WiFi.macAddress().c_str(), WIFI_CHANNEL);

    // Discovery: announce ourselves for 2 s and count who answers, then blink the count.
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
        send_heartbeat(millis());
        Heartbeat h;
        while (xQueueReceive(hb_q, &h, 0) == pdTRUE) brain.on_heartbeat(h, millis());
        delay(100);
    }
    int n = brain.neighbors(millis());
    Serial.printf("discovery: %d other robot(s) nearby\n", n);
    delay(300);
    blink(n, 250, 250);
}

void loop() {
    static uint32_t last_ctrl = 0, last_hb = 0, last_log = 0;
    uint32_t now = millis();

    Snapshot s;
    if (xQueueReceive(snap_q, &s, 0) == pdTRUE) brain.on_snapshot(s, now);
    Heartbeat h;
    while (xQueueReceive(hb_q, &h, 0) == pdTRUE) brain.on_heartbeat(h, now);
    if (estop_cmd >= 0) { brain.estop = estop_cmd; estop_cmd = -1; }

    uint16_t mv = battery_mv();
    if (PIN_BATT >= 0 && mv && mv < BATT_CUTOFF_MV) brain.estop = true;   // park before the cells are damaged

    if (now - last_ctrl >= 20) {          // 50 Hz control
        last_ctrl = now;
        float l, r;
        brain.step(now, l, r);
        drive(l, r);
        gripper(brain.grip, now);
        // LED: solid = pushing/carrying, slow blink = working, off = idle, fast blink = stopped
        bool led = brain.state == ST_PUSH || brain.state == ST_CARRY || (brain.state == ST_STOPPED ? (now / 100) % 2 : brain.state != ST_IDLE && (now / 500) % 2);
        digitalWrite(PIN_LED, led);
    }
    if (now - last_hb >= 100) { last_hb = now; send_heartbeat(now); }   // 10 Hz
    if (now - last_log >= 1000) {
        last_log = now;
        Serial.printf("state %d task %d help %d neighbours %d batt %u mV\n", brain.state, brain.task == NONE ? -1 : brain.task,
                      brain.help == NONE ? -1 : brain.help, brain.neighbors(now), mv);
    }
}
