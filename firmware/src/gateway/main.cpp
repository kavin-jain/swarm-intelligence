// Gateway firmware: the ESP32 on the laptop's USB port. Turns camera frames from
// satellite.py into the shared snapshot, broadcasts it to every robot, and relays the
// robots' heartbeats back to the laptop. It never tells a robot what to do.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "world.h"

#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL 1   // must match robot_config.h
#endif

using namespace swarm;

static World world;
static Deframer deframer;
static QueueHandle_t hb_q;
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t*, const uint8_t* d, int n) {
#else
static void on_recv(const uint8_t*, const uint8_t* d, int n) {
#endif
    Heartbeat h;
    if (n > 0 && d[0] == MSG_HEARTBEAT && decode(d, n, h)) xQueueSend(hb_q, &h, 0);
}

static void to_laptop(const uint8_t* payload, int n) {
    uint8_t out[MAX_PAYLOAD + 8];
    Serial.write(out, frame(payload, n, out));
}

void setup() {
    Serial.setRxBufferSize(2048);
    Serial.begin(230400);
    hb_q = xQueueCreate(32, sizeof(Heartbeat));
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (esp_now_init() != ESP_OK) { for (;;) delay(1000); }
    esp_now_register_recv_cb(on_recv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = WIFI_CHANNEL;
    esp_now_add_peer(&peer);
}

void loop() {
    uint32_t now = millis();
    while (Serial.available()) {
        int n = deframer.feed((uint8_t)Serial.read());
        if (!n) continue;
        const uint8_t* p = deframer.buf;
        if (p[0] == MSG_VISION) {
            Vision v;
            if (!decode(p, n, v)) continue;
            world.on_vision(v, now);
            Snapshot s = world.snapshot(now);
            uint8_t buf[MAX_PAYLOAD];
            int len = encode(s, buf, sizeof buf);
            if (len > 0) { esp_now_send(BCAST, buf, len); to_laptop(buf, len); }
        } else if (p[0] == MSG_ESTOP && n >= 2) {
            for (int i = 0; i < 5; i++) { esp_now_send(BCAST, p, 2); delay(5); }   // repeat: it must get through
        }
    }
    Heartbeat h;
    while (xQueueReceive(hb_q, &h, 0) == pdTRUE) {
        world.on_heartbeat(h, now);
        uint8_t buf[16];
        to_laptop(buf, encode(h, buf, sizeof buf));
    }
    // No camera frames = no snapshots = every robot stops itself within 0.5 s.
}
