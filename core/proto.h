// Wire protocol shared by robots, gateway, simulator and (mirrored) satellite/proto.py.
// Explicit little-endian byte packing, no packed structs: the layout is the same on
// ESP32, x86 and ARM, and the Python side can mirror it field by field.
#pragma once
#include <stdint.h>
#include <string.h>

namespace swarm {

constexpr uint8_t MAX_ROBOTS = 10;
constexpr uint8_t MAX_ZONES = 3;    // docks; a load of kind k goes to dock k % zones
constexpr uint8_t MAX_OBJECTS = 16;
constexpr uint8_t NONE = 0xFF;      // "no task / no object"
constexpr int MAX_PAYLOAD = 250;    // ESP-NOW v1 limit

enum MsgType : uint8_t { MSG_VISION = 1, MSG_SNAPSHOT = 2, MSG_HEARTBEAT = 3, MSG_ESTOP = 4 };

// Robot behaviour states, reported in heartbeats and echoed in snapshots.
enum State : uint8_t { ST_IDLE = 0, ST_GOTO = 1, ST_ALIGN = 2, ST_WAIT = 3, ST_PUSH = 4, ST_BACKOFF = 5, ST_STOPPED = 6,
                      // carry mode (gripper fitted): drive onto the load, grip it, carry it, set it down
                      ST_DOCK = 7, ST_GRIP = 8, ST_CARRY = 9, ST_PLACE = 10 };

// ---- satellite -> gateway: what the camera sees this frame -------------------
struct VisRobot { uint8_t id; int16_t x, y, th; };   // mm, mm, mrad (CCW from +x)
struct VisObject { uint8_t id; int16_t x, y; uint8_t kind, r; };   // mm; kind = colour class (picks the dock); r = footprint radius, mm/2
struct Zone { int16_t x, y; uint16_t r; };            // a dock (circle), mm
struct Vision {
    uint16_t seq;
    uint8_t nz; Zone z[MAX_ZONES];                  // docks
    uint16_t arena_w, arena_h;                       // arena is [0,w] x [0,h], mm
    uint8_t nr, no;
    VisRobot r[MAX_ROBOTS];
    VisObject o[MAX_OBJECTS];
};

// ---- gateway -> robots (ESP-NOW broadcast): the shared world model ----------
struct SnapRobot { uint8_t id; int16_t x, y, th; uint8_t alive, task, state; };
// status: OBJ_OPEN, OBJ_DELIVERED, or OBJ_STUCK (the swarm tried with every robot it has and gave up)
enum ObjStatus : uint8_t { OBJ_OPEN = 0, OBJ_DELIVERED = 1, OBJ_STUCK = 2 };
struct SnapObject { uint8_t id; int16_t x, y; uint8_t demand, status, kind; uint16_t r = 0; };   // demand+kind share a wire byte, status+r another; r in mm (0 = unknown)
struct Snapshot {
    uint16_t seq;
    uint8_t nz; Zone z[MAX_ZONES];
    uint16_t arena_w, arena_h;
    uint8_t nr, no;
    SnapRobot r[MAX_ROBOTS];
    SnapObject o[MAX_OBJECTS];
};

// ---- robot -> everyone (ESP-NOW broadcast) ----------------------------------
struct Heartbeat {
    uint8_t id;
    uint16_t seen_seq;    // last snapshot seq this robot acted on
    uint8_t state, task;
    uint8_t help;         // object id it is stalled on and needs help with, or NONE
    uint8_t neighbors;    // robots it has heard in the last second
    uint16_t batt_mv;
};

// ---- byte packing ------------------------------------------------------------
struct Writer {
    uint8_t* p; int n = 0, cap;
    Writer(uint8_t* buf, int c) : p(buf), cap(c) {}
    void u8(uint8_t v) { if (n < cap) p[n] = v; n++; }
    void u16(uint16_t v) { u8(v & 0xFF); u8(v >> 8); }
    void i16(int16_t v) { u16((uint16_t)v); }
    bool ok() const { return n <= cap; }
};
struct Reader {
    const uint8_t* p; int n = 0, len;
    Reader(const uint8_t* buf, int l) : p(buf), len(l) {}
    uint8_t u8() { return n < len ? p[n++] : (n++, 0); }
    uint16_t u16() { uint16_t lo = u8(); return lo | (uint16_t)(u8() << 8); }
    int16_t i16() { return (int16_t)u16(); }
    bool ok() const { return n <= len; }
};

inline int16_t clamp16(float v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f); }

inline void put_zones(Writer& w, uint8_t nz, const Zone* z) {
    w.u8(nz);
    for (int i = 0; i < nz; i++) { w.i16(z[i].x); w.i16(z[i].y); w.u16(z[i].r); }
}
inline bool get_zones(Reader& r, uint8_t& nz, Zone* z) {
    nz = r.u8();
    if (nz > MAX_ZONES) return false;
    for (int i = 0; i < nz; i++) { z[i].x = r.i16(); z[i].y = r.i16(); z[i].r = r.u16(); }
    return true;
}

inline int encode(const Vision& v, uint8_t* buf, int cap) {
    Writer w(buf, cap);
    w.u8(MSG_VISION); w.u16(v.seq); put_zones(w, v.nz, v.z);
    w.u16(v.arena_w); w.u16(v.arena_h);
    w.u8(v.nr); w.u8(v.no);
    for (int i = 0; i < v.nr; i++) { w.u8(v.r[i].id); w.i16(v.r[i].x); w.i16(v.r[i].y); w.i16(v.r[i].th); }
    for (int i = 0; i < v.no; i++) { w.u8(v.o[i].id); w.i16(v.o[i].x); w.i16(v.o[i].y); w.u8(v.o[i].kind); w.u8(v.o[i].r); }
    return w.ok() ? w.n : -1;
}
inline bool decode(const uint8_t* buf, int len, Vision& v) {
    Reader r(buf, len);
    if (r.u8() != MSG_VISION) return false;
    v.seq = r.u16();
    if (!get_zones(r, v.nz, v.z)) return false;
    v.arena_w = r.u16(); v.arena_h = r.u16();
    v.nr = r.u8(); v.no = r.u8();
    if (v.nr > MAX_ROBOTS || v.no > MAX_OBJECTS) return false;
    for (int i = 0; i < v.nr; i++) { v.r[i].id = r.u8(); v.r[i].x = r.i16(); v.r[i].y = r.i16(); v.r[i].th = r.i16(); }
    for (int i = 0; i < v.no; i++) { v.o[i].id = r.u8(); v.o[i].x = r.i16(); v.o[i].y = r.i16(); v.o[i].kind = r.u8(); v.o[i].r = r.u8(); }
    return r.ok() && r.n == len;
}

inline int encode(const Snapshot& s, uint8_t* buf, int cap) {
    Writer w(buf, cap);
    w.u8(MSG_SNAPSHOT); w.u16(s.seq); put_zones(w, s.nz, s.z);
    w.u16(s.arena_w); w.u16(s.arena_h);
    w.u8(s.nr); w.u8(s.no);
    for (int i = 0; i < s.nr; i++) {
        const SnapRobot& r = s.r[i];
        w.u8(r.id); w.i16(r.x); w.i16(r.y); w.i16(r.th); w.u8(r.alive); w.u8(r.task); w.u8(r.state);
    }
    for (int i = 0; i < s.no; i++) {
        const SnapObject& o = s.o[i];
        uint16_t r5 = (o.r + 2) / 5; if (r5 > 63) r5 = 63;   // 5 mm steps, up to 315 mm
        w.u8(o.id); w.i16(o.x); w.i16(o.y); w.u8((uint8_t)((o.kind << 4) | (o.demand & 0x0F))); w.u8((uint8_t)((r5 << 2) | (o.status & 3)));
    }
    return w.ok() ? w.n : -1;
}
inline bool decode(const uint8_t* buf, int len, Snapshot& s) {
    Reader r(buf, len);
    if (r.u8() != MSG_SNAPSHOT) return false;
    s.seq = r.u16();
    if (!get_zones(r, s.nz, s.z)) return false;
    s.arena_w = r.u16(); s.arena_h = r.u16();
    s.nr = r.u8(); s.no = r.u8();
    if (s.nr > MAX_ROBOTS || s.no > MAX_OBJECTS) return false;
    for (int i = 0; i < s.nr; i++) {
        SnapRobot& x = s.r[i];
        x.id = r.u8(); x.x = r.i16(); x.y = r.i16(); x.th = r.i16(); x.alive = r.u8(); x.task = r.u8(); x.state = r.u8();
    }
    for (int i = 0; i < s.no; i++) {
        SnapObject& o = s.o[i];
        o.id = r.u8(); o.x = r.i16(); o.y = r.i16();
        uint8_t dk = r.u8(); o.demand = dk & 0x0F; o.kind = dk >> 4;
        uint8_t sr = r.u8(); o.status = sr & 3; o.r = (uint16_t)((sr >> 2) * 5);
    }
    return r.ok() && r.n == len;
}

inline int encode(const Heartbeat& h, uint8_t* buf, int cap) {
    Writer w(buf, cap);
    w.u8(MSG_HEARTBEAT); w.u8(h.id); w.u16(h.seen_seq); w.u8(h.state); w.u8(h.task); w.u8(h.help);
    w.u8(h.neighbors); w.u16(h.batt_mv);
    return w.ok() ? w.n : -1;
}
inline bool decode(const uint8_t* buf, int len, Heartbeat& h) {
    Reader r(buf, len);
    if (r.u8() != MSG_HEARTBEAT) return false;
    h.id = r.u8(); h.seen_seq = r.u16(); h.state = r.u8(); h.task = r.u8(); h.help = r.u8();
    h.neighbors = r.u8(); h.batt_mv = r.u16();
    return r.ok() && r.n == len;
}

// ---- serial framing between laptop and gateway: A5 5A len payload crc8 --------
inline uint8_t crc8(const uint8_t* d, int n) {  // CRC-8/ATM, poly 0x07
    uint8_t c = 0;
    for (int i = 0; i < n; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
    return c;
}
inline int frame(const uint8_t* payload, int n, uint8_t* out) {
    out[0] = 0xA5; out[1] = 0x5A; out[2] = (uint8_t)n;
    memcpy(out + 3, payload, n);
    out[3 + n] = crc8(payload, n);
    return n + 4;
}
// Byte-at-a-time deframer. feed() returns payload length when a valid frame completes, else 0.
struct Deframer {
    uint8_t buf[256]; int stage = 0, len = 0, got = 0;
    int feed(uint8_t b) {
        switch (stage) {
            case 0: stage = (b == 0xA5) ? 1 : 0; return 0;
            case 1: stage = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); return 0;
            case 2: len = b; got = 0; stage = len ? 3 : 0; return 0;
            case 3: buf[got++] = b; if (got == len) stage = 4; return 0;
            default: stage = 0; return crc8(buf, len) == b ? len : 0;
        }
    }
};

}  // namespace swarm
