// Host unit tests for core/: wire format, framing, allocation rules, gateway bookkeeping.
//   c++ -std=c++17 -Wall -fsanitize=address,undefined -I core test/test_core.cpp -o build/test_core && build/test_core
#include "brain.h"
#include "world.h"
#include <cassert>
#include <cstdio>
#include <cstring>

using namespace swarm;

static Snapshot arena() {
    Snapshot s{};
    s.nz = 1; s.z[0] = {1320, 500, 160}; s.arena_w = 1500; s.arena_h = 1000;
    return s;
}
static void robot(Snapshot& s, uint8_t id, int x, int y, uint8_t alive = 1) { s.r[s.nr++] = {id, (int16_t)x, (int16_t)y, 0, alive, NONE, ST_IDLE}; }
static void load(Snapshot& s, uint8_t id, int x, int y, uint8_t demand = 1, uint8_t status = OBJ_OPEN) { s.o[s.no++] = {id, (int16_t)x, (int16_t)y, demand, status, 0}; }
static int count(const uint8_t* plan, int n, uint8_t task) { int c = 0; for (int i = 0; i < n; i++) c += plan[i] == task; return c; }

static void test_snapshot_roundtrip_and_size() {
    Snapshot s = arena(); s.seq = 513; s.nz = 3; s.z[1] = {100, 200, 90}; s.z[2] = {-5, 7, 60};
    for (int i = 0; i < MAX_ROBOTS; i++) s.r[s.nr++] = {(uint8_t)(i + 1), (int16_t)(-1000 + i), (int16_t)(32000 - i), (int16_t)(-3141 + i), 1, (uint8_t)i, ST_PUSH};
    for (int j = 0; j < MAX_OBJECTS; j++) s.o[s.no++] = {(uint8_t)(j + 1), (int16_t)(j * 10), (int16_t)(-j), 2, OBJ_STUCK, (uint8_t)(j % 3), (uint16_t)(40 + j * 5)};
    uint8_t buf[MAX_PAYLOAD];
    int n = encode(s, buf, sizeof buf);
    assert(n > 0 && n <= MAX_PAYLOAD);   // a full snapshot must fit one ESP-NOW packet
    Snapshot d{};
    assert(decode(buf, n, d));
    assert(d.seq == 513 && d.nr == MAX_ROBOTS && d.no == MAX_OBJECTS);
    assert(d.r[0].x == -1000 && d.r[0].y == 32000 && d.r[0].th == -3141 && d.r[5].task == 5);
    assert(d.o[15].x == 150 && d.o[15].y == -15 && d.o[15].status == OBJ_STUCK && d.o[15].demand == 2 && d.o[15].kind == 0 && d.o[14].kind == 2);
    assert(d.o[0].r == 40 && d.o[15].r == 115);   // load size rides in the status byte, 5 mm steps
    assert(d.nz == 3 && d.z[2].x == -5 && d.z[1].r == 90);
    assert(!decode(buf, n - 1, d));      // truncated packet rejected
    buf[0] = MSG_HEARTBEAT;
    assert(!decode(buf, n, d));          // wrong type rejected
}

static void test_vision_golden_bytes() {
    // Same frame is packed by satellite/proto.py; tests there compare against these bytes.
    Vision v{}; v.seq = 7; v.nz = 1; v.z[0] = {1320, 500, 160}; v.arena_w = 1500; v.arena_h = 1000;
    v.nr = 1; v.r[0] = {2, 300, -40, 1571}; v.no = 1; v.o[0] = {9, 812, 433, 1, 20};   // r = 20 -> 40 mm
    uint8_t buf[64]; int n = encode(v, buf, sizeof buf);
    const uint8_t golden[] = {1, 7, 0, 1, 0x28, 0x05, 0xF4, 0x01, 0xA0, 0x00, 0xDC, 0x05, 0xE8, 0x03, 1, 1,
                              2, 0x2C, 0x01, 0xD8, 0xFF, 0x23, 0x06, 9, 0x2C, 0x03, 0xB1, 0x01, 1, 20};
    assert(n == (int)sizeof golden && !memcmp(buf, golden, n));
    uint8_t fr[80]; int fn = frame(buf, n, fr);
    Deframer df; int got = 0;
    uint8_t noise[] = {0x00, 0xA5, 0x13, 0xA5};   // junk and a false start before the real frame
    for (uint8_t b : noise) assert(!df.feed(b));
    for (int i = 0; i < fn; i++) got = df.feed(fr[i]);
    assert(got == n && !memcmp(df.buf, buf, n));
    fr[5] ^= 0x40;                                  // corrupted byte -> CRC rejects it
    Deframer df2; got = 0;
    for (int i = 0; i < fn; i++) got = df2.feed(fr[i]);
    assert(got == 0);
}

static void test_five_pencils_two_robots_split() {
    Snapshot s = arena();
    robot(s, 1, 150, 250); robot(s, 2, 150, 750);
    load(s, 1, 500, 200); load(s, 2, 650, 820); load(s, 3, 400, 520); load(s, 4, 880, 330); load(s, 5, 760, 640);
    uint8_t plan[MAX_ROBOTS]; allocate(s, Tuning(), plan);
    assert(plan[0] != NONE && plan[1] != NONE && plan[0] != plan[1]);   // two different pencils at once
}

static Tuning push_mode() { Tuning t; t.carry = false; return t; }   // robots without a gripper

static void test_one_pencil_two_robots_share() {
    Snapshot s = arena();
    robot(s, 1, 150, 300); robot(s, 2, 150, 700); load(s, 1, 600, 500);
    uint8_t plan[MAX_ROBOTS]; allocate(s, push_mode(), plan);
    assert(plan[0] == 1 && plan[1] == 1);                                 // pushing: both help with the only pencil
    allocate(s, Tuning(), plan);
    assert(count(plan, 2, 1) == 1 && count(plan, 2, NONE) == 1);          // carrying: one gripper is enough, the other stays parked
}

static void test_heavy_needs_full_team_before_start() {
    // Box needs 2; there's also a pencil; 2 robots. Either both take the box, or one takes
    // the pencil and the other lines up at the box -- never leaves the box half-staffed AND
    // the pencil untouched.
    Snapshot s = arena();
    robot(s, 1, 150, 250); robot(s, 2, 150, 800); load(s, 1, 560, 330, 2); load(s, 2, 520, 800, 1);
    uint8_t plan[MAX_ROBOTS]; allocate(s, Tuning(), plan);
    assert(plan[0] != NONE && plan[1] != NONE);
    bool both_box = plan[0] == 1 && plan[1] == 1;
    bool split = count(plan, 2, 1) == 1 && count(plan, 2, 2) == 1;
    assert(both_box || split);
}

static void test_dead_robots_get_no_work_and_cap() {
    Snapshot s = arena();
    robot(s, 1, 200, 200); robot(s, 2, 200, 500, 0); robot(s, 3, 200, 800); robot(s, 4, 250, 500);
    load(s, 1, 600, 500);
    uint8_t plan[MAX_ROBOTS]; allocate(s, push_mode(), plan);
    assert(plan[1] == NONE);                       // dead robot: no job
    assert(count(plan, 4, 1) == 2);                // a pencil gets at most 2 robots (a helper beside the pusher)
}

static void test_heavy_load_gets_no_third_robot() {
    // Three robots, one 2-robot box: a third pusher wouldn't fit behind it, so it stays free.
    Snapshot s = arena();
    robot(s, 1, 200, 200); robot(s, 2, 200, 500); robot(s, 3, 200, 800);
    load(s, 1, 600, 500, 2);
    uint8_t plan[MAX_ROBOTS]; allocate(s, Tuning(), plan);
    assert(count(plan, 3, 1) == 2 && count(plan, 3, NONE) == 1);
}

static void test_allocation_is_identical_on_every_robot() {
    // Leaderless consensus: robots never negotiate, so the same snapshot must give the same
    // plan no matter who computes it -- and regardless of the order robots appear in.
    Snapshot a = arena();
    robot(a, 3, 300, 300); robot(a, 1, 300, 700); robot(a, 2, 200, 500);
    load(a, 7, 700, 300); load(a, 8, 700, 700); load(a, 9, 900, 500, 2);
    Snapshot b = a;                                // same world, robots listed in reverse
    for (int i = 0; i < a.nr; i++) b.r[i] = a.r[a.nr - 1 - i];
    uint8_t pa[MAX_ROBOTS], pb[MAX_ROBOTS];
    allocate(a, Tuning(), pa); allocate(b, Tuning(), pb);
    for (int i = 0; i < a.nr; i++) {
        int j = a.nr - 1 - i;
        if (pa[i] != pb[j]) { printf("robot %d: %d vs %d\n", a.r[i].id, pa[i], pb[j]); assert(false); }
    }
    Brain r1(1), r3(3);
    r1.on_snapshot(a, 10); r3.on_snapshot(a, 10);
    assert(!memcmp(r1.plan, r3.plan, a.nr));
}

static void test_world_delivery_help_and_stuck() {
    World w;
    Vision v{}; v.nz = 1; v.z[0] = {1320, 500, 160}; v.arena_w = 1500; v.arena_h = 1000;
    v.nr = 1; v.r[0] = {1, 300, 300, 0};
    v.no = 2; v.o[0] = {1, 1320 + 155, 500}; v.o[1] = {2, 700, 500};   // #1 on the zone's edge: not delivered yet
    w.on_vision(v, 100);
    w.on_heartbeat(Heartbeat{1, 0, ST_PUSH, 2, NONE, 0, 7400}, 100);
    Snapshot s = w.snapshot(100);
    assert(s.nr == 1 && s.r[0].alive && s.r[0].task == 2);
    assert(s.o[0].status == OBJ_OPEN);
    v.o[0].x = 1330; w.on_vision(v, 200);                                 // pushed deep inside
    assert(w.snapshot(200).o[0].status == OBJ_DELIVERED);

    // One robot alive, it reports load 2 won't budge: with nobody else to call, it's stuck.
    w.on_heartbeat(Heartbeat{1, 0, ST_PUSH, 2, 2, 0, 7400}, 300);
    s = w.snapshot(300);
    assert(s.o[1].status == OBJ_STUCK && s.o[1].demand == 1);
    // A second robot comes online: the stuck load reopens, now rated for 2.
    v.nr = 2; v.r[1] = {5, 300, 700, 0};
    w.on_vision(v, 400); w.on_heartbeat(Heartbeat{5, 0, ST_IDLE, NONE, NONE, 1, 7400}, 400);
    w.on_heartbeat(Heartbeat{1, 0, ST_IDLE, NONE, NONE, 1, 7400}, 400);
    s = w.snapshot(400);
    assert(s.o[1].status == OBJ_OPEN && s.o[1].demand == 2);
    // Repeated help inside the cooldown doesn't escalate again.
    w.on_heartbeat(Heartbeat{1, 0, ST_PUSH, 2, 2, 1, 7400}, 500);
    assert(w.snapshot(500).o[1].demand == 2);

    // Robot that stops heartbeating (battery) is no longer alive, even though it's visible.
    w.on_vision(v, 1600);
    s = w.snapshot(1600);
    assert(!s.r[0].alive && !s.r[1].alive);
}

static void test_sorting_by_kind() {
    // Two docks: a kind-1 load is pushed toward dock 1, and only counts as delivered there.
    Snapshot s = arena(); s.nz = 2; s.z[0] = {1300, 800, 150}; s.z[1] = {1300, 200, 150};
    load(s, 1, 700, 500); s.o[0].kind = 1;
    V2 u = push_dir(s, s.o[0], Tuning());
    assert(u.x > 0.8f && u.y < -0.4f);                       // heading for the lower dock, not the upper one
    World w;
    Vision v{}; v.nz = 2; v.z[0] = {1300, 800, 150}; v.z[1] = {1300, 200, 150}; v.arena_w = 1500; v.arena_h = 1000;
    v.no = 2; v.o[0] = {1, 1300, 800, 1}; v.o[1] = {2, 1300, 200, 1};   // both kind 1: one sits in the wrong dock
    w.on_vision(v, 10);
    Snapshot out = w.snapshot(10);
    assert(out.o[0].status == OBJ_OPEN && out.o[1].status == OBJ_DELIVERED);
}

static void test_brain_safety_stop() {
    Snapshot s = arena(); robot(s, 1, 300, 500); load(s, 1, 700, 500);
    Brain b(1); float l, r;
    b.step(0, l, r); assert(l == 0 && r == 0 && b.state == ST_STOPPED);   // no world yet: don't move
    b.on_snapshot(s, 1000);
    b.step(1010, l, r); b.step(1030, l, r);
    assert(b.state != ST_STOPPED);
    b.step(1000 + Tuning().snapshot_timeout_ms + 1, l, r);                // camera/radio silent: stop
    assert(l == 0 && r == 0 && b.state == ST_STOPPED);
    b.on_snapshot(s, 2000); b.estop = true; b.step(2001, l, r);
    assert(l == 0 && r == 0);
}

static void test_neighbour_discovery() {
    Brain b(1);
    assert(b.neighbors(100) == 0);
    b.on_heartbeat(Heartbeat{2, 0, 0, NONE, NONE, 0, 0}, 100);
    b.on_heartbeat(Heartbeat{3, 0, 0, NONE, NONE, 0, 0}, 120);
    b.on_heartbeat(Heartbeat{2, 0, 0, NONE, NONE, 0, 0}, 150);
    b.on_heartbeat(Heartbeat{1, 0, 0, NONE, NONE, 0, 0}, 150);    // its own echo doesn't count
    assert(b.neighbors(200) == 2);
    assert(b.neighbors(1130) == 1);                                // robot 3 silent for > 1 s
}

static void test_motor_ramp() {
    const float dv = 8 * 0.02f;                                     // one 20 ms tick at the default ramp
    assert(fabsf(Brain::slew(1, 0, dv) - dv) < 1e-6f);              // speeding up is rate-limited
    assert(Brain::slew(0.3f, 0.9f, dv) == 0.3f);                    // slowing down is instant
    assert(Brain::slew(0, 1, dv) == 0);                             // so is stopping
    assert(fabsf(Brain::slew(-1, 0.8f, dv) + dv) < 1e-6f);          // reversing brakes to 0, then ramps
}

// Pattern learning: both robots learn the same hotspot from the same snapshots; exactly one waits
// beside it, on the far side from the dock. On a small floor the trip home is short: nobody waits.
static void test_learned_waiting_spot() {
    for (int big = 0; big < 2; big++) {
        int k = big ? 2 : 1;
        Snapshot s{}; s.nz = 1; s.z[0] = {(int16_t)(1320 * k), (int16_t)(500 * k), 150}; s.arena_w = 1500 * k; s.arena_h = 1000 * k;
        robot(s, 1, 130, 300 * k); robot(s, 2, 130, 700 * k);
        Brain a(1), b(2);
        uint32_t now = 1;
        for (uint8_t id = 1; id <= 4; id++) {   // four parcels arrive at the same bay, one after another
            Snapshot with = s; load(with, id, 750 * k, 500 * k);
            Brain* both[2] = {&a, &b};
            for (Brain* x : both) { x->on_snapshot(s, now); x->on_snapshot(with, now + 100); x->on_snapshot(s, now + 200); }
            now += 300;
        }
        if (!big) { assert(!a.has_wait_ && !b.has_wait_); continue; }
        assert(a.has_wait_ != b.has_wait_);
        V2 w = a.has_wait_ ? a.wait_ : b.wait_;
        assert(fabsf(len(w - V2{1500, 1000}) - 250) < 40 && w.x < 1500);
    }
}

int main() {
    test_snapshot_roundtrip_and_size();
    test_vision_golden_bytes();
    test_five_pencils_two_robots_split();
    test_one_pencil_two_robots_share();
    test_heavy_needs_full_team_before_start();
    test_dead_robots_get_no_work_and_cap();
    test_heavy_load_gets_no_third_robot();
    test_allocation_is_identical_on_every_robot();
    test_world_delivery_help_and_stuck();
    test_sorting_by_kind();
    test_brain_safety_stop();
    test_neighbour_discovery();
    test_motor_ramp();
    test_learned_waiting_spot();
    puts("core tests: all passed");
}
