// The robot's brain. Runs on every robot; nothing here is decided centrally.
//
// Each robot receives the same snapshot (positions only), runs the same deterministic
// allocator, and so arrives at the same answer for "who does what" without a leader
// or a negotiation round. It then plans its own motion: line up behind its object,
// wait for teammates if the load needs several robots, push it into its dock,
// and ask for help (raise the load's headcount) if the load won't move.
#pragma once
#include "proto.h"
#include <math.h>

namespace swarm {

// Calibration knobs. Defaults are sized for ~12 cm two-wheel robots; measure vmax and
// wheel_base on your hardware (see firmware/README) and adjust.
struct Tuning {
    float robot_radius = 60;     // mm, half the chassis width
    float object_radius = 45;    // mm, effective footprint of a pencil
    float team_spacing = 120;    // mm between side-by-side pushers
    float approach_back = 170;   // mm behind the object where a pusher lines up
    float push_lead = 250;       // mm ahead of the object the pusher aims at
    float contact_dist = 135;    // mm: closer than this along the push line = touching
    float arrive_tol = 40;       // mm
    float align_tol = 0.20f;     // rad (~11 deg)
    float cruise = 260;          // mm/s (measured: 260 beats 220 on delivery, finish time and energy; 290 left no steering headroom)
    float push_speed = 140;      // mm/s
    float vmax = 300;            // mm/s at full motor command
    float wheel_base = 110;      // mm between wheel contact points
    float k_turn = 2.5f;         // rad/s per rad of heading error
    float turn_in_place = 1.0f;  // rad: above this error, rotate on the spot
    float max_turn = 3.0f;       // rad/s
    float avoid_radius = 170;    // mm of clearance kept from other robots/objects
    float lost_lateral = 70;     // mm the object may slide off the push line
    float sticky = 150;          // mm bonus for keeping the current job (anti-thrash)
    float help_penalty = 300;    // mm cost of joining an object that already has enough robots
    uint32_t stall_ms = 2500;    // pushing this long without the object moving = too heavy
    float stall_dist = 15;       // mm
    uint32_t backoff_ms = 700;
    uint32_t sync_wait_ms = 3000;  // max wait for a nearby teammate so helpers push together
    uint32_t snapshot_timeout_ms = 500;  // no world update for this long = stop the motors

    // Carry mode: a gripper on the robot's front (GRIPPER_PIN in the firmware) holds the load,
    // like real warehouse robots. Off = push mode, for robots without a gripper.
    bool carry = true;
    float pre_dock = 80;          // mm: line up this far back from the grip point, then drive straight in
    float dock_speed = 90;        // mm/s for the last stretch onto a load or into a dock slot
    float carry_speed = 240;      // mm/s with a load on the gripper
    float turn_cost = 150;        // mm of travel worth one unit of (1 - cos) between grip side and dock direction
    uint32_t grip_ms = 300;       // gripper on, robot still, before moving off
    float grip_tol = 45;          // mm the load may sit off the gripper before it counts as dropped
    uint8_t grip_retries = 2;     // failed grips before asking the gateway to escalate
    float carry_body = 60;        // mm: planner footprint while carrying = the chassis; steering guards keep the held load itself off loads and walls (measured: 60 beats 80-110)
    uint8_t overbook = 2;         // pickups allowed beyond a dock's free slots: carriers stage beside a full dock, ready the moment it clears (measured: 2 halves the wait; 3+ causes collisions on dense floors)
    bool learn = true;            // learn where loads arrive; idle robots wait beside the busiest spots instead of going home
    float wait_gap = 400;         // mm from a hotspot's centre to where an idle robot waits: outside where parcels get dropped (measured, 6 m2 floor: 250 blocked the unloading, wait 11.4 s; 400: 8.5 s; manager off: 9.8 s)
    float save_s = 5;             // a hotspot gets a waiting robot only if that saves at least this much driving from home (measured: on a small floor the standing post costs more than it saves)
    float work_density = 0;       // robots on duty per m2 of floor; the rest park (0 = everyone works). Off: measured on a saturated 1.5 m2 floor, 3 per m2 cut energy per parcel 18-32% but added no parcels, and parked robots don't yet keep to the walls. See allocate()
    uint32_t deadlock_ms = 20000; // every carrier on the floor without progress this long = deadlock: one sets its load down (see watch_carriers)
    uint32_t cooldown_ms = 30000; // a load set down to break a deadlock isn't picked up again for this long
    float cell_margin = 1;        // mm added to the chassis radius: each robot keeps this far inside its half of the gap to every other robot (0 = off; 20 froze robots packed at the start of a dense floor). See keep_in_cell()
    float cell_horizon = 0.3f;    // s: approach a cell's edge no faster than this lets us stop at it
    float ramp = 8;               // wheel command per second a wheel may speed up by (0 to full in 125 ms): no inrush spikes to brown out the ESP32; slowing and stopping are instant. Measured: 0 hard starts (was 45,576 per 480 runs), delivery unchanged
};

struct V2 { float x, y; };
inline V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
inline V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
inline V2 operator*(V2 a, float k) { return {a.x * k, a.y * k}; }
inline float dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
inline float len(V2 a) { return sqrtf(a.x * a.x + a.y * a.y); }
inline V2 unit(V2 a) { float l = len(a); return l > 1e-6f ? a * (1.0f / l) : V2{1, 0}; }
inline V2 rot(V2 a, float r) { float c = cosf(r), s = sinf(r); return {a.x * c - a.y * s, a.x * s + a.y * c}; }
inline float wrap(float a) { while (a > 3.14159265f) a -= 6.2831853f; while (a < -3.14159265f) a += 6.2831853f; return a; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
inline float seg_dist(V2 p, V2 a, V2 b) {  // distance from p to segment ab
    V2 ab = b - a; float l2 = dot(ab, ab);
    float t = l2 > 0 ? clampf(dot(p - a, ab) / l2, 0, 1) : 0;
    return len(p - (a + ab * t));
}

inline V2 pos(const SnapRobot& r) { return {(float)r.x, (float)r.y}; }
inline V2 pos(const SnapObject& o) { return {(float)o.x, (float)o.y}; }

// Keep a point where a robot can physically stand (clear of the arena walls).
inline V2 keep_in(const Snapshot& s, V2 a, const Tuning& t) {
    if (!s.arena_w) return a;
    float m = t.robot_radius + 15;   // close enough to a wall to push a load off it at an angle
    return {clampf(a.x, m, s.arena_w - m), clampf(a.y, m, s.arena_h - m)};
}
// Push direction for an object: at the drop zone, unless the spot behind it is inside a
// wall. Then push from the nearest reachable spot instead, which slides the object away
// from the wall; the direction re-aims at the zone as the object moves.
//
// If that lining-up spot is occupied by something that won't move -- a dead robot or
// another load -- swing the push angle (20 deg steps, smallest first) to a free spot.
// Depends only on the snapshot, so every robot computes the same direction.
// slack > 0 accepts a spot that is that much tighter (hysteresis for a spot already in use).
inline bool spot_free(const Snapshot& s, const SnapObject& o, V2 a, const Tuning& t, float slack = 0) {
    for (int i = 0; i < s.nr; i++)
        if (!s.r[i].alive && len(pos(s.r[i]) - a) < 2 * t.robot_radius + 10 - slack) return false;
    for (int j = 0; j < s.no; j++)
        if (s.o[j].id != o.id && s.o[j].status != OBJ_DELIVERED && len(pos(s.o[j]) - a) < t.robot_radius + t.object_radius + 10 - slack) return false;
    return true;
}
// Each load goes to the dock for its kind (colour), so the swarm sorts as it delivers.
inline V2 dock_of(const Snapshot& s, const SnapObject& o) {
    if (!s.nz) return pos(o) + V2{1, 0};
    const Zone& d = s.z[o.kind % s.nz];
    return {(float)d.x, (float)d.y};
}
inline V2 push_dir(const Snapshot& s, const SnapObject& o, const Tuning& t) {
    V2 z = unit(dock_of(s, o) - pos(o));
    V2 u = unit(pos(o) - keep_in(s, pos(o) - z * t.approach_back, t));
    for (int k = 0; k <= 8; k++) {
        V2 r = rot(u, (k % 2 ? 1 : -1) * ((k + 1) / 2) * 0.349f);
        V2 a = pos(o) - r * t.approach_back;
        if (len(keep_in(s, a, t) - a) < 1 && spot_free(s, o, a, t)) return r;
    }
    return u;
}

// ---- carry mode: grip side and dock slots ------------------------------------------
inline float grip_reach(const Tuning& t) { return t.robot_radius + t.object_radius; }   // robot centre to held load centre
inline float orad(const SnapObject& o, const Tuning& t) { return o.r ? (float)o.r : t.object_radius; }   // camera-measured size if known
inline float grip_reach(const Tuning& t, const SnapObject& o) { return t.robot_radius + orad(o, t); }

// Room a robot needs to turn on the spot with a load on its gripper: the radius it sweeps.
inline bool room_to_turn(const Snapshot& s, V2 a, const Tuning& t, float slack = 0) {
    float m = grip_reach(t) + t.object_radius - slack;
    return !s.arena_w || (a.x >= m && a.y >= m && a.x <= s.arena_w - m && a.y <= s.arena_h - m);
}
// The side to grip a load from. Any side works where the robot lines up with room to turn a load
// afterwards -- it can back a load off a wall -- so take the cheapest for a robot at `from`:
// travel there, plus turning toward the dock.
// `half` > 0: two robots side by side, this far either side of the load's centre line.
// A pair can't latch its face privately (the two could pick different ones), so it recomputes from
// the snapshot every tick. To stop that flickering, the robots already on the load count too: the
// cost includes their distance to the face's two spots, and the face they're both lined up at keeps
// a 15 mm allowance on its room checks, so camera noise at a threshold can't pull them away
// (measured: random layout 67, the face flipped every ~2 s and the pair chased it for 4 minutes).
inline V2 grip_dir(const Snapshot& s, const SnapObject& o, V2 from, const Tuning& t, float half = 0) {
    V2 z = unit(dock_of(s, o) - pos(o)), best = z;
    float bc = 1e30f, back = grip_reach(t, o) + t.pre_dock;
    int team[2], nt = 0;
    if (half > 0)
        for (int i = 0; i < s.nr && nt < 2; i++) if (s.r[i].alive && s.r[i].task == o.id) team[nt++] = i;
    for (int k = 0; k < 8; k++) {
        V2 d = rot(z, k * 0.7853982f), a = pos(o) - d * back, side = V2{-d.y, d.x} * half;
        float on = 0;   // the team's distance to this face's spots (best pairing)
        if (nt == 2) on = fminf(len(a + side - pos(s.r[team[0]])) + len(a - side - pos(s.r[team[1]])),
                                len(a - side - pos(s.r[team[0]])) + len(a + side - pos(s.r[team[1]])));
        else if (nt == 1) on = fminf(len(a + side - pos(s.r[team[0]])), len(a - side - pos(s.r[team[0]])));
        float slack = nt && on < 60.0f * nt ? 15 : 0;
        if (!room_to_turn(s, a + side, t, slack) || !room_to_turn(s, a - side, t, slack) || !spot_free(s, o, a + side, t, slack) || !spot_free(s, o, a - side, t, slack)) continue;
        float c = len(a - from) + t.turn_cost * (1 - dot(d, z)) + on;
        if (c < bc) { bc = c; best = d; }
    }
    return best;
}

// Slots in dock k: a ring with room between loads. Each is filled by a robot that lines up outside
// it and drives straight in (in[] = the way it faces), so the line-up point needs room to turn with
// a load on the gripper. No centre slot: reaching it means passing loads already in the ring.
inline int dock_slots(const Snapshot& s, int k, const Tuning& t, V2* slot, V2* in) {
    const Zone& d = s.z[k];
    V2 c = {(float)d.x, (float)d.y};
    float rho = fminf(d.r - t.object_radius - 10, 2 * t.object_radius + 20);
    int n = 0, ring = 0;
    if (rho > t.object_radius) ring = (int)(6.2831853f / (2 * asinf(t.object_radius / rho)));
    if (ring > 6) ring = 6;
    for (int i = 0; i < ring; i++) {
        V2 u = rot(V2{-1, 0}, i * 6.2831853f / ring), stage = c + u * (rho + grip_reach(t) + t.pre_dock);
        if (!room_to_turn(s, stage, t)) continue;
        slot[n] = c + u * rho; in[n] = u * -1.0f; n++;
    }
    return n;
}

inline float job_cost(const Snapshot& s, const SnapRobot& r, const SnapObject& o, const Tuning& t) {
    bool pair = o.demand >= 2;   // a pair grips from a side both agree on, so it can't depend on who's asking
    V2 a = t.carry ? pos(o) - grip_dir(s, o, pair ? pos(o) : pos(r), t, pair ? t.team_spacing * 0.5f : 0) * (grip_reach(t, o) + t.pre_dock)
                   : pos(o) - push_dir(s, o, t) * t.approach_back;
    return len(a - pos(r)) - (r.task == o.id ? t.sticky : 0);
}
inline bool holding_state(uint8_t st) { return st == ST_GRIP || st == ST_CARRY || st == ST_PLACE; }

// Who does what. Deterministic: the same snapshot gives the same answer on every robot.
// out[i] is the object id for snapshot robot i (or NONE).
//  Pass 1: fully staff whole jobs, cheapest average travel first. A job that needs k
//          robots is only started when k free robots exist (so 5 pencils / 2 robots
//          splits the work, and a heavy box isn't half-staffed while pencils wait).
//  Pass 2: leftover robots go where they help most: lining up at a job that is still
//          short of robots, or joining a job already underway (1 pencil / 2 robots
//          means both push it). At most 2 robots per load: that's how many fit behind one.
// reach: bit j set = load j can be got to (see Brain::reachable_loads); all by default.
// benched: bit i set = snapshot robot i is off duty (ramp metering, below).
inline void allocate(const Snapshot& s, const Tuning& t, uint8_t out[MAX_ROBOTS], uint32_t reach = 0xFFFFFFFFu, uint32_t* benched = nullptr) {
    bool free_[MAX_ROBOTS] = {};
    int nfree = 0, team[MAX_OBJECTS] = {};
    // Admission control (carry mode): only start a pickup if its dock has a free slot that no
    // carrier has claimed yet, plus a couple staged beside it. Otherwise every robot grabs a load
    // at once and the floor gridlocks. A load lying in a dock's lane is cleared first, and until it
    // is, nobody stages for that dock: carriers holding loads for slots that only a free robot can
    // unblock, with every robot holding, is a hold-and-wait deadlock (measured: random layout 251
    // froze all 4 robots for good).
    int room[MAX_ZONES] = {};
    uint32_t blocker = 0;   // loads lying in a dock lane: always worth picking up, that's what frees the lane
    if (t.carry)
        for (int k = 0; k < s.nz; k++) {
            V2 S[8], I[8];
            int n = dock_slots(s, k, t, S, I);
            bool lane_blocked = false;
            for (int q = 0; q < n; q++) {   // free, with its approach lane clear (as the carrier itself will judge it)
                V2 a = S[q] - I[q] * (grip_reach(t) + t.pre_dock), b = S[q] - I[q] * grip_reach(t);
                bool busy = false;
                for (int j = 0; j < s.no; j++) {   // no early exit: every lane blocker must be found
                    bool held = false;
                    for (int i = 0; i < s.nr; i++) held = held || (s.r[i].alive && s.r[i].task == s.o[j].id && (s.r[i].state == ST_GRIP || s.r[i].state == ST_CARRY));
                    bool in_lane = !held && s.o[j].status == OBJ_OPEN && seg_dist(pos(s.o[j]), a, b) < t.carry_body + t.object_radius;
                    if (in_lane) { blocker |= 1u << j; lane_blocked = true; }
                    busy = busy || len(pos(s.o[j]) - S[q]) < 2 * t.object_radius - 10 || in_lane;
                }
                room[k] += !busy;
            }
            if (!lane_blocked) room[k] += t.overbook;
            for (int i = 0; i < s.nr; i++)   // carriers already heading there have claimed theirs
                if (s.r[i].alive && (s.r[i].state == ST_GRIP || s.r[i].state == ST_CARRY))
                    for (int j = 0; j < s.no; j++) if (s.o[j].id == s.r[i].task && s.o[j].kind % s.nz == k) room[k]--;
        }
    auto admits = [&](const SnapObject& o) {
        int j = (int)(&o - s.o);
        return (reach >> j & 1) && (!t.carry || !s.nz || room[o.kind % s.nz] > 0 || (blocker >> j & 1));
    };
    for (int i = 0; i < s.nr; i++) { out[i] = NONE; if (s.r[i].alive) { free_[i] = true; nfree++; } }
    // A robot holding a load keeps it: never reshuffle a job mid-carry.
    for (int i = 0; i < s.nr; i++) {
        if (!free_[i] || !holding_state(s.r[i].state)) continue;
        for (int j = 0; j < s.no; j++)
            if (s.o[j].id == s.r[i].task && s.o[j].status == OBJ_OPEN) { out[i] = s.o[j].id; free_[i] = false; nfree--; team[j]++; }
    }
    // Ramp metering: like the lights on a freeway on-ramp, only let as many robots work as the
    // floor can use. Past that, extra robots add no parcels, only energy and traffic.
    // On duty, in order: loads in hand, robots already on a job, then the robots nearest the work,
    // like a manager sending the closest hands. Parked robots are then the ones furthest back, so
    // they can't wall the workers in (measured: by id, parked robots boxed a worker in for good).
    if (benched) *benched = 0;
    if (t.work_density > 0 && s.arena_w) {
        int cap = (int)(t.work_density * s.arena_w * s.arena_h * 1e-6f), duty = 0;
        bool on[MAX_ROBOTS] = {};
        float near[MAX_ROBOTS];
        for (int i = 0; i < s.nr; i++) {
            near[i] = 1e30f;
            for (int j = 0; j < s.no; j++) if (s.o[j].status == OBJ_OPEN) near[i] = fminf(near[i], len(pos(s.o[j]) - pos(s.r[i])));
            if (s.r[i].alive && !free_[i]) { on[i] = true; duty++; }
        }
        if (cap < duty + 1) cap = duty + 1;   // always one free hand beyond the loads in hand: if carriers jam, someone can clear the way (measured: without it, 4 boxed-in carriers stalled a dense floor for good)
        for (int pass = 0; pass < 2; pass++)
            for (;;) {
                int ri = -1;
                for (int i = 0; i < s.nr; i++)
                    if (free_[i] && !on[i] && (pass || s.r[i].task != NONE) &&
                        (ri < 0 || near[i] < near[ri] - 1e-3f || (fabsf(near[i] - near[ri]) <= 1e-3f && s.r[i].id < s.r[ri].id))) ri = i;
                if (ri < 0 || duty >= cap) break;
                on[ri] = true; duty++;
            }
        for (int i = 0; i < s.nr; i++)
            if (free_[i] && !on[i]) { free_[i] = false; nfree--; if (benched) *benched |= 1u << i; }
    }

    for (;;) {
        int best_o = -1; float best_avg = 1e30f; int best_sel[MAX_ROBOTS]; int best_k = 0;
        for (int j = 0; j < s.no; j++) {
            const SnapObject& o = s.o[j];
            int k = o.demand ? o.demand : 1;
            if (o.status != OBJ_OPEN || team[j] >= k || k - team[j] > nfree || (!team[j] && !admits(o))) continue;
            k -= team[j];   // a carrier already on it counts
            int sel[MAX_ROBOTS]; float sum = 0; bool taken[MAX_ROBOTS] = {};
            for (int n = 0; n < k; n++) {  // k cheapest free robots for this object
                int bi = -1; float bc = 1e30f;
                for (int i = 0; i < s.nr; i++) {
                    if (!free_[i] || taken[i]) continue;
                    float c = job_cost(s, s.r[i], o, t);
                    if (c < bc - 1e-3f || (fabsf(c - bc) <= 1e-3f && bi >= 0 && s.r[i].id < s.r[bi].id)) { bc = c; bi = i; }
                }
                taken[bi] = true; sel[n] = bi; sum += bc;
            }
            float avg = sum / k - (blocker >> j & 1 ? 1e5f : 0);   // lane blockers first
            if (avg < best_avg - 1e-3f || (fabsf(avg - best_avg) <= 1e-3f && best_o >= 0 && o.id < s.o[best_o].id)) {
                best_avg = avg; best_o = j; best_k = k;
                for (int n = 0; n < k; n++) best_sel[n] = sel[n];
            }
        }
        if (best_o < 0) break;
        if (!team[best_o] && t.carry && s.nz) room[s.o[best_o].kind % s.nz]--;
        for (int n = 0; n < best_k; n++) { out[best_sel[n]] = s.o[best_o].id; free_[best_sel[n]] = false; }
        team[best_o] += best_k; nfree -= best_k;
    }

    for (int pass = 0; pass < s.nr && nfree > 0; pass++) {
        int ri = -1;  // lowest-id free robot
        for (int i = 0; i < s.nr; i++) if (free_[i] && (ri < 0 || s.r[i].id < s.r[ri].id)) ri = i;
        int bj = -1; float bc = 1e30f;
        for (int j = 0; j < s.no; j++) {
            const SnapObject& o = s.o[j];
            int k = o.demand ? o.demand : 1;
            // A spare robot helps a 1-robot load (two side by side push it together), but a
            // heavy load already has as many pushers as fit behind it -- a third would miss it.
            // In carry mode one gripper is enough for a 1-robot load: spare robots stay parked.
            if (o.status != OBJ_OPEN || team[j] >= (k >= 2 || t.carry ? k : 2) || (!team[j] && !admits(o))) continue;
            float c = job_cost(s, s.r[ri], o, t) + (team[j] >= k ? t.help_penalty : 0);
            if (c < bc - 1e-3f || (fabsf(c - bc) <= 1e-3f && bj >= 0 && o.id < s.o[bj].id)) { bc = c; bj = j; }
        }
        free_[ri] = false; nfree--;
        if (bj >= 0) { if (!team[bj] && t.carry && s.nz) room[s.o[bj].kind % s.nz]--; out[ri] = s.o[bj].id; team[bj]++; }
    }
}

struct Brain {
    uint8_t id;
    Tuning t;
    State state = ST_STOPPED;
    uint8_t task = NONE, help = NONE;
    bool estop = false, have_snap = false;
    Snapshot snap{};
    uint8_t plan[MAX_ROBOTS];
    uint32_t last_snap = 0, state_since = 0, stall_since = 0;
    V2 stall_at{0, 0};
    // carry mode
    bool grip = false;               // output: gripper on (read by the firmware / simulator)
    bool held_ = false;              // we think the load is on the gripper
    bool gd_set_ = false, inserting_ = false, home_set_ = false;
    V2 gd_{1, 0}, gd_at_{0, 0};      // latched grip side, and where the load was when we chose it
    V2 slot_{0, 0}, in_{1, 0};       // dock slot being filled, and the way we face to fill it
    V2 home_{0, 0};                  // where we park when there's no work (first place we were seen)
    uint8_t misses_ = 0;
    float cmd_ = 0;                  // how hard we drove the wheels last tick
    float wl_ = 0, wr_ = 0; uint32_t ramp_at_ = 0;   // last wheel commands, for the ramp
    float off_ = 0;                  // my offset across the load's face while carrying (0 alone, ±half in a pair)
    uint8_t help_demand_ = 0;        // the load's headcount when we asked for help
    uint32_t tug_since_ = 0;         // grip check in progress
    float stall_th_ = 0;             // heading when the stall timer started (turning on the spot is progress too)
    V2 carry_from_{0, 0};            // where the load was when we got a grip on it
    float best_[MAX_ROBOTS] = {};    // deadlock watch: each carrier's closest yet (its load to its dock), and when it got there
    uint32_t moved_at_[MAX_ROBOTS] = {};
    uint8_t mark_id_[MAX_ROBOTS] = {}, mark_task_[MAX_ROBOTS] = {};
    uint8_t victim_ = NONE;          // load the swarm agreed to set down this snapshot
    uint8_t cool_id_[4] = {}; uint32_t cool_t_[4] = {};   // loads set down to break a deadlock, and when
    uint8_t blocked_ = 0;            // times this carry got blocked and we lined up again
    uint32_t pair_wait_ = 0;         // holding a pair load, waiting for the partner to hold it too, since
    V2 odo_{0, 0}; uint32_t odo_at_ = 0;   // our own travel since the last snapshot (dead reckoning), for keep_in_cell()
    uint8_t gd_need_ = 0;            // headcount the grip side was chosen for (a pair needs the side they agree on)
    struct Peer { uint8_t id; uint32_t t; } peers[MAX_ROBOTS] = {};

    // Pattern learning: every robot keeps the same map of where new loads appear. They all learn
    // from the same snapshots, so there's still no central brain.
    static constexpr int HM = 20;    // map cells per side
    float heat_[HM * HM] = {}, hx_[HM * HM] = {}, hy_[HM * HM] = {};   // arrivals per cell, and the sum of where exactly (same decay)
    uint32_t seen_[8] = {};          // load ids on the floor in the last snapshot
    bool seen_init_ = false, has_wait_ = false;
    V2 wait_{0, 0};                  // where to wait while idle (beside a hotspot), if has_wait_
    uint32_t benched_ = 0;           // snapshot robots the metering has parked (bit per robot)
    bool staging_ = false;           // carrying, waiting beside a full dock (read by the website)

    explicit Brain(uint8_t id_, Tuning tune = Tuning()) : id(id_), t(tune) {}

    int me() const { for (int i = 0; i < snap.nr; i++) if (snap.r[i].id == id) return i; return -1; }
    int obj(uint8_t oid) const { for (int j = 0; j < snap.no; j++) if (snap.o[j].id == oid) return j; return -1; }

    void set_state(State s, uint32_t now) {
        if (s == state) return;
        if (state == ST_PUSH) help = NONE;     // only ask for help while actually pushing
        if (s == ST_PUSH) stall_since = 0;
        state = s; state_since = now;
    }

    V2 cell_centre(int c) const { return {(c % HM + 0.5f) * snap.arena_w / HM, (c / HM + 0.5f) * snap.arena_h / HM}; }
    void learn_arrivals() {
        uint32_t now_seen[8] = {};
        for (int j = 0; j < snap.no; j++) {
            const SnapObject& o = snap.o[j];
            bool known = (seen_[o.id >> 5] >> (o.id & 31)) & 1;
            now_seen[o.id >> 5] |= 1u << (o.id & 31);
            if (!seen_init_ || known || o.status != OBJ_OPEN || !snap.arena_w) continue;
            for (int c = 0; c < HM * HM; c++) { heat_[c] *= 0.97f; hx_[c] *= 0.97f; hy_[c] *= 0.97f; }   // older arrivals count less, so the map follows a changing day
            int cx = (int)clampf(o.x * HM / (float)snap.arena_w, 0, HM - 1), cy = (int)clampf(o.y * HM / (float)snap.arena_h, 0, HM - 1);
            heat_[cy * HM + cx] += 1; hx_[cy * HM + cx] += o.x; hy_[cy * HM + cx] += o.y;
        }
        for (int i = 0; i < 8; i++) seen_[i] = now_seen[i];
        seen_init_ = true;
    }
    // Where to wait while idle: beside the busiest arrival spot that no closer idle robot covers.
    // Same map, snapshot and plan on every robot, so idle robots spread out without talking.
    void choose_wait(int m) {
        bool had = has_wait_; V2 was = wait_;
        has_wait_ = false;
        if (!t.learn || !t.carry || !snap.arena_w || plan[m] != NONE) return;
        float score[HM * HM];
        for (int c = 0; c < HM * HM; c++) {
            float sc = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int x = c % HM + dx, y = c / HM + dy;
                    if (x >= 0 && y >= 0 && x < HM && y < HM) sc += heat_[y * HM + x];
                }
            score[c] = sc;
        }
        bool taken[MAX_ROBOTS] = {};
        for (int k = 0; k < snap.nr; k++) {
            int best = 0;
            for (int c = 1; c < HM * HM; c++) if (score[c] > score[best]) best = c;
            if (score[best] < 1.5f) return;   // too few arrivals to call it a pattern
            V2 h{0, 0}; float wsum = 0;       // the hotspot: mean arrival point over its 3x3 cells
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int x = best % HM + dx, y = best / HM + dy;
                    if (x < 0 || y < 0 || x >= HM || y >= HM) continue;
                    h = h + V2{hx_[y * HM + x], hy_[y * HM + x]}; wsum += heat_[y * HM + x];
                }
            h = h * (1 / wsum);
            for (int c = 0; c < HM * HM; c++) if (len(cell_centre(c) - h) < 300) score[c] = 0;
            V2 spot;
            if (!spot_near(h, spot) || len(spot - home_) < t.save_s * t.cruise) continue;
            int who = -1; float bd = 1e30f;   // the idle robot nearest the spot takes it
            for (int i = 0; i < snap.nr; i++)
                if (!taken[i] && snap.r[i].alive && plan[i] == NONE && !(benched_ >> i & 1) && len(pos(snap.r[i]) - spot) < bd) { bd = len(pos(snap.r[i]) - spot); who = i; }
            if (who < 0) return;
            taken[who] = true;
            if (who == m) { wait_ = had && len(spot - was) < 150 ? was : spot; has_wait_ = true; return; }   // ignore small drifts of the hotspot
        }
    }
    // A place beside a hotspot: on its far side from the nearest dock, so a pickup heads straight
    // for the dock, clear of walls, loads and dock lanes.
    bool spot_near(V2 h, V2& out) const {
        V2 d{-1, 0}; float bd = 1e30f;
        for (int z = 0; z < snap.nz; z++) { V2 zp{(float)snap.z[z].x, (float)snap.z[z].y}; if (len(zp - h) < bd) { bd = len(zp - h); d = unit(h - zp); } }
        for (int k = 0; k < 8; k++) {
            V2 a = h + rot(d, (k % 2 ? 1 : -1) * ((k + 1) / 2) * 0.7853982f) * t.wait_gap;
            bool clear = room_to_turn(snap, a, t);
            for (int j = 0; j < snap.no; j++) clear = clear && len(pos(snap.o[j]) - a) >= t.robot_radius + orad(snap.o[j], t) + 30;
            for (int z = 0; z < snap.nz; z++) clear = clear && len(V2{(float)snap.z[z].x, (float)snap.z[z].y} - a) >= snap.z[z].r + t.robot_radius + 100;
            if (clear) { out = a; return true; }
        }
        return false;
    }

    // Discovery: every heartbeat heard from another robot counts it as a neighbour.
    void on_heartbeat(const Heartbeat& h, uint32_t now) {
        if (h.id == id) return;
        int slot = -1;
        for (int i = 0; i < MAX_ROBOTS; i++) if (peers[i].t && peers[i].id == h.id) slot = i;
        for (int i = 0; slot < 0 && i < MAX_ROBOTS; i++) if (!peers[i].t || now - peers[i].t > 5000) slot = i;
        if (slot >= 0) peers[slot] = {h.id, now ? now : 1};
    }
    uint8_t neighbors(uint32_t now) const {
        uint8_t n = 0;
        for (int i = 0; i < MAX_ROBOTS; i++) if (peers[i].t && now - peers[i].t <= 1000) n++;
        return n;
    }

    void on_snapshot(const Snapshot& s, uint32_t now) {
        snap = s; have_snap = true; last_snap = now; odo_ = {0, 0};
        int m = me();
        if (m < 0) return;
        if (!home_set_) { home_ = pos(snap.r[m]); home_set_ = true; }
        watch_carriers(now);
        if (victim_ != NONE && victim_ == task && holding_state(state)) { grip = held_ = gd_set_ = inserting_ = false; set_state(ST_BACKOFF, now); }
        uint32_t reach = t.carry ? reachable_loads() : 0xFFFFFFFFu;
        for (int c = 0; c < 4; c++)
            if (cool_t_[c] && now - cool_t_[c] < t.cooldown_ms) { int j = obj(cool_id_[c]); if (j >= 0) reach &= ~(1u << j); }
        allocate(snap, t, plan, reach, &benched_);
        learn_arrivals();
        choose_wait(m);
        uint8_t next = plan[m];
        int old = obj(task);
        // Holding a load: keep it whatever the plan says (the snapshot can lag our own state by a tick).
        if (holding_state(state) && old >= 0 && snap.o[old].status == OBJ_OPEN) next = task;
        if (next == task) return;
        bool finished = task != NONE && (old < 0 || snap.o[old].status == OBJ_DELIVERED);
        task = next; help = NONE; stall_since = 0;
        held_ = gd_set_ = inserting_ = false; misses_ = 0; pair_wait_ = 0;
        if (finished) set_state(ST_BACKOFF, now);          // reverse out of the drop zone first
        else set_state(task == NONE ? ST_IDLE : ST_GOTO, now);
    }

    // Deadlock recovery. A deadlock needs all four of Coffman's conditions (Coffman, Elphick,
    // Shoshani, Computing Surveys 1971): exclusive use (a patch of floor), hold-and-wait (carriers
    // keep their loads), circular wait, and no preemption. On a crowded floor the first three can't
    // be ruled out (measured: dense floors where every robot ends up holding a load, none able to
    // move). So we break the fourth: when every carrier has gone deadlock_ms without progress, the
    // one farthest from its dock (ties: lowest load id) sets its load down and backs off, freeing
    // its patch, and nobody picks that load up for cooldown_ms. Every robot watches the same
    // snapshots, so all agree on the victim without a word. Classic OS recovery by preemption.
    void watch_carriers(uint32_t now) {
        victim_ = NONE;
        int n = 0, stuck = 0; float far = -1;
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            int j = r.alive && (r.state == ST_GRIP || r.state == ST_CARRY) ? obj(r.task) : -1;
            // Progress = the load 30 mm nearer its dock than ever before on this job. Shuffling to
            // and fro isn't progress (measured: one carrier rocking 40 mm hid a deadlock of five).
            float d = j >= 0 ? len(pos(snap.o[j]) - dock_of(snap, snap.o[j])) : 0;
            if (mark_id_[i] != r.id || mark_task_[i] != r.task || j < 0 || !moved_at_[i] || d < best_[i] - 30) { best_[i] = d; moved_at_[i] = now; mark_id_[i] = r.id; mark_task_[i] = r.task; }
            if (j < 0) continue;
            n++;
            if (now - moved_at_[i] < t.deadlock_ms) continue;
            stuck++;
            if (d > far + 1e-3f || (fabsf(d - far) <= 1e-3f && r.task < victim_)) { far = d; victim_ = r.task; }
        }
        if (n < 2 || stuck < n) { victim_ = NONE; return; }
        for (int i = 0; i < snap.nr; i++) moved_at_[i] = now;   // the rest get a fresh deadlock_ms before the next one (best_ kept: rocking still isn't progress)
        int c = 0;
        for (int k = 1; k < 4; k++) if (cool_t_[k] < cool_t_[c]) c = k;   // oldest entry
        cool_id_[c] = victim_; cool_t_[c] = now;
    }

    Heartbeat heartbeat(uint32_t now, uint16_t batt_mv) const {
        return Heartbeat{id, snap.seq, (uint8_t)state, task, help, neighbors(now), batt_mv};
    }

    // Robots on my object, and my slot among them. Slots are ordered by which side of the
    // push line each robot is on, so teammates never have to cross through each other.
    int team(int oj, V2 v, int& my_slot) const {
        int n = 0, m = me(); my_slot = 0;
        V2 op = pos(snap.o[oj]);
        float mine = dot(pos(snap.r[m]) - op, v);
        for (int i = 0; i < snap.nr; i++) {
            if (plan[i] != snap.o[oj].id) continue;
            n++;
            if (i == m) continue;
            float lat = dot(pos(snap.r[i]) - op, v);
            if (lat < mine - 1e-3f || (fabsf(lat - mine) <= 1e-3f && snap.r[i].id < id)) my_slot++;
        }
        return n;
    }
    // Teammates lined up and ready (from their heartbeats), counting myself by my live state.
    // (Carry mode: lined up or further along -- docking, gripping, carrying. `holding`: gripping only.)
    int ready(int oj, bool holding = false, bool moving = false) const {
        auto ok = [&](uint8_t st) {
            if (holding) return st == ST_GRIP || st == ST_CARRY;
            if (moving) return st == ST_CARRY;
            return st == ST_WAIT || st == ST_PUSH || (t.carry && (st == ST_DOCK || st == ST_GRIP || st == ST_CARRY));
        };
        int n = ok(state) ? 1 : 0;
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            if (r.id != id && r.alive && plan[i] == snap.o[oj].id && r.task == snap.o[oj].id && ok(r.state)) n++;
        }
        return n;
    }

    // Carry mode: which loads can a robot actually get to? One flood fill over the floor from every
    // live robot at once (loads, dead robots and walls block it). A load counts if any of its grip
    // spots is reached. Robots then clear a packed area from the outside in, instead of driving into
    // the middle, grabbing a load and finding themselves walled in. Only uses the shared snapshot,
    // so every robot gets the same answer.
    uint32_t reachable_loads() const {
        if (!snap.arena_w) return 0xFFFFFFFFu;
        float cell = fmaxf(50.0f, fmaxf(snap.arena_w / (float)GW, snap.arena_h / (float)GH));
        int w = (int)(snap.arena_w / cell) + 1, h = (int)(snap.arena_h / cell) + 1;
        if (w > GW) w = GW;
        if (h > GH) h = GH;
        float obj_clear = t.robot_radius + t.object_radius - 5;
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                V2 c = {(x + 0.5f) * cell, (y + 0.5f) * cell};
                bool b = c.x < t.robot_radius || c.y < t.robot_radius || c.x > snap.arena_w - t.robot_radius || c.y > snap.arena_h - t.robot_radius;
                for (int j = 0; !b && j < snap.no; j++) b = snap.o[j].status != OBJ_DELIVERED && !carried(j) && len(c - pos(snap.o[j])) < obj_clear + orad(snap.o[j], t) - t.object_radius;
                for (int i = 0; !b && i < snap.nr; i++) b = !snap.r[i].alive && len(c - pos(snap.r[i])) < 2 * t.robot_radius - 10;
                grid_[y * w + x] = b; parent_[y * w + x] = -1;
            }
        int head = 0, tail = 0;
        auto cellof = [&](V2 p) { int cx = (int)(p.x / cell), cy = (int)(p.y / cell); cx = cx < 0 ? 0 : cx >= w ? w - 1 : cx; cy = cy < 0 ? 0 : cy >= h ? h - 1 : cy; return cy * w + cx; };
        for (int i = 0; i < snap.nr; i++)
            if (snap.r[i].alive) { int c = cellof(pos(snap.r[i])); if (parent_[c] < 0) { parent_[c] = (int16_t)c; queue_[tail++] = (int16_t)c; } }
        static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
        while (head < tail) {
            int c = queue_[head++], x = c % w, y = c / w;
            for (int k = 0; k < 4; k++) {
                int nx = x + DX[k], ny = y + DY[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int n = ny * w + nx;
                if (parent_[n] >= 0 || (grid_[n] && !grid_[c])) continue;   // may leave a blocked start cell, not enter one
                parent_[n] = (int16_t)c; queue_[tail++] = (int16_t)n;
            }
        }
        uint32_t mask = 0;
        for (int j = 0; j < snap.no; j++) {
            const SnapObject& o = snap.o[j];
            V2 z = unit(dock_of(snap, o) - pos(o));
            for (int k = 0; k < 8 && !(mask >> j & 1); k++) {
                V2 a = pos(o) - rot(z, k * 0.7853982f) * (grip_reach(t, o) + t.pre_dock);
                if (room_to_turn(snap, a, t) && parent_[cellof(a)] >= 0 && !grid_[cellof(a)]) mask |= 1u << j;
            }
        }
        return mask;
    }

    // ---- path planning: BFS on a coarse occupancy grid of the arena ----------------
    // Walls, every object not yet delivered (including the one we're heading for) and other
    // robots are obstacles, inflated by our radius. Re-planned every tick, so it routes
    // around loads and teammates as they move. Returns the next point to steer at.
    static constexpr int GW = 64, GH = 48;
    mutable uint8_t grid_[GW * GH];
    mutable int16_t parent_[GW * GH];
    mutable int16_t queue_[GW * GH];
    mutable int16_t path_[GW * GH];
    mutable int path_n_ = 0, path_w_ = 1; mutable float path_cell_ = 50;   // last route, for visualisers

    // Plan with a comfortable margin around loads; if that walls us in (crowded arena),
    // re-plan with a tight one -- a possible graze beats standing still forever.
    // `body` > 0: our footprint radius when it's bigger than the chassis (carrying a load);
    // `skip`: an object that isn't an obstacle to us (the one on our gripper).
    // `wall` > 0: clearance from the walls, when it differs (a held load can swing out that far).
    V2 path_step(V2 from, V2 to, int m, int skip = -1, float body = 0, float wall = 0) const {
        bool reached;
        V2 w = route(from, to, m, 15, reached, skip, body, wall);
        return reached ? w : route(from, to, m, -5, reached, skip, body, wall);
    }
    V2 route(V2 from, V2 to, int m, float margin, bool& reached, int skip = -1, float body = 0, float wall = 0) const {
        reached = false;
        if (!snap.arena_w) { reached = true; return to; }
        float cell = fmaxf(50.0f, fmaxf(snap.arena_w / (float)GW, snap.arena_h / (float)GH));
        int w = (int)(snap.arena_w / cell) + 1, h = (int)(snap.arena_h / cell) + 1;
        if (w > GW) w = GW;
        if (h > GH) h = GH;
        auto cx = [&](float x) { int c = (int)(x / cell); return c < 0 ? 0 : c >= w ? w - 1 : c; };
        auto cy = [&](float y) { int c = (int)(y / cell); return c < 0 ? 0 : c >= h ? h - 1 : c; };
        float R = body > 0 ? body : t.robot_radius;
        float obj_clear = R + t.object_radius + margin, rob_clear = R + t.robot_radius - 10;
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                V2 c = {(x + 0.5f) * cell, (y + 0.5f) * cell};
                float Rw = wall > 0 ? wall : R;
                bool b = c.x < Rw || c.y < Rw || c.x > snap.arena_w - Rw || c.y > snap.arena_h - Rw;
                for (int j = 0; !b && j < snap.no; j++)
                    if (j != skip && (snap.o[j].status != OBJ_DELIVERED || t.carry) && len(c - pos(snap.o[j])) < obj_clear) b = true;   // set-down loads are real obstacles
                for (int i = 0; !b && i < snap.nr; i++)
                    if (i != m && !(skip >= 0 && plan[i] == snap.o[skip].id && holding_state(snap.r[i].state)) && len(c - pos(snap.r[i])) < rob_clear) b = true;   // a co-carrier is part of us
                grid_[y * w + x] = b;
                parent_[y * w + x] = -1;
            }
        int s0 = cy(from.y) * w + cx(from.x), g = cy(to.y) * w + cx(to.x);
        grid_[g] = 0;  // the goal is reachable by definition (it's clear of the load)
        int head = 0, tail = 0, best = s0;
        float best_d = len(to - from);
        queue_[tail++] = (int16_t)s0; parent_[s0] = (int16_t)s0;
        static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        while (head < tail && parent_[g] < 0) {
            int c = queue_[head++], x = c % w, y = c / w;
            for (int k = 0; k < 8; k++) {
                int nx = x + DX[k], ny = y + DY[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int n = ny * w + nx;
                if (parent_[n] >= 0 || grid_[n]) continue;
                if (k >= 4 && (grid_[y * w + nx] || grid_[ny * w + x])) continue;  // no corner cutting
                parent_[n] = (int16_t)c; queue_[tail++] = (int16_t)n;
                float dg = len(V2{(nx + 0.5f) * cell, (ny + 0.5f) * cell} - to);
                if (dg < best_d) { best_d = dg; best = n; }
            }
        }
        // Boxed in: head for the reachable cell nearest the goal rather than ploughing through.
        reached = parent_[g] >= 0;
        int end = reached ? g : best;
        if (end == s0 && !reached && grid_[s0]) {
            // We're inside an obstacle's margin with every neighbour blocked too (squeezed between
            // loads): take the shortest way out to free floor, then plan normally from there.
            for (int c = 0; c < w * h; c++) parent_[c] = -1;
            head = tail = 0; queue_[tail++] = (int16_t)s0; parent_[s0] = (int16_t)s0;
            while (head < tail) {
                int c = queue_[head++], x = c % w, y = c / w;
                if (!grid_[c]) return {(x + 0.5f) * cell, (y + 0.5f) * cell};
                for (int k = 0; k < 4; k++) {
                    int nx = x + DX[k], ny = y + DY[k];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h || parent_[ny * w + nx] >= 0) continue;
                    parent_[ny * w + nx] = (int16_t)c; queue_[tail++] = (int16_t)(ny * w + nx);
                }
            }
        }
        if (end == s0) return reached ? to : from;
        int n = 0;
        for (int c = end; c != s0 && n < GW * GH; c = parent_[c]) path_[n++] = (int16_t)c;   // goal ... first step
        path_n_ = n; path_w_ = w; path_cell_ = cell;
        // Aim at the farthest path cell we can reach in a straight line without crossing an
        // inflated obstacle; aiming blindly a few cells ahead cuts corners into loads.
        auto clear_line = [&](V2 a, V2 b) {
            float L = len(b - a);
            for (float d = cell; d < L; d += cell * 0.4f) {
                V2 q = a + (b - a) * (d / L);
                if (grid_[cy(q.y) * w + cx(q.x)]) return false;
            }
            return true;
        };
        int pick = n - 1;
        for (int i = n - 1; i >= 0 && i >= n - 1 - 10; i--) {
            V2 c = {(path_[i] % w + 0.5f) * cell, (path_[i] / w + 0.5f) * cell};
            if (!clear_line(from, c)) break;
            pick = i;
        }
        if (pick == 0 && end == g) return to;
        return {(path_[pick] % w + 0.5f) * cell, (path_[pick] / w + 0.5f) * cell};
    }

    bool teammate_arriving(int oj) const {
        V2 op = pos(snap.o[oj]);
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            if (r.id == id || !r.alive || plan[i] != snap.o[oj].id) continue;
            if (r.state != ST_WAIT && r.state != ST_PUSH && len(pos(r) - op) < t.approach_back + 250) return true;
        }
        return false;
    }

    // Steer toward `goal`, bending away from other robots and walls (objects are the
    // planner's job). Returns a unit heading.
    V2 steer(V2 me_p, V2 goal, int oj_target, bool pushing, bool holding = false) const {
        if (len(goal - me_p) < 1) return {0, 0};   // planner boxed in: hold still (unit() would say "east")
        V2 d = unit(goal - me_p);
        auto repel = [&](V2 p, float r_extra, float gain) {
            float R = t.avoid_radius + r_extra, dist = len(me_p - p);
            if (dist < R && dist > 1e-3f) {
                V2 away = (me_p - p) * (1.0f / dist);
                d = d + rot(away, 1.0f) * ((R - dist) / R * gain);  // swerve with a fixed handedness so head-on pairs pass
            }
        };
        int m = me();
        for (int i = 0; i < snap.nr; i++) {
            if (i == m) continue;
            bool teammate = oj_target >= 0 && plan[i] == snap.o[oj_target].id;
            if (teammate && holding) continue;                // carrying together: we're one rigid vehicle
            if (teammate && pushing) {                      // side-by-side pushers are meant to be close, not touching
                float d_ = len(me_p - pos(snap.r[i])), R = 2 * t.robot_radius + 10;
                if (d_ < R && d_ > 1e-3f) d = d + (me_p - pos(snap.r[i])) * ((R - d_) / R * 1.5f / d_);
                continue;
            }
            repel(pos(snap.r[i]), teammate ? 0 : t.robot_radius, pushing ? 0.8f : 1.6f);
        }
        // Never shove your own load while driving round to line up behind it. A robot that
        // just lost its push line starts out touching it; ploughing on pushes it into walls.
        if (!pushing && !holding && oj_target >= 0) {
            V2 to = pos(snap.o[oj_target]) - me_p; float dist = len(to);
            if (dist < t.robot_radius + t.object_radius && dist > 1e-3f) {
                to = to * (1.0f / dist);
                float into = dot(d, to);
                if (into > 0) { d = d - to * into; if (len(d) < 0.2f) d = rot(to, 1.57f); }   // head-on: go round, fixed side
            }
        }
        // Carry mode: a robot never knocks a load -- neither with its chassis nor with the load on
        // its gripper. Touching one, slide round it. (Docking onto a load is driven straight, not steered.)
        if (t.carry)
            for (int j = 0; j < snap.no; j++) {
                if (holding && j == oj_target) continue;
                auto keep_off = [&](V2 from, float reach) {
                    V2 to = pos(snap.o[j]) - from; float dist = len(to);
                    if (dist > reach || dist < 1e-3f) return;
                    to = to * (1.0f / dist);
                    float into = dot(d, to);
                    if (into > 0) {
                        d = d - to * into;
                        if (len(d) < 0.2f) { V2 side = rot(to, 1.57f); d = dot(side, goal - me_p) >= 0 ? side : side * -1.0f; }   // head-on: go round on the goal's side
                    }
                };
                keep_off(me_p, t.robot_radius + t.object_radius + 15);
                if (holding && oj_target >= 0) keep_off(pos(snap.o[oj_target]), 2 * t.object_radius + 15);
            }
        if (holding && oj_target >= 0 && snap.arena_w) {   // and never scrape the held load along a wall
            V2 L = pos(snap.o[oj_target]); float m = t.object_radius + 15;
            if (L.x < m && d.x < 0) d.x = 0;
            if (L.y < m && d.y < 0) d.y = 0;
            if (L.x > snap.arena_w - m && d.x > 0) d.x = 0;
            if (L.y > snap.arena_h - m && d.y > 0) d.y = 0;
        }
        if (len(d) < 1e-3f) return {0, 0};
        float m_ = t.robot_radius + 15;
        if (snap.arena_w) {
            if (me_p.x < m_) d.x += (m_ - me_p.x) / m_ * 2;
            if (me_p.x > snap.arena_w - m_) d.x -= (me_p.x - (snap.arena_w - m_)) / m_ * 2;
            if (me_p.y < m_) d.y += (m_ - me_p.y) / m_ * 2;
            if (me_p.y > snap.arena_h - m_) d.y -= (me_p.y - (snap.arena_h - m_)) / m_ * 2;
        }
        return unit(d);
    }

    // Unicycle controller -> wheel commands in [-1, 1].
    void drive(V2 dir, float speed, float th, float& l, float& r) const {
        if (len(dir) < 1e-3f) { l = r = 0; return; }
        float e = wrap(atan2f(dir.y, dir.x) - th), v, w;
        if (fabsf(e) > t.turn_in_place) { v = 0; w = (e > 0 ? 1 : -1) * t.max_turn * 0.6f; }
        else { v = speed * cosf(e); w = clampf(t.k_turn * e, -t.max_turn, t.max_turn); }
        l = (v - w * t.wheel_base / 2) / t.vmax;
        r = (v + w * t.wheel_base / 2) / t.vmax;
        float big = fmaxf(fabsf(l), fabsf(r));
        if (big > 1) { l /= big; r /= big; }
    }
    void turn_to(float heading, float th, float& l, float& r) const {
        float e = wrap(heading - th), w = clampf(t.k_turn * e, -t.max_turn, t.max_turn);
        l = -w * t.wheel_base / 2 / t.vmax; r = -l;
    }
    // drive(), but backing up when the goal is behind: with a load on the gripper a robot can't
    // spin round next to a wall, so it reverses out instead.
    void drive_rev(V2 dir, float speed, float th, float& l, float& r) const {
        if (len(dir) < 1e-3f) { l = r = 0; return; }
        float e = wrap(atan2f(dir.y, dir.x) - th);
        if (fabsf(e) <= 1.75f) { drive(dir, speed, th, l, r); return; }
        float er = wrap(e - 3.14159265f), w = clampf(t.k_turn * er, -t.max_turn, t.max_turn), v = -speed * cosf(er);
        l = (v - w * t.wheel_base / 2) / t.vmax;
        r = (v + w * t.wheel_base / 2) / t.vmax;
        float big = fmaxf(fabsf(l), fabsf(r));
        if (big > 1) { l /= big; r /= big; }
    }

    // ---- carry mode ------------------------------------------------------------------
    bool carried(int j) const {   // on someone's gripper (from the robots' reported states)
        for (int i = 0; i < snap.nr; i++)
            if (snap.r[i].alive && snap.r[i].task == snap.o[j].id && (snap.r[i].state == ST_GRIP || snap.r[i].state == ST_CARRY)) return true;
        return false;
    }
    // My slot in my load's dock. Robots carrying to the same dock are ranked by load id and take
    // the free slots in order, so every robot computes the same assignment. False = dock full.
    bool my_slot(int oj, V2& slot, V2& in) const {
        if (!snap.nz) return false;
        int k = snap.o[oj].kind % snap.nz;
        V2 S[8], I[8];
        int n = dock_slots(snap, k, t, S, I), rank = 0;
        bool busy[8] = {};
        float reach = grip_reach(t);
        for (int q = 0; q < n; q++) {
            // Taken, or its approach lane is blocked by a load left lying there or a dead robot.
            V2 a = S[q] - I[q] * (reach + t.pre_dock), b = S[q] - I[q] * reach;
            for (int j = 0; j < snap.no; j++) {
                if (j == oj || carried(j)) continue;
                if (len(pos(snap.o[j]) - S[q]) < 2 * t.object_radius - 10 ||
                    seg_dist(pos(snap.o[j]), a, b) < t.carry_body + t.object_radius) busy[q] = true;
            }
            for (int i = 0; i < snap.nr; i++)
                if (!snap.r[i].alive && seg_dist(pos(snap.r[i]), a, b) < t.carry_body + t.robot_radius) busy[q] = true;
        }
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            if (r.id == id || !r.alive || (r.state != ST_GRIP && r.state != ST_CARRY)) continue;
            int j = obj(r.task);
            if (j >= 0 && snap.o[j].kind % snap.nz == k && r.task < snap.o[oj].id) rank++;
        }
        for (int q = 0; q < n; q++) {
            if (busy[q]) continue;
            if (rank-- == 0) { slot = S[q]; in = I[q]; return true; }
        }
        return false;
    }

    // Two robots gripping one load side by side are one wide two-wheeled vehicle: each turns the
    // vehicle's (v, w) into its own wheel speeds from where it sits across it (y = its offset to the
    // left of the centre line). Both scale by the same factor, so the pair stays rigid.
    // The pair model only holds if my partner really is beside me: same heading, one spacing across.
    bool partner_beside(int oj, V2 p, float th) const {
        V2 left = {-sinf(th), cosf(th)};
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            if (r.id == id || !r.alive || r.task != snap.o[oj].id || !holding_state(r.state)) continue;
            float across = dot(pos(r) - p, left);
            return fabsf(wrap(r.th / 1000.0f - th)) < 0.35f && fabsf(fabsf(across) - t.team_spacing) < 40 && fabsf(dot(pos(r) - p, V2{cosf(th), sinf(th)})) < 40;
        }
        return false;
    }
    void pair_wheels(float y, float& l, float& r) const {
        float v = (l + r) * 0.5f * t.vmax, w = (r - l) * t.vmax / t.wheel_base, big = 1;
        for (int k = 0; k < 2; k++) {
            float vi = v - w * (k ? -y : y);
            big = fmaxf(big, fmaxf(fabsf(vi - w * t.wheel_base / 2), fabsf(vi + w * t.wheel_base / 2)) / t.vmax);
        }
        float vi = (v - w * y) / big, wi = w / big;
        l = (vi - wi * t.wheel_base / 2) / t.vmax;
        r = (vi + wi * t.wheel_base / 2) / t.vmax;
    }

    void step_carry(uint32_t now, V2 p, float th, int oj, float& l, float& r) {
        const SnapObject& o = snap.o[oj];
        int need = o.demand ? o.demand : 1;
        bool pair = need >= 2;
        float half = pair ? t.team_spacing * 0.5f : 0, reach = grip_reach(t, o);
        V2 op = pos(o), hd = {cosf(th), sinf(th)};
        if (help == o.id && o.demand > help_demand_) help = NONE;   // the gateway heard us: more robots are coming
        if (!holding_state(state)) {   // keep the chosen side while it stays usable, so we don't dither on the way
            V2 a = op - gd_ * (reach + t.pre_dock);
            // (A pair recomputes every tick from the shared snapshot -- latching privately, the two could pick different faces.)
            if (pair || !gd_set_ || gd_need_ != need || len(op - gd_at_) > 60 || !room_to_turn(snap, a, t) || !spot_free(snap, o, a, t)) {
                gd_ = grip_dir(snap, o, pair ? op : p, t, half); gd_at_ = op; gd_set_ = true; gd_need_ = (uint8_t)need;
            }
        }
        // My place across the load's face: centred alone, or left/right of my partner.
        V2 left = {-gd_.y, gd_.x};
        float off = 0;
        if (pair) { int slot, n = team(oj, left, slot); off = (slot - (n - 1) * 0.5f) * t.team_spacing; }
        V2 a = op - gd_ * (reach + t.pre_dock) + left * off;
        switch (state) {
            case ST_IDLE: set_state(ST_GOTO, now); break;
            case ST_GOTO: {
                if (len(a - p) < t.arrive_tol) { set_state(ST_ALIGN, now); break; }
                V2 goal = path_step(p, a, me());
                drive(steer(p, goal, oj, false), fminf(t.cruise, 1.5f * len(a - p) + 60), th, l, r);
                break;
            }
            case ST_ALIGN: {
                if (len(a - p) > 2.5f * t.arrive_tol) { set_state(ST_GOTO, now); break; }
                float h = atan2f(gd_.y, gd_.x);
                if (fabsf(wrap(h - th)) < t.align_tol) { set_state(pair ? ST_WAIT : ST_DOCK, now); break; }
                turn_to(h, th, l, r);
                break;
            }
            case ST_WAIT:   // a pair docks together -- each exactly on its own spot, or it blocks its partner's
                if (len(a - p) > 1.2f * t.arrive_tol) { set_state(ST_GOTO, now); break; }
                if (fabsf(wrap(atan2f(gd_.y, gd_.x) - th)) > 2 * t.align_tol) { set_state(ST_ALIGN, now); break; }
                if (ready(oj) >= need) set_state(ST_DOCK, now);
                break;
            case ST_DOCK: {   // straight in onto the load, slowly
                V2 g = op - gd_ * reach + left * off, rel = g - p;
                float ahead = dot(rel, gd_), lat = dot(rel, left);
                // Off the line: start over. A lone robot only has to land on its gripper (it catches
                // +-70 mm and steers in as it goes), so it tolerates more than the 40 mm it arrived
                // within -- at 30 mm it went round this loop ~18 times a parcel. A pair is shoulder to
                // shoulder: 30 mm.
                if (fabsf(lat) > (pair ? 30 : 50) || ahead > t.pre_dock + 60) { set_state(ST_GOTO, now); break; }
                if (ahead < 8) { set_state(ST_GRIP, now); off_ = off; break; }
                // Ease in: the camera is ~0.1 s behind, so arriving fast means touching (and nudging) the load.
                drive(unit(g + gd_ * 60 - p), clampf(30 + 1.5f * ahead, 30, t.dock_speed), th, l, r);
                break;
            }
            case ST_GRIP: {
                grip = true;
                // Once the gripper has closed (both grippers, for a pair): tug -- back up a few cm.
                // A held load follows; a missed one stays put. The camera can't tell holding from
                // pushing while driving forward, so this is the only honest check without a sensor.
                if (pair && now - state_since > t.grip_ms + 2 * t.sync_wait_ms) { grip = gd_set_ = false; set_state(ST_BACKOFF, now); break; }   // partner never closed: line up again
                if (now - state_since < t.grip_ms || (pair && ready(oj, true) < need)) { tug_since_ = 0; break; }
                if (!tug_since_) tug_since_ = now;
                if (now - tug_since_ < 500) { l = r = -t.dock_speed / t.vmax; break; }
                if (now - tug_since_ < 750) break;   // let the camera catch up
                tug_since_ = 0;
                V2 m = p - V2{-hd.y, hd.x} * off_;
                if (len(op - (m + hd * reach)) < 25) { set_state(ST_CARRY, now); held_ = true; inserting_ = false; stall_since = 0; carry_from_ = op; blocked_ = 0; break; }
                grip = gd_set_ = false;   // missed: try again from the top
                if (++misses_ > t.grip_retries) { help = o.id; help_demand_ = o.demand; }
                set_state(ST_GOTO, now);
                break;
            }
            case ST_CARRY: {
                grip = true;
                // It turned out to need two: let go and back off, so we can both line up across its face.
                if (pair && fabsf(off_) < 1) { grip = held_ = gd_set_ = false; set_state(ST_BACKOFF, now); break; }
                if (pair && ready(oj, false, true) < need) {   // partner still tugging, or let go: hold still, but not forever
                    stall_since = 0;
                    if (!pair_wait_) pair_wait_ = now;
                    else if (now - pair_wait_ > 2 * t.sync_wait_ms) { pair_wait_ = 0; grip = held_ = gd_set_ = false; set_state(ST_BACKOFF, now); }   // partner can't join: both line up again
                    break;
                }
                pair_wait_ = 0;
                if (pair && !partner_beside(oj, p, th)) { grip = held_ = false; set_state(ST_BACKOFF, now); break; }   // not side by side: let go, line up again
                // Drive as the vehicle's centre: me, or the midpoint of the pair.
                V2 m = p - V2{-hd.y, hd.x} * off_;
                // Did we really get it? The camera has to show the load riding on the gripper(s).
                if (now - state_since > 600 && len(op - (m + hd * reach)) > t.grip_tol) {
                    grip = held_ = gd_set_ = false;
                    if (++misses_ > t.grip_retries) { help = o.id; help_demand_ = o.demand; }   // can't hold it: let the gateway escalate
                    set_state(ST_GOTO, now);
                    break;
                }
                // Too heavy for the grippers on it: we're driving but not moving. Ask for help.
                // (Only while the wheels are actually driving: waiting for a slot isn't a stall.)
                // A load we've already carried isn't too heavy, it's blocked (e.g. loads in the dock in
                // the way): line up on the slot again instead -- calling for help there got crates
                // flagged as immovable right at the dock (measured: 8 runs). Four times (8 used up the shift); then ask
                // anyway, so a crate wedged for good is flagged rather than freezing the floor.
                if (!stall_since || len(p - stall_at) > t.stall_dist || fabsf(wrap(th - stall_th_)) > 0.15f || cmd_ < 0.05f) { stall_since = now; stall_at = p; stall_th_ = th; }
                else if (now - stall_since > t.stall_ms) {
                    if (len(op - carry_from_) > 100 && blocked_ < 4) { blocked_++; inserting_ = false; stall_since = 0; }
                    else if (help != o.id) { help = o.id; help_demand_ = o.demand; }
                }
                if (!inserting_) {
                    V2 slot, in;
                    if (!my_slot(oj, slot, in)) {
                        // Dock full: queue out of the way. Waiting near it would park us on another
                        // carrier's lane -- and then the slot we're waiting for can never free up.
                        stall_since = 0; staging_ = true;
                        const Zone& dz = snap.z[o.kind % snap.nz];
                        V2 c = {(float)dz.x, (float)dz.y}, q = c + unit(m - c) * (dz.r + reach + t.pre_dock + 250);
                        if (len(m - c) < len(q - c) - 20) {
                            q = keep_in(snap, q, t);
                            drive_rev(steer(m, path_step(m, q, me(), oj, t.carry_body + half, reach + orad(o, t)), oj, false, true), t.carry_speed, th, l, r);
                            if (pair) pair_wheels(off_, l, r);
                        }
                        break;
                    }
                    V2 stage = slot - in * (reach + t.pre_dock);
                    if (len(stage - m) > t.arrive_tol) {
                        V2 goal = path_step(m, stage, me(), oj, t.carry_body + half, reach + orad(o, t));
                        drive_rev(steer(m, goal, oj, false, true), fminf(t.carry_speed, 1.5f * len(stage - m) + 60), th, l, r);
                    } else {
                        float h = atan2f(in.y, in.x);
                        // Square up tightly: slots leave ~15 mm between loads, and 10 deg off swings the load ~20 mm.
                        if (fabsf(wrap(h - th)) > 0.07f) turn_to(h, th, l, r);
                        else { inserting_ = true; slot_ = slot; in_ = in; }
                    }
                }
                // Turning on the spot swings the held load round: if it would clip a loose load, back off first.
                if (!inserting_ && fabsf(l - r) > 0.2f && fabsf(l + r) < 0.3f) {
                    float sweep = (r > l ? 1 : -1) * 0.6f;
                    V2 L2 = m + rot(hd, sweep) * reach;
                    for (int j = 0; j < snap.no; j++)
                        if (j != oj && !carried(j) && len(pos(snap.o[j]) - L2) < orad(snap.o[j], t) + orad(o, t) + 10) { l = r = -0.3f; break; }
                }
                if (inserting_) {
                    float ahead = dot(slot_ - in_ * reach - m, in_);   // straight in, set it on the slot
                    if (ahead < 8) { grip = held_ = false; set_state(ST_PLACE, now); break; }
                    drive(unit(slot_ - m), t.dock_speed, th, l, r);
                }
                if (pair) pair_wheels(off_, l, r);
                break;
            }
            case ST_PLACE:   // load set down; wait for the gateway to see it in the dock
                if (now - state_since > 3000) { gd_set_ = false; set_state(ST_GOTO, now); }   // not counted: pick it up again
                break;
            default: set_state(ST_GOTO, now); break;
        }
    }

    // One control tick. Writes wheel commands in [-1, 1].
    void step(uint32_t now, float& l, float& r) {
        tick(now, l, r);
        float dv = t.ramp * (float)(now - ramp_at_ < 100 ? now - ramp_at_ : 100) / 1000;
        ramp_at_ = now; l = slew(l, wl_, dv); r = slew(r, wr_, dv);
        keep_in_cell(now, l, r);
        wl_ = l; wr_ = r;
        cmd_ = fabsf(l) + fabsf(r);
    }
    // Buffered Voronoi cells (Zhou, Wang, Bandyopadhyay, Schwager, RA-L 2017, Thm 1): if every robot
    // stays on its own side of the half-way line to each other robot, less a safety radius, no two
    // ever touch -- whatever else each one is doing (their Remark 1). Both robots of a pair draw the
    // same line, from the shared snapshot. Between snapshots our own travel (dead reckoning from the
    // wheel commands) counts against our side. Only speed toward a robot is limited, reaching the
    // line no sooner than cell_horizon: turning and moving away never are. Teammates on one load
    // (a pair carrying a crate) are meant to be close. Last thing before the motors, so it holds
    // whatever the states above asked for.
    void keep_in_cell(uint32_t now, float& l, float& r) {
        int m = me();
        float dt = odo_at_ && now > odo_at_ ? (now - odo_at_) / 1000.0f : 0;
        odo_at_ = now;
        if (t.cell_margin <= 0 || m < 0 || !have_snap) return;
        float th = snap.r[m].th / 1000.0f;
        V2 hd = {cosf(th), sinf(th)}, p = pos(snap.r[m]);
        odo_ = odo_ + hd * ((wl_ + wr_) * 0.5f * t.vmax * dt);   // where last tick's command took us
        float v = (l + r) * 0.5f * t.vmax;
        if (fabsf(v) < 1) return;
        V2 dir = v > 0 ? hd : hd * -1.0f;
        float allow = fabsf(v), rs = t.robot_radius + t.cell_margin;
        for (int j = 0; j < snap.nr; j++) {
            if (j == m || (task != NONE && snap.r[j].task == task)) continue;
            V2 to = pos(snap.r[j]) - p;
            float d = len(to);
            if (d < 1) continue;
            float toward = dot(dir, to * (1 / d));
            if (toward < 0.05f) continue;
            float room = d * 0.5f - rs - dot(odo_, to * (1 / d));   // what's left of our side
            allow = fminf(allow, fmaxf(room, 0.0f) / (t.cell_horizon * toward));
        }
        if (allow >= fabsf(v)) return;
        float k = allow / fabsf(v), fwd = (l + r) * 0.5f * k, turn = (r - l) * 0.5f;
        l = fwd - turn; r = fwd + turn;
    }
    // Speeding up is rate-limited; slowing down or stopping never is. A reversal brakes to 0 first.
    static float slew(float want, float was, float dv) {
        if (want * was >= 0 && fabsf(want) <= fabsf(was)) return want;
        float base = want * was < 0 ? 0 : was;
        return base + clampf(want - base, -dv, dv);
    }
    void tick(uint32_t now, float& l, float& r) {
        l = r = 0; staging_ = false;
        grip = held_;   // a held load stays held, even through a stop
        int m = me();
        if (estop || !have_snap || now - last_snap > t.snapshot_timeout_ms || m < 0 || !snap.r[m].alive) {
            set_state(ST_STOPPED, now);
            return;
        }
        if (state == ST_STOPPED) set_state(task == NONE ? ST_IDLE : held_ ? ST_CARRY : ST_GOTO, now);

        const SnapRobot& self = snap.r[m];
        V2 p = pos(self); float th = self.th / 1000.0f;

        if (state == ST_BACKOFF) {
            if (now - state_since < t.backoff_ms) { l = r = -t.push_speed / t.vmax; return; }
            set_state(task == NONE ? ST_IDLE : ST_GOTO, now);
        }
        int oj = obj(task);
        if (task == NONE || oj < 0 || snap.o[oj].status != OBJ_OPEN) {
            grip = held_ = false;
            set_state(ST_IDLE, now);
            // No work: wait beside the busiest arrival spot if we've learned one, else (carry mode)
            // park at home, out of everyone's way.
            V2 rest = has_wait_ ? wait_ : home_;
            if ((t.carry || has_wait_) && len(rest - p) > 60) drive(steer(p, path_step(p, rest, m), -1, false), fminf(t.cruise, 1.5f * len(rest - p) + 60), th, l, r);
            return;
        }
        if (t.carry) { step_carry(now, p, th, oj, l, r); return; }

        const SnapObject& o = snap.o[oj];
        int need = o.demand ? o.demand : 1;
        V2 u = push_dir(snap, o, t), v = {-u.y, u.x}, op = pos(o);
        int slot, n = team(oj, v, slot);
        float off = (slot - (n - 1) * 0.5f) * t.team_spacing;
        V2 a = keep_in(snap, op - u * t.approach_back + v * off, t);
        float heading = atan2f(u.y, u.x);
        float drift = len(p - a);

        switch (state) {
            case ST_IDLE: set_state(ST_GOTO, now);  // fallthrough into GOTO next tick
                break;
            case ST_GOTO: {
                if (drift < t.arrive_tol) { set_state(ST_ALIGN, now); break; }
                V2 goal = path_step(p, a, m);
                drive(steer(p, goal, oj, false), fminf(t.cruise, 1.5f * len(a - p) + 60), th, l, r);
                break;
            }
            case ST_ALIGN:
                if (drift > 2.5f * t.arrive_tol) { set_state(ST_GOTO, now); break; }
                if (fabsf(wrap(heading - th)) < t.align_tol) { set_state(ST_WAIT, now); break; }
                turn_to(heading, th, l, r);
                break;
            case ST_WAIT:
                if (drift > 2.5f * t.arrive_tol) { set_state(ST_GOTO, now); break; }
                if (fabsf(wrap(heading - th)) > 2 * t.align_tol) { set_state(ST_ALIGN, now); break; }
                // Enough of us to move it -- but if a teammate is about to line up, give it
                // a moment so we push together instead of knocking the load off its line.
                if (ready(oj) >= need && (!teammate_arriving(oj) || now - state_since > t.sync_wait_ms)) set_state(ST_PUSH, now);
                break;
            case ST_PUSH: {
                V2 rel = op - p;
                float ahead = dot(rel, u), lateral = dot(rel, v) + off;
                if (ahead < -10 || fabsf(lateral) > t.lost_lateral) { set_state(ST_GOTO, now); break; }
                if (ready(oj) < need) { set_state(ST_WAIT, now); break; }
                // Stall detection only once we're actually touching the load.
                if (ahead < t.contact_dist) {
                    if (!stall_since || len(op - stall_at) > t.stall_dist) { stall_since = now; stall_at = op; }
                    else if (now - stall_since > t.stall_ms) help = o.id;   // too heavy: recruit
                }
                V2 target = op + u * t.push_lead + v * off;
                drive(steer(p, target, oj, true), t.push_speed, th, l, r);
                break;
            }
            default: break;
        }
    }
};

}  // namespace swarm
