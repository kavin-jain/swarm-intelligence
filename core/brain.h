// The robot's brain. Runs on every robot; nothing here is decided centrally.
//
// Each robot receives the same snapshot (positions only), runs the same deterministic
// allocator, and so arrives at the same answer for "who does what" without a leader
// or a negotiation round. It then plans its own motion: line up behind its object,
// wait for teammates if the load needs several robots, push it into the drop zone,
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
    float cruise = 220;          // mm/s
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
inline bool spot_free(const Snapshot& s, const SnapObject& o, V2 a, const Tuning& t) {
    for (int i = 0; i < s.nr; i++)
        if (!s.r[i].alive && len(pos(s.r[i]) - a) < 2 * t.robot_radius + 10) return false;
    for (int j = 0; j < s.no; j++)
        if (s.o[j].id != o.id && s.o[j].status != OBJ_DELIVERED && len(pos(s.o[j]) - a) < t.robot_radius + t.object_radius + 10) return false;
    return true;
}
inline V2 push_dir(const Snapshot& s, const SnapObject& o, const Tuning& t) {
    V2 z = unit(V2{(float)s.zone_x, (float)s.zone_y} - pos(o));
    V2 u = unit(pos(o) - keep_in(s, pos(o) - z * t.approach_back, t));
    for (int k = 0; k <= 8; k++) {
        V2 r = rot(u, (k % 2 ? 1 : -1) * ((k + 1) / 2) * 0.349f);
        V2 a = pos(o) - r * t.approach_back;
        if (len(keep_in(s, a, t) - a) < 1 && spot_free(s, o, a, t)) return r;
    }
    return u;
}

inline float job_cost(const Snapshot& s, const SnapRobot& r, const SnapObject& o, const Tuning& t) {
    V2 a = pos(o) - push_dir(s, o, t) * t.approach_back;
    return len(a - pos(r)) - (r.task == o.id ? t.sticky : 0);
}

// Who does what. Deterministic: the same snapshot gives the same answer on every robot.
// out[i] is the object id for snapshot robot i (or NONE).
//  Pass 1: fully staff whole jobs, cheapest average travel first. A job that needs k
//          robots is only started when k free robots exist (so 5 pencils / 2 robots
//          splits the work, and a heavy box isn't half-staffed while pencils wait).
//  Pass 2: leftover robots go where they help most: lining up at a job that is still
//          short of robots, or joining a job already underway (1 pencil / 2 robots
//          means both push it). At most 2 robots per load: that's how many fit behind one.
inline void allocate(const Snapshot& s, const Tuning& t, uint8_t out[MAX_ROBOTS]) {
    bool free_[MAX_ROBOTS] = {};
    int nfree = 0, team[MAX_OBJECTS] = {};
    for (int i = 0; i < s.nr; i++) { out[i] = NONE; if (s.r[i].alive) { free_[i] = true; nfree++; } }

    for (;;) {
        int best_o = -1; float best_avg = 1e30f; int best_sel[MAX_ROBOTS]; int best_k = 0;
        for (int j = 0; j < s.no; j++) {
            const SnapObject& o = s.o[j];
            int k = o.demand ? o.demand : 1;
            if (o.status != OBJ_OPEN || team[j] || k > nfree) continue;
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
            float avg = sum / k;
            if (avg < best_avg - 1e-3f || (fabsf(avg - best_avg) <= 1e-3f && best_o >= 0 && o.id < s.o[best_o].id)) {
                best_avg = avg; best_o = j; best_k = k;
                for (int n = 0; n < k; n++) best_sel[n] = sel[n];
            }
        }
        if (best_o < 0) break;
        for (int n = 0; n < best_k; n++) { out[best_sel[n]] = s.o[best_o].id; free_[best_sel[n]] = false; }
        team[best_o] = best_k; nfree -= best_k;
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
            if (o.status != OBJ_OPEN || team[j] >= (k >= 2 ? k : 2)) continue;
            float c = job_cost(s, s.r[ri], o, t) + (team[j] >= k ? t.help_penalty : 0);
            if (c < bc - 1e-3f || (fabsf(c - bc) <= 1e-3f && bj >= 0 && o.id < s.o[bj].id)) { bc = c; bj = j; }
        }
        free_[ri] = false; nfree--;
        if (bj >= 0) { out[ri] = s.o[bj].id; team[bj]++; }
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
    struct Peer { uint8_t id; uint32_t t; } peers[MAX_ROBOTS] = {};

    explicit Brain(uint8_t id_, Tuning tune = Tuning()) : id(id_), t(tune) {}

    int me() const { for (int i = 0; i < snap.nr; i++) if (snap.r[i].id == id) return i; return -1; }
    int obj(uint8_t oid) const { for (int j = 0; j < snap.no; j++) if (snap.o[j].id == oid) return j; return -1; }

    void set_state(State s, uint32_t now) {
        if (s == state) return;
        if (state == ST_PUSH) help = NONE;     // only ask for help while actually pushing
        if (s == ST_PUSH) stall_since = 0;
        state = s; state_since = now;
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
        snap = s; have_snap = true; last_snap = now;
        int m = me();
        if (m < 0) return;
        allocate(snap, t, plan);
        uint8_t next = plan[m];
        if (next == task) return;
        int old = obj(task);
        bool finished = task != NONE && (old < 0 || snap.o[old].status == OBJ_DELIVERED);
        task = next; help = NONE; stall_since = 0;
        if (finished) set_state(ST_BACKOFF, now);          // reverse out of the drop zone first
        else set_state(task == NONE ? ST_IDLE : ST_GOTO, now);
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
    int ready(int oj) const {
        int n = (state == ST_WAIT || state == ST_PUSH) ? 1 : 0;
        for (int i = 0; i < snap.nr; i++) {
            const SnapRobot& r = snap.r[i];
            if (r.id != id && r.alive && plan[i] == snap.o[oj].id && r.task == snap.o[oj].id &&
                (r.state == ST_WAIT || r.state == ST_PUSH)) n++;
        }
        return n;
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

    // Plan with a comfortable margin around loads; if that walls us in (crowded arena),
    // re-plan with a tight one -- a possible graze beats standing still forever.
    V2 path_step(V2 from, V2 to, int m) const {
        bool reached;
        V2 w = route(from, to, m, 15, reached);
        return reached ? w : route(from, to, m, -5, reached);
    }
    V2 route(V2 from, V2 to, int m, float margin, bool& reached) const {
        reached = false;
        if (!snap.arena_w) { reached = true; return to; }
        float cell = fmaxf(50.0f, fmaxf(snap.arena_w / (float)GW, snap.arena_h / (float)GH));
        int w = (int)(snap.arena_w / cell) + 1, h = (int)(snap.arena_h / cell) + 1;
        if (w > GW) w = GW;
        if (h > GH) h = GH;
        auto cx = [&](float x) { int c = (int)(x / cell); return c < 0 ? 0 : c >= w ? w - 1 : c; };
        auto cy = [&](float y) { int c = (int)(y / cell); return c < 0 ? 0 : c >= h ? h - 1 : c; };
        float obj_clear = t.robot_radius + t.object_radius + margin, rob_clear = 2 * t.robot_radius - 10;
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                V2 c = {(x + 0.5f) * cell, (y + 0.5f) * cell};
                bool b = c.x < t.robot_radius || c.y < t.robot_radius || c.x > snap.arena_w - t.robot_radius || c.y > snap.arena_h - t.robot_radius;
                for (int j = 0; !b && j < snap.no; j++)
                    if (snap.o[j].status != OBJ_DELIVERED && len(c - pos(snap.o[j])) < obj_clear) b = true;
                for (int i = 0; !b && i < snap.nr; i++)
                    if (i != m && len(c - pos(snap.r[i])) < rob_clear) b = true;
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
        if (end == s0) return reached ? to : from;
        int n = 0;
        for (int c = end; c != s0 && n < GW * GH; c = parent_[c]) path_[n++] = (int16_t)c;   // goal ... first step
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
    V2 steer(V2 me_p, V2 goal, int oj_target, bool pushing) const {
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
            if (teammate && pushing) {                      // side-by-side pushers are meant to be close, not touching
                float d_ = len(me_p - pos(snap.r[i])), R = 2 * t.robot_radius + 10;
                if (d_ < R && d_ > 1e-3f) d = d + (me_p - pos(snap.r[i])) * ((R - d_) / R * 1.5f / d_);
                continue;
            }
            repel(pos(snap.r[i]), teammate ? 0 : t.robot_radius, pushing ? 0.8f : 1.6f);
        }
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

    // One control tick. Writes wheel commands in [-1, 1].
    void step(uint32_t now, float& l, float& r) {
        l = r = 0;
        int m = me();
        if (estop || !have_snap || now - last_snap > t.snapshot_timeout_ms || m < 0 || !snap.r[m].alive) {
            set_state(ST_STOPPED, now);
            return;
        }
        if (state == ST_STOPPED) set_state(task == NONE ? ST_IDLE : ST_GOTO, now);

        const SnapRobot& self = snap.r[m];
        V2 p = pos(self); float th = self.th / 1000.0f;

        if (state == ST_BACKOFF) {
            if (now - state_since < t.backoff_ms) { l = r = -t.push_speed / t.vmax; return; }
            set_state(task == NONE ? ST_IDLE : ST_GOTO, now);
        }
        int oj = obj(task);
        if (task == NONE || oj < 0 || snap.o[oj].status != OBJ_OPEN) { set_state(ST_IDLE, now); return; }

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
