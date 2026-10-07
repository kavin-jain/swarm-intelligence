// Simulation engine shared by the test simulator (sim.cpp) and the browser build
// (web/engine_wasm.cpp). Every robot runs the real core/brain.h and the gateway runs
// the real core/world.h; only physics, camera and radio are simulated here.
#pragma once
#include "brain.h"
#include "world.h"
#include <random>
#include <vector>

namespace swarm {

struct Engine {
    static constexpr float DT = 0.02f;                 // physics + control at 50 Hz
    static constexpr float ROBOT_R = 60, VMAX_TRUE = 300, WHEEL_BASE = 110, DEADBAND = 0.08f;

    struct Body { float x, y, th, l = 0, r = 0, gain; bool dead = false, grip = false, tried = false; int held = -1; float grip_t = 0, gf = 0, gs = 0; };   // held = thing id at (gf ahead, gs left)
    struct Thing { float x, y, r; int weight; uint8_t id; uint8_t kind = 0; bool delivered = false; float delivered_at = -1; };

    float arena_w, arena_h, loss = 0;
    Tuning tune;                      // every robot's brain gets this (tune.carry: gripper fitted or not)
    float grip_miss = 0.05f;          // chance a grip attempt doesn't catch the load
    int wall_drag = 0;                // steps a load scraped along a wall
    int hard_starts = 0;              // ticks a wheel command jumped up by > 0.25 (inrush current spikes)
    float shoved_mm = 0;              // carry mode: how far robots knocked loose loads, in total (should be ~0)
    float shoved_by_state[16] = {};   // ...split by the state of the robot that did it (diagnostics)
    // Energy model, per robot (assumptions -- calibrate on the real robots with a USB power meter):
    //  electronics (ESP32 + radio) 0.5 W; each TT gear motor ~2.5 W at full command, 1.6x that when
    //  the wheels are commanded but held (stall current); an L298N wastes ~25% (its ~2 V drop on a 7.4 V
    //  pack); gripper 3 W while closing, 1 W holding with peak-and-hold (35% duty).
    static constexpr float P_ELEC = 0.5f, P_MOTOR = 2.5f, STALL = 1.6f, DRIVER_EFF = 0.75f, P_GRAB = 3.0f, P_HOLD = 1.05f;
    double energy_j = 0, idle_drive_j = 0;   // idle_drive_j: motor energy spent with no job (parking, waiting spots)
    // Liveness: a robot on a job must keep making progress -- itself nearer its parcel, or the parcel
    // nearer its dock. A stall = less than 30 mm of progress for STALL_S, outside the waits that are by
    // design (staging beside a full dock, setting a parcel down).
    static constexpr float STALL_S = 20;
    int stalls = 0; float stall_max = 0;   // stall episodes longer than STALL_S; longest time without progress
    int stall_state[16] = {};              // ...by the robot's state when the stall was counted
    int stall_cause[5] = {};               // ...by what's next to it then: robot, loose load, its dock, partner, nothing
    struct Watch { int task = -1; float best = 0, since = 0; bool counted = false; };
    std::vector<Watch> watch;
    std::vector<Zone> docks;          // a load of kind k is delivered at docks[k % size]
    float ship_after = 0;            // > 0: delivered loads leave the dock after this many seconds
    std::mt19937 rng;
    std::normal_distribution<float> noise{0, 1};
    std::uniform_real_distribution<float> uni{0, 1};
    std::vector<Body> bodies;
    std::vector<Brain> brains;
    std::vector<Thing> things;
    World world;
    std::vector<Vision> vis_queue;
    std::vector<uint32_t> vis_due;
    uint16_t vseq = 0;
    uint8_t next_obj_id = 1;
    int k = 0, help_events = 0, collisions = 0, shipped = 0;

    Engine(float w, float h, float zx, float zy, float zr, uint32_t seed)
        : arena_w(w), arena_h(h), rng(seed) { add_dock(zx, zy, zr); }

    void add_dock(float x, float y, float r) { if (docks.size() < MAX_ZONES) docks.push_back({(int16_t)x, (int16_t)y, (uint16_t)r}); }
    const Zone& dock_for(const Thing& t) const { return docks[t.kind % docks.size()]; }

    float time() const { return k * DT; }

    int add_robot(float x, float y, float th) {
        if (bodies.size() >= MAX_ROBOTS) return -1;
        bodies.push_back({x, y, th, 0, 0, 1.0f + 0.08f * (uni(rng) - 0.5f) * 2});
        brains.emplace_back((uint8_t)bodies.size(), tune);
        return (int)bodies.size() - 1;
    }
    int add_object(float x, float y, int weight, float radius, uint8_t kind = 0) {
        if (things.size() >= MAX_OBJECTS) return -1;
        for (int tries = 0; tries < 256; tries++, next_obj_id = next_obj_id % 250 + 1) {
            bool taken = World::find(world.obj, MAX_OBJECTS, next_obj_id, false) != nullptr;
            for (auto& t : things) taken = taken || t.id == next_obj_id;
            if (!taken) break;
        }
        things.push_back({x, y, radius, weight, next_obj_id, kind});
        next_obj_id = next_obj_id % 250 + 1;
        return (int)things.size() - 1;
    }
    int thing_index(int id) const { for (size_t j = 0; j < things.size(); j++) if (things[j].id == id) return (int)j; return -1; }
    bool held(const Thing& t) const { for (auto& b : bodies) if (b.held == t.id) return true; return false; }
    const World::Obj* world_obj(const Thing& t) const {
        for (int i = 0; i < MAX_OBJECTS; i++) if (world.obj[i].used && world.obj[i].id == t.id) return &world.obj[i];
        return nullptr;
    }

    // One 20 ms tick: camera, radio, every robot's brain, then physics.
    void step() {
        float tsec = k * DT;
        uint32_t now = (uint32_t)(tsec * 1000) + 1;

        // camera: 15 fps, 3 mm / ~1 deg noise, 80 ms to reach the gateway
        if (k % 3 == 0) {
            Vision v{};
            v.seq = ++vseq; v.nz = (uint8_t)docks.size(); for (int i = 0; i < v.nz; i++) v.z[i] = docks[i];
            v.arena_w = (uint16_t)arena_w; v.arena_h = (uint16_t)arena_h;
            for (size_t i = 0; i < bodies.size(); i++)
                v.r[v.nr++] = {(uint8_t)(i + 1), clamp16(bodies[i].x + 3 * noise(rng)), clamp16(bodies[i].y + 3 * noise(rng)),
                               clamp16(wrap(bodies[i].th + 0.02f * noise(rng)) * 1000)};
            for (auto& t : things)
                v.o[v.no++] = {t.id, clamp16(t.x + 3 * noise(rng)), clamp16(t.y + 3 * noise(rng)), t.kind, (uint8_t)(t.r / 2 + 0.5f)};
            vis_queue.push_back(v); vis_due.push_back(now + 80);
        }
        while (!vis_due.empty() && vis_due.front() <= now) {
            world.on_vision(vis_queue.front(), now);
            vis_queue.erase(vis_queue.begin()); vis_due.erase(vis_due.begin());
            Snapshot s = world.snapshot(now);
            uint8_t buf[MAX_PAYLOAD]; int n = encode(s, buf, sizeof buf);   // through the real wire format
            for (size_t i = 0; i < brains.size(); i++) {
                if (bodies[i].dead || uni(rng) < loss) continue;
                Snapshot rx; if (decode(buf, n, rx)) brains[i].on_snapshot(rx, now);
            }
        }

        // heartbeats: 10 Hz, broadcast to gateway and every other robot
        if (k % 5 == 0)
            for (size_t i = 0; i < brains.size(); i++) {
                if (bodies[i].dead) continue;
                Heartbeat h = brains[i].heartbeat(now, 7600);
                uint8_t buf[32]; int n = encode(h, buf, sizeof buf); Heartbeat rx; decode(buf, n, rx);
                if (uni(rng) >= loss) {
                    uint8_t before[MAX_OBJECTS]; for (int j = 0; j < MAX_OBJECTS; j++) before[j] = world.obj[j].demand;
                    world.on_heartbeat(rx, now);
                    for (int j = 0; j < MAX_OBJECTS; j++) if (world.obj[j].used && world.obj[j].demand > before[j]) help_events++;
                }
                for (size_t q = 0; q < brains.size(); q++)
                    if (q != i && !bodies[q].dead && uni(rng) >= loss) brains[q].on_heartbeat(rx, now);
            }

        for (size_t i = 0; i < brains.size(); i++) {
            if (bodies[i].dead) { bodies[i].l = bodies[i].r = 0; bodies[i].grip = false; continue; }   // flat battery: the magnet lets go
            float pl = bodies[i].l, pr = bodies[i].r;
            brains[i].step(now, bodies[i].l, bodies[i].r);
            for (float d : {fabsf(bodies[i].l) - (bodies[i].l * pl > 0 ? fabsf(pl) : 0), fabsf(bodies[i].r) - (bodies[i].r * pr > 0 ? fabsf(pr) : 0)}) hard_starts += d > 0.25f;
            bodies[i].grip = brains[i].grip;
        }
        physics();

        // delivery (ground truth) and shipping
        for (auto& o : things) {
            const Zone& d = dock_for(o);
            float dx = o.x - d.x, dy = o.y - d.y;
            if (!o.delivered && !held(o) && dx * dx + dy * dy <= (float)d.r * d.r) { o.delivered = true; o.delivered_at = tsec; }
        }
        if (ship_after > 0)
            for (size_t j = 0; j < things.size();) {
                Thing& o = things[j];
                const World::Obj* wo = world_obj(o);
                if (o.delivered && wo && wo->status == OBJ_DELIVERED && tsec - o.delivered_at > ship_after) {
                    things.erase(things.begin() + j); shipped++;   // the gateway finds out the real way: the camera stops seeing it
                } else j++;
            }
        k++;
    }

    void physics() {
        size_t nb = bodies.size(), nt = things.size();
        // Grippers: a gripper bar across the robot's front catches a load touching it (within 25 mm)
        // after 0.25 s, or misses. Two robots can hold one load side by side.
        for (auto& b : bodies) {
            if (!b.grip) { b.held = -1; b.grip_t = 0; b.tried = false; continue; }
            if (b.held >= 0 || b.tried || (b.grip_t += DT) < 0.25f) continue;
            b.tried = true;
            float c = cosf(b.th), sn = sinf(b.th), bd = 1e9f, bs = 0;
            int best = -1;
            for (size_t j = 0; j < nt; j++) {
                float dx = things[j].x - b.x, dy = things[j].y - b.y, f = dx * c + dy * sn, sd = -dx * sn + dy * c;   // load ahead / to the left
                float e = fabsf(f - (ROBOT_R + things[j].r));
                if (e < 25 && fabsf(sd) < ROBOT_R + 10 && e + fabsf(sd) < bd) { bd = e + fabsf(sd); best = (int)j; bs = sd; }
            }
            if (best >= 0 && uni(rng) >= grip_miss) { b.held = things[best].id; b.gf = ROBOT_R + things[best].r; b.gs = bs; }
        }
        std::vector<int> holders(nt, 0), hj(nb, -1), first(nt, -1);
        for (size_t i = 0; i < nb; i++)
            if (bodies[i].held >= 0 && (hj[i] = thing_index(bodies[i].held)) >= 0) { holders[hj[i]]++; if (first[hj[i]] < 0) first[hj[i]] = (int)i; }
        // A held load rides rigidly on its gripper(s): the first holder places it, any partner sits on it.
        auto sync = [&](int j) {
            Thing& t = things[j]; const Body& f = bodies[first[j]];
            t.x = f.x + cosf(f.th) * f.gf - sinf(f.th) * f.gs; t.y = f.y + sinf(f.th) * f.gf + cosf(f.th) * f.gs;
            for (size_t i = 0; i < nb; i++) {
                if (hj[i] != j || (int)i == first[j]) continue;
                Body& b = bodies[i];
                b.x = t.x - (cosf(b.th) * b.gf - sinf(b.th) * b.gs); b.y = t.y - (sinf(b.th) * b.gf + cosf(b.th) * b.gs);
            }
        };

        std::vector<float> vx(nb), vy(nb), ox(nt), oy(nt), vc(nb), wc(nb);
        for (size_t j = 0; j < nt; j++) { ox[j] = things[j].x; oy[j] = things[j].y; }
        for (size_t i = 0; i < nb; i++) {
            Body& b = bodies[i];
            auto motor = [&](float c) { return fabsf(c) < 0.02f ? 0.0f : (c > 0 ? 1 : -1) * fmaxf(0.0f, fabsf(c) - DEADBAND * 0.5f); };
            float l = motor(b.l) * VMAX_TRUE * b.gain, r = motor(b.r) * VMAX_TRUE * b.gain;
            vc[i] = (l + r) / 2; wc[i] = (r - l) / WHEEL_BASE;
            if (hj[i] >= 0 && things[hj[i]].weight > holders[hj[i]]) vc[i] = wc[i] = 0;   // too heavy for the grippers on it: wheels slip
            if (!b.dead) {
                float cmd = fabsf(b.l) + fabsf(b.r), slip = (cmd > 0.05f && vc[i] == 0 && wc[i] == 0) ? STALL : 1;
                energy_j += (P_ELEC + P_MOTOR * fminf(cmd, 2.0f) * slip / DRIVER_EFF + (b.grip ? (b.held >= 0 ? P_HOLD : P_GRAB) : 0)) * DT;
                if (brains[i].state == ST_IDLE) idle_drive_j += P_MOTOR * fminf(cmd, 2.0f) / DRIVER_EFF * DT;
            }
        }
        std::vector<bool> done(nb, false);
        for (size_t j = 0; j < nt; j++) {   // a load held by two: one rigid vehicle, moving as their average
            if (holders[j] < 2) continue;
            float mx = 0, my = 0, W = 0, V = 0; int n = 0;
            for (size_t i = 0; i < nb; i++) if (hj[i] == (int)j) { mx += bodies[i].x; my += bodies[i].y; W += wc[i]; n++; }
            mx /= n; my /= n; W /= n;
            const Body& f = bodies[first[j]];
            float hx = cosf(f.th), hy = sinf(f.th);
            for (size_t i = 0; i < nb; i++) if (hj[i] == (int)j) V += vc[i] + W * ((bodies[i].x - mx) * -hy + (bodies[i].y - my) * hx);
            V /= n;
            float da = W * DT, ca = cosf(da), sa = sinf(da), tx = hx * V * DT, ty = hy * V * DT;
            for (size_t i = 0; i < nb; i++) {
                if (hj[i] != (int)j) continue;
                Body& b = bodies[i];
                float rx = b.x - mx, ry = b.y - my, px = b.x, py = b.y;
                b.x = mx + tx + rx * ca - ry * sa; b.y = my + ty + rx * sa + ry * ca; b.th = wrap(b.th + da);
                vx[i] = (b.x - px) / DT; vy[i] = (b.y - py) / DT; done[i] = true;
            }
        }
        for (size_t i = 0; i < nb; i++) {
            if (done[i]) continue;
            Body& b = bodies[i];
            b.th = wrap(b.th + wc[i] * DT);
            vx[i] = vc[i] * cosf(b.th); vy[i] = vc[i] * sinf(b.th);
            b.x += vx[i] * DT; b.y += vy[i] * DT;
        }
        for (size_t j = 0; j < nt; j++) if (holders[j]) sync((int)j);
        for (size_t j = 0; j < nt; j++) {
            Thing& o = things[j];
            if (holders[j]) {   // a held load is part of its robot: anyone else gets pushed out of it
                for (size_t i = 0; i < nb; i++) {
                    if (hj[i] == (int)j) continue;
                    float dx = o.x - bodies[i].x, dy = o.y - bodies[i].y, d = sqrtf(dx * dx + dy * dy), pen = ROBOT_R + o.r - d;
                    if (pen > 0 && d > 1e-3f) { bodies[i].x -= dx / d * pen; bodies[i].y -= dy / d * pen; }
                }
                continue;
            }
            // a loose load moves only if enough robots push it
            std::vector<int> pushers;
            for (size_t i = 0; i < nb; i++) {
                float dx = o.x - bodies[i].x, dy = o.y - bodies[i].y, d = sqrtf(dx * dx + dy * dy);
                if (d < ROBOT_R + o.r && dx * vx[i] + dy * vy[i] > 0) pushers.push_back((int)i);
            }
            bool moves = (int)pushers.size() >= o.weight;
            for (size_t i = 0; i < nb; i++) {
                float dx = o.x - bodies[i].x, dy = o.y - bodies[i].y, d = sqrtf(dx * dx + dy * dy);
                float pen = ROBOT_R + o.r - d;
                if (pen <= 0 || d < 1e-3f) continue;
                dx /= d; dy /= d;
                bool is_pusher = false; for (int p : pushers) if (p == (int)i) is_pusher = true;
                if (moves && is_pusher) { o.x += dx * pen; o.y += dy * pen; if (tune.carry) { shoved_mm += pen; shoved_by_state[brains[i].state & 15] += pen; } }
                else { bodies[i].x -= dx * pen; bodies[i].y -= dy * pen; }
            }
        }
        for (size_t a = 0; a < nb; a++)
            for (size_t b = a + 1; b < nb; b++) {
                if (hj[a] >= 0 && hj[a] == hj[b]) continue;   // co-carriers are bolted together through the load
                float dx = bodies[b].x - bodies[a].x, dy = bodies[b].y - bodies[a].y, d = sqrtf(dx * dx + dy * dy);
                float pen = 2 * ROBOT_R - d;
                if (pen > 0 && d > 1e-3f) {
                    if (pen > 15) collisions++;
                    dx /= d; dy /= d;
                    bodies[a].x -= dx * pen / 2; bodies[a].y -= dy * pen / 2; bodies[b].x += dx * pen / 2; bodies[b].y += dy * pen / 2;
                }
            }
        for (size_t a = 0; a < nt; a++)
            for (size_t b = a + 1; b < nt; b++) {
                if (holders[a] && holders[b]) continue;
                float dx = things[b].x - things[a].x, dy = things[b].y - things[a].y, d = sqrtf(dx * dx + dy * dy);
                float pen = things[a].r + things[b].r - d;
                if (pen <= 0 || d < 1e-3f) continue;
                dx /= d; dy /= d;
                float ka = holders[a] ? 0 : holders[b] ? 1 : 0.5f, kb = 1 - ka;   // a held load doesn't give way
                if (tune.carry && (holders[a] || holders[b])) { shoved_mm += pen; shoved_by_state[15] += pen; }   // a carried load knocked a loose one (bin 15)
                things[a].x -= dx * pen * ka; things[a].y -= dy * pen * ka; things[b].x += dx * pen * kb; things[b].y += dy * pen * kb;
            }
        // walls: a held load that hits one stops its robot(s) too
        for (size_t j = 0; j < nt; j++) {
            if (!holders[j]) continue;
            Thing& o = things[j];
            float cx = clampf(o.x, o.r, arena_w - o.r) - o.x, cy = clampf(o.y, o.r, arena_h - o.r) - o.y;
            for (size_t i = 0; i < nb; i++) if (hj[i] == (int)j) { bodies[i].x += cx; bodies[i].y += cy; }
        }
        for (auto& b : bodies) { b.x = clampf(b.x, ROBOT_R, arena_w - ROBOT_R); b.y = clampf(b.y, ROBOT_R, arena_h - ROBOT_R); }
        for (size_t j = 0; j < nt; j++) if (holders[j]) sync((int)j);
        for (size_t j = 0; j < nt; j++) {
            Thing& o = things[j];
            float cx = clampf(o.x, o.r, arena_w - o.r), cy = clampf(o.y, o.r, arena_h - o.r);
            bool touching = cx != o.x || cy != o.y || o.x - o.r < 1 || o.y - o.r < 1 || o.x + o.r > arena_w - 1 || o.y + o.r > arena_h - 1;
            o.x = cx; o.y = cy;
            if (touching && hypotf(o.x - ox[j], o.y - oy[j]) > 0.5f) wall_drag++;   // scraping along a wall
        }
        watch_progress(k * DT);
    }
    // What a stalled robot is up against: 0 another robot within 250 mm (not its partner), 1 a loose
    // load within 200 mm, 2 within 300 mm of its load's dock, 3 a partner it's waiting on, 4 none of these.
    int cause(size_t i, const Thing& t) const {
        const Body& me = bodies[i];
        for (size_t q = 0; q < bodies.size(); q++)
            if (q != i && !(brains[q].task == brains[i].task && t.weight >= 2) && hypotf(bodies[q].x - me.x, bodies[q].y - me.y) < 250) return 0;
        for (auto& o : things)
            if (o.id != t.id && !o.delivered && hypotf(o.x - me.x, o.y - me.y) < 200) return 1;
        const Zone& d = dock_for(t);
        if (hypotf(d.x - me.x, d.y - me.y) < d.r + 300) return 2;
        if (t.weight >= 2) return 3;
        return 4;
    }
    void watch_progress(float tsec) {
        watch.resize(brains.size());
        for (size_t i = 0; i < brains.size(); i++) {
            Watch& w = watch[i]; const Brain& b = brains[i];
            int ti = b.task == NONE ? -1 : thing_index(b.task);
            if (bodies[i].dead || ti < 0 || b.staging_ || b.state == ST_PLACE) { w.task = -1; continue; }
            const Thing& t = things[ti]; const Zone& d = dock_for(t);
            float phi = bodies[i].held == t.id ? hypotf(t.x - d.x, t.y - d.y) : hypotf(t.x - bodies[i].x, t.y - bodies[i].y);
            if (w.task != b.task || phi < w.best - 30) { w.task = b.task; w.best = phi; w.since = tsec; w.counted = false; continue; }
            float st = tsec - w.since;
            if (st > stall_max) stall_max = st;
            if (st > STALL_S && !w.counted) { stalls++; stall_state[b.state & 15]++; stall_cause[cause(i, t)]++; w.counted = true; }
        }
    }
};

}  // namespace swarm
