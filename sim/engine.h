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

    struct Body { float x, y, th, l = 0, r = 0, gain; bool dead = false; };
    struct Thing { float x, y, r; int weight; uint8_t id; uint8_t kind = 0; bool delivered = false; float delivered_at = -1; };

    float arena_w, arena_h, loss = 0;
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
        brains.emplace_back((uint8_t)bodies.size());
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
                v.o[v.no++] = {t.id, clamp16(t.x + 3 * noise(rng)), clamp16(t.y + 3 * noise(rng)), t.kind};
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
            if (bodies[i].dead) { bodies[i].l = bodies[i].r = 0; continue; }
            brains[i].step(now, bodies[i].l, bodies[i].r);
        }
        physics();

        // delivery (ground truth) and shipping
        for (auto& o : things) {
            const Zone& d = dock_for(o);
            float dx = o.x - d.x, dy = o.y - d.y;
            if (!o.delivered && dx * dx + dy * dy <= (float)d.r * d.r) { o.delivered = true; o.delivered_at = tsec; }
        }
        if (ship_after > 0)
            for (size_t j = 0; j < things.size();) {
                Thing& o = things[j];
                const World::Obj* wo = world_obj(o);
                if (o.delivered && wo && wo->status == OBJ_DELIVERED && tsec - o.delivered_at > ship_after) {
                    for (int i = 0; i < MAX_OBJECTS; i++) if (world.obj[i].used && world.obj[i].id == o.id) world.obj[i].used = false;
                    things.erase(things.begin() + j); shipped++;
                } else j++;
            }
        k++;
    }

    void physics() {
        std::vector<float> vx(bodies.size()), vy(bodies.size());
        for (size_t i = 0; i < bodies.size(); i++) {
            Body& b = bodies[i];
            auto motor = [&](float c) { return fabsf(c) < 0.02f ? 0.0f : (c > 0 ? 1 : -1) * fmaxf(0.0f, fabsf(c) - DEADBAND * 0.5f); };
            float l = motor(b.l) * VMAX_TRUE * b.gain, r = motor(b.r) * VMAX_TRUE * b.gain;
            float v = (l + r) / 2, w = (r - l) / WHEEL_BASE;
            b.th = wrap(b.th + w * DT);
            vx[i] = v * cosf(b.th); vy[i] = v * sinf(b.th);
            b.x += vx[i] * DT; b.y += vy[i] * DT;
        }
        for (auto& o : things) {   // a load moves only if enough robots push it
            std::vector<int> pushers;
            for (size_t i = 0; i < bodies.size(); i++) {
                float dx = o.x - bodies[i].x, dy = o.y - bodies[i].y, d = sqrtf(dx * dx + dy * dy);
                if (d < ROBOT_R + o.r && dx * vx[i] + dy * vy[i] > 0) pushers.push_back((int)i);
            }
            bool moves = (int)pushers.size() >= o.weight;
            for (size_t i = 0; i < bodies.size(); i++) {
                float dx = o.x - bodies[i].x, dy = o.y - bodies[i].y, d = sqrtf(dx * dx + dy * dy);
                float pen = ROBOT_R + o.r - d;
                if (pen <= 0 || d < 1e-3f) continue;
                dx /= d; dy /= d;
                bool is_pusher = false; for (int p : pushers) if (p == (int)i) is_pusher = true;
                if (moves && is_pusher) { o.x += dx * pen; o.y += dy * pen; }
                else { bodies[i].x -= dx * pen; bodies[i].y -= dy * pen; }
            }
        }
        for (size_t a = 0; a < bodies.size(); a++)
            for (size_t b = a + 1; b < bodies.size(); b++) {
                float dx = bodies[b].x - bodies[a].x, dy = bodies[b].y - bodies[a].y, d = sqrtf(dx * dx + dy * dy);
                float pen = 2 * ROBOT_R - d;
                if (pen > 0 && d > 1e-3f) {
                    if (pen > 15) collisions++;
                    dx /= d; dy /= d;
                    bodies[a].x -= dx * pen / 2; bodies[a].y -= dy * pen / 2; bodies[b].x += dx * pen / 2; bodies[b].y += dy * pen / 2;
                }
            }
        for (size_t a = 0; a < things.size(); a++)
            for (size_t b = a + 1; b < things.size(); b++) {
                float dx = things[b].x - things[a].x, dy = things[b].y - things[a].y, d = sqrtf(dx * dx + dy * dy);
                float pen = things[a].r + things[b].r - d;
                if (pen > 0 && d > 1e-3f) { dx /= d; dy /= d; things[a].x -= dx * pen / 2; things[a].y -= dy * pen / 2; things[b].x += dx * pen / 2; things[b].y += dy * pen / 2; }
            }
        for (auto& b : bodies) { b.x = clampf(b.x, ROBOT_R, arena_w - ROBOT_R); b.y = clampf(b.y, ROBOT_R, arena_h - ROBOT_R); }
        for (auto& o : things) { o.x = clampf(o.x, o.r, arena_w - o.r); o.y = clampf(o.y, o.r, arena_h - o.r); }
    }
};

}  // namespace swarm
