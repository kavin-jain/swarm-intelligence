// Host simulator. Every robot runs the real core/brain.h and the gateway runs the real
// core/world.h; only the physics, the camera and the radio are simulated.
//
//   c++ -std=c++17 -O2 -Wall -I core sim/sim.cpp -o build/sim
//   build/sim                 run every scenario, exit 1 if any check fails
//   build/sim --trace DIR     also write DIR/<scenario>.json replays for the web viewer
#include "brain.h"
#include "world.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace swarm;

struct RobotSpec { float x, y, th; };
struct ObjSpec { float x, y; int weight; float radius; };  // weight = robots needed to move it
struct Scenario {
    const char* name; const char* title;
    float arena_w, arena_h, zone_x, zone_y, zone_r;
    std::vector<RobotSpec> robots;
    std::vector<ObjSpec> objects;
    float loss = 0;                        // radio packet loss, both directions
    int kill_robot = 0; float kill_at = 0; // robot id that dies (battery) at time kill_at
    int join_robot = 0; float join_at = 0; // robot id that is switched on late, at join_at
    float duration = 180;
};

struct Result {
    std::vector<float> delivered_at;       // per object, -1 if never
    std::vector<int> credit;               // deliveries credited per robot (in contact at delivery)
    std::vector<int> max_team;             // most robots assigned to an object at once
    std::vector<int> final_demand;
    int help_events = 0, collisions = 0;
    std::vector<int> final_state, final_status;
    std::vector<V2> final_xy;
    float t_end = 0;
};

constexpr float DT = 0.02f;                // physics + control at 50 Hz
constexpr float ROBOT_R = 60, VMAX_TRUE = 300, WHEEL_BASE = 110, DEADBAND = 0.08f;

static Result run(const Scenario& sc, FILE* trace, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0, 1);
    std::uniform_real_distribution<float> uni(0, 1);

    struct Body { float x, y, th, l = 0, r = 0, gain; bool dead = false; };
    struct Thing { float x, y, r; int weight; bool delivered = false; };
    std::vector<Body> bodies;
    std::vector<Thing> things;
    std::vector<Brain> brains;
    for (size_t i = 0; i < sc.robots.size(); i++) {
        bodies.push_back({sc.robots[i].x, sc.robots[i].y, sc.robots[i].th, 0, 0, 1.0f + 0.08f * (uni(rng) - 0.5f) * 2});
        brains.emplace_back((uint8_t)(i + 1));
    }
    for (auto& o : sc.objects) things.push_back({o.x, o.y, o.radius, o.weight});

    World world;
    Result res;
    res.delivered_at.assign(things.size(), -1);
    res.credit.assign(bodies.size(), 0);
    res.max_team.assign(things.size(), 0);
    std::vector<Vision> vis_queue;         // camera latency: frames reach the gateway late
    std::vector<uint32_t> vis_due;
    uint16_t vseq = 0;
    bool first_trace = true;
    if (trace) {
        fprintf(trace, "{\"name\":\"%s\",\"title\":\"%s\",\"arena\":[%.0f,%.0f],\"zone\":[%.0f,%.0f,%.0f],\"robot_r\":%.0f,\"objects\":[",
                sc.name, sc.title, sc.arena_w, sc.arena_h, sc.zone_x, sc.zone_y, sc.zone_r, ROBOT_R);
        for (size_t j = 0; j < things.size(); j++) fprintf(trace, "%s[%.0f,%d]", j ? "," : "", things[j].r, things[j].weight);
        fprintf(trace, "],\"frames\":[");
    }

    int steps = (int)(sc.duration / DT);
    for (int k = 0; k < steps; k++) {
        float tsec = k * DT;
        uint32_t now = (uint32_t)(tsec * 1000) + 1;

        if (sc.kill_robot && tsec >= sc.kill_at) bodies[sc.kill_robot - 1].dead = true;
        if (sc.join_robot) bodies[sc.join_robot - 1].dead = tsec < sc.join_at;

        // ---- camera: 15 fps, 3 mm / ~1 deg noise, 80 ms to reach the gateway ----
        if (k % 3 == 0) {
            Vision v{};
            v.seq = ++vseq; v.zone_x = (int16_t)sc.zone_x; v.zone_y = (int16_t)sc.zone_y; v.zone_r = (uint16_t)sc.zone_r;
            v.arena_w = (uint16_t)sc.arena_w; v.arena_h = (uint16_t)sc.arena_h;
            for (size_t i = 0; i < bodies.size(); i++)
                v.r[v.nr++] = {(uint8_t)(i + 1), clamp16(bodies[i].x + 3 * noise(rng)), clamp16(bodies[i].y + 3 * noise(rng)),
                               clamp16(wrap(bodies[i].th + 0.02f * noise(rng)) * 1000)};
            for (size_t j = 0; j < things.size(); j++)
                v.o[v.no++] = {(uint8_t)(j + 1), clamp16(things[j].x + 3 * noise(rng)), clamp16(things[j].y + 3 * noise(rng))};
            vis_queue.push_back(v); vis_due.push_back(now + 80);
        }
        while (!vis_due.empty() && vis_due.front() <= now) {
            world.on_vision(vis_queue.front(), now);
            vis_queue.erase(vis_queue.begin()); vis_due.erase(vis_due.begin());
            Snapshot s = world.snapshot(now);
            uint8_t buf[MAX_PAYLOAD]; int n = encode(s, buf, sizeof buf);   // go through the real wire format
            for (size_t i = 0; i < brains.size(); i++) {
                if (bodies[i].dead || uni(rng) < sc.loss) continue;
                Snapshot rx; if (decode(buf, n, rx)) brains[i].on_snapshot(rx, now);
            }
        }

        // ---- heartbeats: 10 Hz, broadcast to gateway and every other robot ----
        if (k % 5 == 0)
            for (size_t i = 0; i < brains.size(); i++) {
                if (bodies[i].dead) continue;
                Heartbeat h = brains[i].heartbeat(now, 7600);
                uint8_t buf[32]; int n = encode(h, buf, sizeof buf); Heartbeat rx; decode(buf, n, rx);
                if (uni(rng) >= sc.loss) {
                    uint8_t before[MAX_OBJECTS]; for (int j = 0; j < MAX_OBJECTS; j++) before[j] = world.obj[j].demand;
                    world.on_heartbeat(rx, now);
                    for (int j = 0; j < MAX_OBJECTS; j++) if (world.obj[j].used && world.obj[j].demand > before[j]) res.help_events++;
                }
                for (size_t q = 0; q < brains.size(); q++)
                    if (q != i && !bodies[q].dead && uni(rng) >= sc.loss) brains[q].on_heartbeat(rx, now);
            }

        // ---- control ----
        for (size_t i = 0; i < brains.size(); i++) {
            if (bodies[i].dead) { bodies[i].l = bodies[i].r = 0; continue; }
            brains[i].step(now, bodies[i].l, bodies[i].r);
        }
        for (size_t j = 0; j < things.size(); j++) {
            int n = 0;
            for (auto& b : brains) if (b.task == j + 1) n++;
            if (n > res.max_team[j]) res.max_team[j] = n;
        }

        // ---- physics ----
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
        for (size_t j = 0; j < things.size(); j++) {   // pushing: a load moves only if enough robots push it
            Thing& o = things[j];
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
                    if (pen > 15) res.collisions++;
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
        for (auto& b : bodies) { b.x = clampf(b.x, ROBOT_R, sc.arena_w - ROBOT_R); b.y = clampf(b.y, ROBOT_R, sc.arena_h - ROBOT_R); }
        for (auto& o : things) { o.x = clampf(o.x, o.r, sc.arena_w - o.r); o.y = clampf(o.y, o.r, sc.arena_h - o.r); }

        // ---- delivery bookkeeping (ground truth) ----
        bool all = true;
        for (size_t j = 0; j < things.size(); j++) {
            Thing& o = things[j];
            float dx = o.x - sc.zone_x, dy = o.y - sc.zone_y;
            if (!o.delivered && dx * dx + dy * dy <= sc.zone_r * sc.zone_r) {
                o.delivered = true; res.delivered_at[j] = tsec;
                for (size_t i = 0; i < bodies.size(); i++) {
                    float ex = o.x - bodies[i].x, ey = o.y - bodies[i].y;
                    if (sqrtf(ex * ex + ey * ey) < ROBOT_R + o.r + 25) res.credit[i]++;
                }
            }
            all = all && o.delivered;
        }

        if (trace && k % 5 == 0) {   // 10 fps replay
            fprintf(trace, "%s[%.1f,[", first_trace ? "" : ",", tsec);
            first_trace = false;
            for (size_t i = 0; i < bodies.size(); i++)
                fprintf(trace, "%s[%.0f,%.0f,%.2f,%d,%d,%d,%d]", i ? "," : "", bodies[i].x, bodies[i].y, bodies[i].th,
                        bodies[i].dead ? -1 : (int)brains[i].state, brains[i].task == NONE ? 0 : brains[i].task,
                        brains[i].help == NONE ? 0 : brains[i].help, brains[i].neighbors(now));
            fprintf(trace, "],[");
            for (size_t j = 0; j < things.size(); j++) {
                const World::Obj* wo = World::find(world.obj, MAX_OBJECTS, (uint8_t)(j + 1), false);
                fprintf(trace, "%s[%.0f,%.0f,%d,%d]", j ? "," : "", things[j].x, things[j].y, things[j].delivered ? 1 : (wo && wo->status == OBJ_STUCK ? 2 : 0), wo ? wo->demand : 1);
            }
            fprintf(trace, "]]");
        }
        res.t_end = tsec;
        if (all && tsec > 1) break;
    }
    if (trace) fprintf(trace, "]}\n");
    for (size_t j = 0; j < things.size(); j++) {
        const World::Obj* wo = World::find(world.obj, MAX_OBJECTS, (uint8_t)(j + 1), false);
        res.final_demand.push_back(wo ? wo->demand : 0);
        res.final_status.push_back(wo ? wo->status : 0);
        res.final_xy.push_back({things[j].x, things[j].y});
    }
    for (auto& b : brains) res.final_state.push_back(b.state);
    return res;
}

// ---- scenarios -------------------------------------------------------------------
static std::vector<Scenario> scenarios() {
    std::vector<Scenario> v;
    const float W = 1500, H = 1000, ZX = 1320, ZY = 500, ZR = 160;
    ObjSpec pencil{0, 0, 1, 40}, heavy{0, 0, 2, 60};
    auto at = [](ObjSpec o, float x, float y) { o.x = x; o.y = y; return o; };

    v.push_back({"five_pencils_two_robots", "5 pencils, 2 robots: they split the work", W, H, ZX, ZY, ZR,
                 {{150, 250, 0}, {150, 750, 0}},
                 {at(pencil, 500, 200), at(pencil, 650, 820), at(pencil, 400, 520), at(pencil, 880, 330), at(pencil, 760, 640)}});
    v.push_back({"one_pencil_two_robots", "1 pencil, 2 robots: both push it together", W, H, ZX, ZY, ZR,
                 {{150, 300, 0}, {150, 700, 0}}, {at(pencil, 600, 500)}});
    v.push_back({"heavy_box", "Too heavy for one: it stalls, calls for help, they carry it together", W, H, ZX, ZY, ZR,
                 {{150, 250, 0}, {150, 800, 0}}, {at(heavy, 560, 330), at(pencil, 520, 800)}});
    v.push_back({"robot_failure", "Robot 2 dies mid-run: the others absorb its work", W, H, ZX, ZY, ZR,
                 {{150, 200, 0}, {150, 500, 0}, {150, 800, 0}},
                 {at(pencil, 450, 180), at(pencil, 520, 450), at(pencil, 430, 820), at(pencil, 800, 260), at(pencil, 760, 700), at(pencil, 980, 520)},
                 0, 2, 9});
    Scenario lossy = v[0];
    lossy.name = "radio_loss"; lossy.title = "Same job with 25% of radio packets dropped"; lossy.loss = 0.25f;
    v.push_back(lossy);
    v.push_back({"heavy_alone", "One robot, one heavy box: it tries, then flags the box as stuck instead of grinding", W, H, ZX, ZY, ZR,
                 {{200, 500, 0}}, {at(heavy, 650, 500)}, 0, 0, 0, 0, 0, 40});
    v.push_back({"late_helper", "Box flagged stuck, then a 2nd robot is switched on: it's discovered and they finish the job", W, H, ZX, ZY, ZR,
                 {{200, 500, 0}, {150, 850, 0}}, {at(heavy, 650, 500)}, 0, 0, 0, 2, 25, 120});
    return v;
}

static int fails = 0;
static bool judge(const Scenario& sc, const Result& r, bool verbose);

static bool judge(const Scenario& sc, const Result& r, bool verbose) {
    int before = fails;
    auto check = [&](bool ok, const char* scen, const char* what) {
        if (verbose) printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
        if (!ok) { fails++; if (verbose) fprintf(stderr, "FAIL [%s] %s\n", scen, what); }
    };
    int delivered = 0; for (float t : r.delivered_at) delivered += t >= 0;
    std::string n = sc.name;
    bool all = delivered == (int)r.delivered_at.size();
    if (n == "five_pencils_two_robots" || n == "radio_loss") {
        check(all, sc.name, "every pencil delivered");
        check(r.credit[0] >= 2 && r.credit[1] >= 2, sc.name, "work split: each robot delivered at least 2");
    } else if (n == "one_pencil_two_robots") {
        check(all, sc.name, "pencil delivered");
        check(r.max_team[0] == 2, sc.name, "both robots worked the single pencil");
    } else if (n == "heavy_box") {
        check(all, sc.name, "box and pencil both delivered");
        check(r.help_events >= 1 && r.final_demand[0] == 2, sc.name, "stall detected and box re-rated to 2 robots");
        check(r.max_team[0] >= 2, sc.name, "two robots carried the box");
    } else if (n == "robot_failure") {
        check(all, sc.name, "all six delivered with one robot dead");
        check(r.credit[1] < 6, sc.name, "the dead robot is not carrying the load");
    } else if (n == "heavy_alone") {
        check(!all, sc.name, "one robot cannot move a 2-robot box (physics holds)");
        check(r.final_status[0] == OBJ_STUCK, sc.name, "the box is flagged stuck (every robot available tried)");
        check(r.final_state[0] == ST_IDLE, sc.name, "the robot stops pushing and idles instead of grinding");
    } else if (n == "late_helper") {
        check(all, sc.name, "box delivered after the second robot came online");
        check(r.max_team[0] == 2, sc.name, "both robots carried it");
    }
    check(r.collisions == 0, sc.name, "no robot-robot collisions");
    bool ok = fails == before;
    if (!verbose) fails = before;
    return ok;
}

int main(int argc, char** argv) {
    const char* trace_dir = nullptr;
    int seeds = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace_dir = argv[++i];
        if (!strcmp(argv[i], "--seeds") && i + 1 < argc) seeds = atoi(argv[++i]);
    }
    int randoms = 0; bool tight = false;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--tight")) tight = true;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--random") && i + 1 < argc) randoms = atoi(argv[++i]);
    if (randoms) {  // generality: random arenas, robot counts, load placements, a heavy box sometimes
        std::mt19937 g(42);
        auto U = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(g); };
        int pass = 0;
        for (int n = 0; n < randoms; n++) {
            Scenario sc{"random", "random", 1500, 1000, 1320, 500, 160, {}, {}};
            int nr = 2 + n % 3, no = 3 + (n * 7) % 5;
            bool heavy = n % 4 == 3;
            for (int i = 0; i < nr; i++) sc.robots.push_back({U(100, 300), 150 + i * (700.0f / nr), U(-1.5f, 1.5f)});
            auto clear_of = [&](float x, float y) {
                for (auto& o : sc.objects) if (hypotf(o.x - x, o.y - y) < 160) return false;
                return hypotf(x - sc.zone_x, y - sc.zone_y) > sc.zone_r + 120;
            };
            while ((int)sc.objects.size() < no + heavy) {
                float x = U(420, 1150), y = tight ? U(120, 880) : U(200, 800);  // default: loads >= 16 cm clear of walls (setup rule)
                if (!clear_of(x, y)) continue;
                bool h = heavy && sc.objects.empty();
                sc.objects.push_back({x, y, h ? 2 : 1, h ? 60.0f : 40.0f});
            }
            sc.duration = 300;
            Result r = run(sc, nullptr, 500 + n);
            int d = 0; for (float t : r.delivered_at) d += t >= 0;
            bool ok = d == (int)r.delivered_at.size() && r.collisions == 0;
            pass += ok;
            if (!ok && trace_dir) {
                std::string path = std::string(trace_dir) + "/layout_" + std::to_string(n) + ".json";
                FILE* tf = fopen(path.c_str(), "w"); run(sc, tf, 500 + n); fclose(tf);
            }
            if (!ok) {
                int wall = 0, stuck = 0, open_floor = 0;
                for (size_t j = 0; j < r.delivered_at.size(); j++) {
                    if (r.delivered_at[j] >= 0) continue;
                    float x = r.final_xy[j].x, y = r.final_xy[j].y, m = fminf(fminf(x, sc.arena_w - x), fminf(y, sc.arena_h - y));
                    if (r.final_status[j] == OBJ_STUCK) stuck++; else if (m < 75) wall++; else open_floor++;
                    printf("    load %zu at (%.0f,%.0f) wall-gap %.0f status %d demand %d weight %d\n", j + 1, x, y, m, r.final_status[j], r.final_demand[j], sc.objects[j].weight);
                }
                printf("  layout %d: undelivered -> %d against wall, %d flagged stuck, %d in open floor | ", n, wall, stuck, open_floor);
            }
            if (!ok) printf("  layout %d (%d robots, %d loads%s): %d/%zu delivered, %d collisions\n", n, nr, no + heavy, heavy ? " incl. heavy" : "", d, r.delivered_at.size(), r.collisions);
        }
        printf("random layouts: %d/%d fully delivered with zero collisions\n", pass, randoms);
        return pass == randoms ? 0 : 1;
    }
    if (seeds) {  // robustness: same scenarios, different noise / motor mismatch / packet-loss draws
        int bad = 0;
        for (const Scenario& sc : scenarios()) {
            int pass = 0; float worst = 0;
            for (int s = 0; s < seeds; s++) {
                Result r = run(sc, nullptr, 1000 + s * 7919);
                if (judge(sc, r, false)) pass++;
                if (r.t_end > worst) worst = r.t_end;
            }
            printf("%-24s %3d/%d seeds pass  (slowest run %.0fs)\n", sc.name, pass, seeds, worst);
            bad += seeds - pass;
        }
        return bad ? 1 : 0;
    }

    const char* only = nullptr; uint32_t seed = 1234;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--only") && i + 1 < argc) only = argv[++i];
        if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = (uint32_t)atoi(argv[++i]);
    }
    for (const Scenario& sc : scenarios()) {
        if (only && strcmp(only, sc.name)) continue;
        FILE* tf = nullptr;
        if (trace_dir) { std::string p = std::string(trace_dir) + "/" + sc.name + ".json"; tf = fopen(p.c_str(), "w"); }
        Result r = run(sc, tf, seed);
        if (tf) fclose(tf);

        int delivered = 0; for (float t : r.delivered_at) delivered += t >= 0;
        printf("%-24s %d/%zu delivered in %5.1fs | credit", sc.name, delivered, r.delivered_at.size(), r.t_end);
        for (int c : r.credit) printf(" %d", c);
        printf(" | help %d | collisions %d\n", r.help_events, r.collisions);

        judge(sc, r, true);
    }
    printf(fails ? "\n%d check(s) FAILED\n" : "\nALL SCENARIOS PASS\n", fails);
    return fails ? 1 : 0;
}
