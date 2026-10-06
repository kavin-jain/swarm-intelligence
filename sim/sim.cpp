// Host simulator. Every robot runs the real core/brain.h and the gateway runs the real
// core/world.h; only the physics, the camera and the radio are simulated.
//
//   c++ -std=c++17 -O2 -Wall -I core sim/sim.cpp -o build/sim
//   build/sim                 run every scenario, exit 1 if any check fails
//   build/sim --trace DIR     also write DIR/<scenario>.json replays for the web viewer
//   build/sim --inbound 40    40 ten-minute shifts of parcels arriving at receiving bays (latency, energy)
#include "brain.h"
#include "world.h"
#include "engine.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace swarm;

struct RobotSpec { float x, y, th; };
struct ObjSpec { float x, y; int weight; float radius; int kind = 0; };  // weight = robots needed; kind picks the dock
struct Scenario {
    const char* name; const char* title;
    float arena_w, arena_h, zone_x, zone_y, zone_r;
    std::vector<RobotSpec> robots;
    std::vector<ObjSpec> objects;
    float loss = 0;                        // radio packet loss, both directions
    int kill_robot = 0; float kill_at = 0; // robot id that dies (battery) at time kill_at
    int join_robot = 0; float join_at = 0; // robot id that is switched on late, at join_at
    float duration = 180;
    float dock2_x = 0, dock2_y = 0, dock2_r = 0;   // optional second dock (sorting)
};

// Mode for every run: carry (gripper, the default) or push (today's robots without one).
// Docks ship delivered loads after `g_ship` s (0 = never), like an outbound dock.
static bool g_push = false;
static float g_ship = -1;   // -1: default for the mode (carry 5 s, push never)
static double g_wh = 0; static int g_loads = 0, g_hard = 0;   // energy totals for the seeded summary

struct Result {
    std::vector<float> delivered_at;       // per object, -1 if never
    std::vector<int> credit;               // deliveries credited per robot (in contact at delivery)
    std::vector<int> max_team;             // most robots assigned to an object at once
    std::vector<int> final_demand;
    int help_events = 0, collisions = 0, wall_drag = 0, hard_starts = 0; float shoved_mm = 0, energy_wh = 0;
    std::vector<int> final_state, final_status;
    std::vector<V2> final_xy;
    float t_end = 0;
};

static Result run(const Scenario& sc, FILE* trace, uint32_t seed) {
    Engine e(sc.arena_w, sc.arena_h, sc.zone_x, sc.zone_y, sc.zone_r, seed);
    e.loss = sc.loss;
    e.tune.carry = !g_push;
    e.ship_after = g_ship >= 0 ? g_ship : g_push ? 0 : 5;
    for (auto& r : sc.robots) e.add_robot(r.x, r.y, r.th);
    if (sc.dock2_r > 0) e.add_dock(sc.dock2_x, sc.dock2_y, sc.dock2_r);
    for (auto& o : sc.objects) e.add_object(o.x, o.y, o.weight, o.radius, (uint8_t)o.kind);
    Result res;
    size_t no = e.things.size(), nb = e.bodies.size();
    std::vector<int> ids;   // results are kept per load id: shipped loads leave e.things
    for (auto& t : e.things) ids.push_back(t.id);
    std::vector<V2> last_xy(no);
    res.delivered_at.assign(no, -1);
    res.credit.assign(nb, 0);
    res.max_team.assign(no, 0);
    bool first_trace = true;
    if (trace) {
        fprintf(trace, "{\"name\":\"%s\",\"title\":\"%s\",\"arena\":[%.0f,%.0f],\"zone\":[%.0f,%.0f,%.0f],\"robot_r\":%.0f,\"objects\":[",
                sc.name, sc.title, sc.arena_w, sc.arena_h, sc.zone_x, sc.zone_y, sc.zone_r, Engine::ROBOT_R);
        for (size_t j = 0; j < no; j++) fprintf(trace, "%s[%.0f,%d]", j ? "," : "", e.things[j].r, e.things[j].weight);
        fprintf(trace, "],\"frames\":[");
    }
    int steps = (int)(sc.duration / Engine::DT);
    for (int k = 0; k < steps; k++) {
        float tsec = k * Engine::DT;
        uint32_t now = (uint32_t)(tsec * 1000) + 1;
        if (sc.kill_robot && tsec >= sc.kill_at) e.bodies[sc.kill_robot - 1].dead = true;
        if (sc.join_robot) e.bodies[sc.join_robot - 1].dead = tsec < sc.join_at;
        e.step();
        if (getenv("SIM_WORLD") && k % 250 == 0) {   // SIM_WORLD=1: the gateway's view every 5 s
            fprintf(stderr, "t=%.0f world:", tsec);
            for (auto& o : e.world.obj) if (o.used) fprintf(stderr, " #%d(%.0f,%.0f) st%d d%d", o.id, o.x, o.y, o.status, o.demand);
            fprintf(stderr, " | robots:");
            for (auto& r : e.world.rob) if (r.used) fprintf(stderr, " R%d st%d T%d", r.id, r.state, r.task == NONE ? 0 : r.task);
            fprintf(stderr, "\n");
        }
        if (getenv("SIM_DEBUG") && k % 25 == 0) {   // SIM_DEBUG=robot_index: print that robot's carry targets
            int who = atoi(getenv("SIM_DEBUG"));
            Brain& b = e.brains[who]; const Engine::Body& bd = e.bodies[who];
            int oj = b.obj(b.task);
            V2 slot{0, 0}, in{0, 0}; bool ok = oj >= 0 && b.my_slot(oj, slot, in);
            V2 p{bd.x, bd.y}, stage = slot - in * (grip_reach(b.t) + b.t.pre_dock);
            V2 g = ok ? b.path_step(p, stage, b.me(), oj, b.t.carry_body) : V2{0, 0};
            fprintf(stderr, "t=%.1f R%d st%d task%d pos(%.0f,%.0f,%.2f) lr(%.2f,%.2f) slot(%.0f,%.0f) stage(%.0f,%.0f) goal(%.0f,%.0f) ins%d held%d help%d off%.0f\n", tsec, who + 1, b.state, b.task,
                    bd.x, bd.y, bd.th, bd.l, bd.r, slot.x, slot.y, stage.x, stage.y, g.x, g.y, b.inserting_, bd.held, b.help, b.off_);
        }
        for (size_t j = 0; j < no; j++) {
            int n = 0, ti = e.thing_index(ids[j]);
            for (auto& b : e.brains) if (b.task == ids[j]) n++;
            if (n > res.max_team[j]) res.max_team[j] = n;
            if (ti < 0) continue;   // shipped
            const Engine::Thing& th = e.things[ti];
            last_xy[j] = {th.x, th.y};
            if (th.delivered && res.delivered_at[j] < 0) {   // credit robots touching it as it arrives
                res.delivered_at[j] = tsec;
                for (size_t i = 0; i < nb; i++)
                    if (hypotf(th.x - e.bodies[i].x, th.y - e.bodies[i].y) < Engine::ROBOT_R + th.r + 25) res.credit[i]++;
            }
        }
        if (trace && k % 5 == 0) {   // 10 fps replay
            fprintf(trace, "%s[%.1f,[", first_trace ? "" : ",", tsec);
            first_trace = false;
            for (size_t i = 0; i < nb; i++)
                fprintf(trace, "%s[%.0f,%.0f,%.2f,%d,%d,%d,%d]", i ? "," : "", e.bodies[i].x, e.bodies[i].y, e.bodies[i].th,
                        e.bodies[i].dead ? -1 : (int)e.brains[i].state, e.brains[i].task == NONE ? 0 : e.brains[i].task,
                        e.brains[i].help == NONE ? 0 : e.brains[i].help, e.brains[i].neighbors(now));
            fprintf(trace, "],[");
            for (size_t j = 0; j < no; j++) {
                int ti = e.thing_index(ids[j]);
                if (ti < 0) { fprintf(trace, "%s[%.0f,%.0f,1,1]", j ? "," : "", last_xy[j].x, last_xy[j].y); continue; }
                const World::Obj* wo = e.world_obj(e.things[ti]);
                fprintf(trace, "%s[%.0f,%.0f,%d,%d]", j ? "," : "", e.things[ti].x, e.things[ti].y,
                        e.things[ti].delivered ? 1 : (wo && wo->status == OBJ_STUCK ? 2 : 0), wo ? wo->demand : 1);
            }
            fprintf(trace, "]]");
        }
        res.t_end = tsec;
        bool all = true;
        for (float d : res.delivered_at) all = all && d >= 0;
        if (all && tsec > 1) break;
    }
    if (trace) fprintf(trace, "]}\n");
    res.help_events = e.help_events; res.collisions = e.collisions; res.wall_drag = e.wall_drag; res.hard_starts = e.hard_starts; res.shoved_mm = e.shoved_mm; res.energy_wh = (float)(e.energy_j / 3600);
    for (size_t j = 0; j < no; j++) {
        int ti = e.thing_index(ids[j]);
        const World::Obj* wo = ti >= 0 ? e.world_obj(e.things[ti]) : nullptr;
        res.final_demand.push_back(wo ? wo->demand : 0);
        res.final_status.push_back(ti < 0 ? OBJ_DELIVERED : wo ? wo->status : 0);
        res.final_xy.push_back(last_xy[j]);
    }
    for (auto& b : e.brains) res.final_state.push_back(b.state);
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
    {
        Scenario sort{"sorting", "Two docks: each parcel goes to the dock for its colour", W, H, 1320, 750, 150, {{150, 200, 0}, {150, 500, 0}, {150, 800, 0}}, {}};
        sort.dock2_x = 1320; sort.dock2_y = 250; sort.dock2_r = 150;
        float px[6] = {480, 600, 420, 820, 760, 950}, py[6] = {260, 760, 520, 330, 620, 500};
        for (int i = 0; i < 6; i++) { ObjSpec o = at(pencil, px[i], py[i]); o.kind = i % 2; sort.objects.push_back(o); }
        v.push_back(sort);
    }
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
        if (g_push) check(r.max_team[0] == 2, sc.name, "both robots worked the single pencil");
        else check(r.max_team[0] == 1, sc.name, "one robot carries it; the other stays out of the way");
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
    } else if (n == "sorting") {
        check(all, sc.name, "every parcel delivered to the dock for its colour");
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
        if (!strcmp(argv[i], "--push")) g_push = true;
        if (!strcmp(argv[i], "--ship") && i + 1 < argc) g_ship = (float)atof(argv[++i]);
    }
    int dense = 0;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--dense") && i + 1 < argc) dense = atoi(argv[++i]);
    if (dense) {  // scale: 6-10 robots, 10-14 loads (a crate sometimes), two colour docks
        std::mt19937 g(7);
        auto U = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(g); };
        int pass = 0, loads = 0; float tsum = 0, wh = 0; int coll = 0;
        for (int n = 0; n < dense; n++) {
            Scenario sc{"dense", "dense", 1500, 1000, 1320, 730, 150, {}, {}};
            sc.dock2_x = 1320; sc.dock2_y = 270; sc.dock2_r = 150;
            int nr = 6 + n % 5, no = 10 + (n * 3) % 5;
            for (int i = 0; i < nr; i++) sc.robots.push_back({i < 5 ? 130.0f : 290.0f, 120 + (i % 5) * 190.0f, U(-1.0f, 1.0f)});
            while ((int)sc.objects.size() < no) {
                float x = U(450, 1080), y = U(150, 850); bool ok = true;
                for (auto& o : sc.objects) if (hypotf(o.x - x, o.y - y) < 150) ok = false;
                // site rule: a dock's approach lanes are keep-clear (nothing stored within 40 cm of a dock)
                if (hypotf(x - sc.zone_x, y - sc.zone_y) < sc.zone_r + 250 || hypotf(x - sc.dock2_x, y - sc.dock2_y) < sc.dock2_r + 250) ok = false;
                if (!ok) continue;
                bool h = n % 3 == 2 && sc.objects.empty();
                ObjSpec o{x, y, h ? 2 : 1, h ? 60.0f : 40.0f}; o.kind = (int)sc.objects.size() % 2;
                sc.objects.push_back(o);
            }
            sc.duration = 400;
            int only = getenv("SIM_LAYOUT") ? atoi(getenv("SIM_LAYOUT")) : -1;
            if (only >= 0 && n != only) continue;
            Result r = run(sc, nullptr, 900 + n);
            int d = 0; for (float t : r.delivered_at) d += t >= 0;
            bool ok = d == (int)r.delivered_at.size() && r.collisions == 0;
            pass += ok; coll += r.collisions; wh += r.energy_wh; loads += d;
            if (ok) tsum += r.t_end;
            else printf("  dense %d (%d robots, %d loads): %d/%zu delivered, %d collisions\n", n, nr, no, d, r.delivered_at.size(), r.collisions);
        }
        printf("dense floors: %d/%d fully delivered, zero collisions (collision events %d), mean finish %.0f s, %.1f mWh per delivered load\n", pass, dense, coll, pass ? tsum / pass : 0, loads ? 1000 * wh / loads : 0);
        return pass == dense ? 0 : 1;
    }
    int inbound = 0;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--inbound") && i + 1 < argc) inbound = atoi(argv[++i]);
    // A 10-minute shift: parcels keep arriving at 2-3 receiving bays, one busier than the rest.
    // SIM_SCALE=2: a floor twice the size, same layout. SIM_LOAD=2.5: 2.5x fewer trucks. SIM_FIRST=n: other shifts.
    if (inbound) {
        int arrived = 0, picked = 0, delivered = 0, coll = 0, left = 0, maxq = 0; double wait = 0, cycle = 0, wh = 0, idle_wh = 0;
        std::vector<float> waits;
        int first = getenv("SIM_FIRST") ? atoi(getenv("SIM_FIRST")) : 0;   // SIM_FIRST=40: a different set of shifts
        FILE* log = nullptr;   // --log FILE: what the camera would record, for satellite/kpi.py
        for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--log") && i + 1 < argc) log = fopen(argv[++i], "w");
        int lg_landed = 0, lg_done = 0, lg_picked = 0; double lg_wait = 0, lg_cycle = 0;   // the same KPIs from ground truth, timed from landing
        double budget[16] = {}, staging = 0, robot_s = 0;   // robot-seconds by brain state (SIM_BUDGET=1 prints it)
        int entries[16] = {}, align_back = 0;
        for (int n = first; n < first + inbound; n++) {
            std::mt19937 g(3000 + n), gp(4000 + n);   // the trucks (same arrivals whatever the robots do), and where parcels land
            auto U = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(g); };
            auto UP = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(gp); };
            int nr = getenv("SIM_ROBOTS") ? atoi(getenv("SIM_ROBOTS")) : 2 + n % 4;   // SIM_ROBOTS=6: fixed fleet size
            if (getenv("SIM_LAYOUT") && atoi(getenv("SIM_LAYOUT")) != n) continue;
            const float SC = getenv("SIM_SCALE") ? (float)atof(getenv("SIM_SCALE")) : 1;   // a bigger floor, same layout
            Engine e(1500 * SC, 1000 * SC, 1320 * SC, 730 * SC, 150, 2000 + n);
            e.add_dock(1320 * SC, 270 * SC, 150);
            e.tune.carry = !g_push; e.ship_after = 5;
            for (int i = 0; i < nr; i++) e.add_robot(i < 5 ? 130.0f : 290.0f, (120 + (i % 5) * 190.0f) * SC, 0);   // charging wall: up to 5 a column
            std::vector<V2> bay; std::vector<float> share = n % 2 ? std::vector<float>{0.7f, 0.3f} : std::vector<float>{0.6f, 0.3f, 0.1f};
            for (int tries = 0; bay.size() < share.size(); tries++) {
                if (tries % 200 == 199) bay.clear();   // boxed in by the first bays: start the layout again
                V2 c{U(450, 1000) * SC, U(200, 800) * SC}; bool ok = hypotf(c.x - 1320 * SC, c.y - 730 * SC) > 450 && hypotf(c.x - 1320 * SC, c.y - 270 * SC) > 450;
                for (V2 b : bay) ok = ok && len(b - c) > 350 * SC;
                if (ok) bay.push_back(c);
            }
            struct Rec { float x, y, t_arr, t_land, t_pick = -1; bool done = false; };
            if (log && n == first) {
                fprintf(log, "# arena %.0f %.0f\n", e.arena_w, e.arena_h);
                for (auto& d : e.docks) fprintf(log, "# dock %d %d %d\n", d.x, d.y, d.r);
                fprintf(log, "t,what,id,x,y,th,kind,status,state\n");
            }
            std::vector<std::pair<int, Rec>> rec;   // by load id, while the load is on the floor
            struct Truck { float t; int bay, kind; };
            std::vector<Truck> queue;   // parcels waiting for room at their bay
            float mean = 30.0f / nr * (getenv("SIM_LOAD") ? (float)atof(getenv("SIM_LOAD")) : 1), next = U(2, 6);   // SIM_LOAD=2: half as many trucks
            for (int k = 0; k < (int)(600 / Engine::DT); k++) {
                float tsec = k * Engine::DT;
                while (tsec >= next) {
                    float u = U(0, 1), acc = 0; int b = 0;
                    for (; b < (int)share.size() - 1; b++) if (u < (acc += share[b])) break;
                    queue.push_back({next, b, U(0, 1) < 0.5f});
                    next += -mean * logf(1 - U(0, 0.999f));
                }
                for (size_t qi = 0; qi < queue.size(); qi++) {   // each bay unloads its own trucks, oldest first, when it has room
                    bool first = true;
                    for (size_t qj = 0; qj < qi; qj++) first = first && queue[qj].bay != queue[qi].bay;
                    if (!first) continue;
                    V2 c = bay[queue[qi].bay]; bool landed = false;
                    for (int tries = 0; tries < 10 && !landed; tries++) {
                        float x = c.x + UP(-150, 150), y = c.y + UP(-150, 150); bool ok = true;   // a bay holds ~4 parcels
                        for (auto& t : e.things) ok = ok && hypotf(t.x - x, t.y - y) > 160;
                        for (auto& b : e.bodies) ok = ok && hypotf(b.x - x, b.y - y) > Engine::ROBOT_R + 70;
                        if (!ok) continue;
                        int ti = e.add_object(x, y, 1, 40, (uint8_t)queue[qi].kind);
                        if (ti < 0) break;
                        rec.push_back({e.things[ti].id, {x, y, queue[qi].t, tsec}}); lg_landed++;
                        queue.erase(queue.begin() + qi); qi--; arrived++; landed = true;
                    }
                }
                maxq = std::max(maxq, (int)queue.size());
                e.step();
                for (size_t i = 0; i < e.brains.size(); i++) {
                    if (!e.bodies[i].dead) { budget[e.brains[i].state & 15] += Engine::DT; staging += e.brains[i].staging_ ? Engine::DT : 0; robot_s += Engine::DT; }
                    static uint8_t last[MAX_ROBOTS];
                    if (e.brains[i].state != last[i]) { entries[e.brains[i].state & 15]++; if (last[i] == ST_ALIGN && e.brains[i].state == ST_GOTO) align_back++; last[i] = e.brains[i].state; }
                }
                if (log && k % 3 == 0) {   // 15 fps, like the camera
                    float lt = tsec + (n - first) * 610.0f;   // shifts back to back, 10 s apart
                    for (size_t i = 0; i < e.bodies.size(); i++)
                        fprintf(log, "%.2f,robot,%zu,%.1f,%.1f,%.3f,-1,-1,%d\n", lt, i + 1, e.bodies[i].x, e.bodies[i].y, e.bodies[i].th, e.brains[i].state);
                    for (auto& t : e.things) {
                        const World::Obj* wo = e.world_obj(t);
                        fprintf(log, "%.2f,parcel,%d,%.1f,%.1f,0,%d,%d,-1\n", lt, t.id, t.x, t.y, t.kind, wo ? wo->status : -1);
                    }
                }
                for (size_t q = 0; q < rec.size();) {
                    Rec& r = rec[q].second; int ti = e.thing_index(rec[q].first);
                    if (ti >= 0) {
                        const Engine::Thing& t = e.things[ti];
                        if (r.t_pick < 0 && (g_push ? hypotf(t.x - r.x, t.y - r.y) > 20 : e.held(t))) {
                            r.t_pick = tsec; picked++; wait += tsec - r.t_arr; waits.push_back(tsec - r.t_arr);
                            lg_picked++; lg_wait += tsec - r.t_land;
                        }
                        if (!r.done && t.delivered) { r.done = true; delivered++; cycle += t.delivered_at - r.t_arr; lg_done++; lg_cycle += t.delivered_at - r.t_land; }
                        q++;
                    } else rec.erase(rec.begin() + q);   // shipped
                }
            }
            for (auto& r : rec) left += !r.second.done;
            left += (int)queue.size();
            coll += e.collisions; wh += e.energy_j / 3600; idle_wh += e.idle_drive_j / 3600;
        }
        std::sort(waits.begin(), waits.end());
        if (getenv("SIM_BUDGET")) {
            const char* names[] = {"idle", "goto", "align", "wait", "push", "backoff", "stopped", "dock", "grip", "carry", "place"};
            printf("robot time:");
            for (int st = 0; st <= 10; st++) if (budget[st] > 0) printf(" %s %.1f%%", names[st], 100 * budget[st] / robot_s);
            printf(" | of which staging at a full dock %.1f%% | %.1f parcels per robot-hour\n", 100 * staging / robot_s, delivered / (robot_s / 3600));
            printf("per delivered parcel: entries");
            for (int st = 0; st <= 10; st++) if (entries[st]) printf(" %s %.2f (%.2f s each)", names[st], (double)entries[st] / delivered, budget[st] / entries[st]);
            printf(" | align->goto %.2f\n", (double)align_back / delivered);
        }
        if (log) {
            fclose(log);
            printf("log check: %d landed, %d delivered, wait from landing mean %.1f s, landing to dock mean %.1f s, collisions %d\n",
                   lg_landed, lg_done, lg_picked ? lg_wait / lg_picked : 0, lg_done ? lg_cycle / lg_done : 0, coll);
        }
        printf("inbound shifts: %d landed, %d delivered, %d left at the end | wait for pickup mean %.1f s, p90 %.1f s | arrival to dock %.1f s | %.1f mWh per parcel (%.1f driving with no job) | collisions %d | longest truck queue %d\n",
               arrived, delivered, left, picked ? wait / picked : 0, waits.empty() ? 0 : waits[waits.size() * 9 / 10], delivered ? cycle / delivered : 0,
               delivered ? 1000 * wh / delivered : 0, delivered ? 1000 * idle_wh / delivered : 0, coll, maxq);
        return 0;
    }
    int randoms = 0; bool tight = false;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--tight")) tight = true;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--random") && i + 1 < argc) randoms = atoi(argv[++i]);
    if (randoms) {  // generality: random arenas, robot counts, load placements, a heavy box sometimes
        std::mt19937 g(42);
        auto U = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(g); };
        int pass = 0;
        int only_layout = getenv("SIM_LAYOUT") ? atoi(getenv("SIM_LAYOUT")) : -1;   // SIM_LAYOUT=n: run just that layout
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
            if (only_layout >= 0 && n != only_layout) continue;
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
        // Seeds are fixed, so this is deterministic. It fails on more than 2% bad runs: a few
        // hard draws (a dead robot parked next to the last load) fail on any version, and
        // which ones they are reshuffles with every change. A real regression costs more.
        int bad = 0, total = 0;
        for (const Scenario& sc : scenarios()) {
            int pass = 0, drag = 0, coll = 0, delivered = 0, hard = 0; float worst = 0, knocked = 0, wh = 0;
            for (int s = 0; s < seeds; s++) {
                Result r = run(sc, nullptr, 1000 + s * 7919);
                if (judge(sc, r, false)) pass++;
                if (r.t_end > worst) worst = r.t_end;
                drag += r.wall_drag; hard += r.hard_starts; knocked += r.shoved_mm; coll += r.collisions; wh += r.energy_wh;
                for (float d : r.delivered_at) delivered += d >= 0;
            }
            printf("%-24s %3d/%d seeds pass  (slowest run %.0fs)", sc.name, pass, seeds, worst);
            printf("  | wall drag %d, collisions %d, %.1f mWh/load", drag, coll, delivered ? 1000 * wh / delivered : 0);
            if (!g_push) printf(", knocked %.0f mm", knocked);
            g_wh += wh; g_loads += delivered; g_hard += hard;
            printf("\n");
            bad += seeds - pass; total += seeds;
        }
        printf("seeded runs: %d/%d pass (%.1f%%), %.1f mWh per delivered load, %d hard motor starts\n", total - bad, total, 100.0 * (total - bad) / total, g_loads ? 1000 * g_wh / g_loads : 0, g_hard);
        return bad * 50 > total ? 1 : 0;
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
        printf(" | help %d | collisions %d | wall drag %d | knocked %.0fmm\n", r.help_events, r.collisions, r.wall_drag, r.shoved_mm);

        judge(sc, r, true);
    }
    printf(fails ? "\n%d check(s) FAILED\n" : "\nALL SCENARIOS PASS\n", fails);
    return fails ? 1 : 0;
}
