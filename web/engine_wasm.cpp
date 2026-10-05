// Browser build of the simulator: the robots' real core/brain.h + core/world.h,
// compiled to WebAssembly. JS drives it through these exports and reads state() out
// of wasm memory. Build: bash web/build.sh
#include "engine.h"

using namespace swarm;

static Engine* E = nullptr;
static float OUT[8 + MAX_ROBOTS * 10 + MAX_OBJECTS * 8 + MAX_ROBOTS * 48];

#define EXPORT(name) extern "C" __attribute__((export_name(#name)))

EXPORT(reset) void reset(float w, float h, float zx, float zy, float zr, unsigned seed, float ship_after) {
    delete E;
    E = new Engine(w, h, zx, zy, zr, seed);
    E->ship_after = ship_after;
}
EXPORT(add_robot) int add_robot(float x, float y, float th) { return E->add_robot(x, y, th); }
EXPORT(add_object) int add_object(float x, float y, int weight) { return E->add_object(x, y, weight, weight > 1 ? 60.0f : 40.0f); }
EXPORT(toggle_robot) int toggle_robot(int i) {
    if (i < 0 || i >= (int)E->bodies.size()) return -1;
    E->bodies[i].dead = !E->bodies[i].dead;
    return E->bodies[i].dead ? 0 : 1;
}
EXPORT(set_zone) void set_zone(float x, float y) { E->zone_x = x; E->zone_y = y; }
EXPORT(step) void step(int n) { for (int i = 0; i < n; i++) E->step(); }

// Layout: [t, robots, objects, shipped, help_events, zone x, y, r]
//   robots  x10: x, y, th, off, state, task, help, neighbours, hear_mask, path_points
//   objects x8 : id, x, y, r, weight, status (0 open, 1 delivered, 2 stuck), demand, team
//   paths   x48: up to 24 (x, y) points per robot, robot -> goal
EXPORT(state) float* state() {
    float* o = OUT;
    uint32_t now = (uint32_t)(E->time() * 1000) + 1;
    int nr = (int)E->bodies.size(), no = (int)E->things.size();
    o[0] = E->time(); o[1] = nr; o[2] = no; o[3] = E->shipped; o[4] = E->help_events;
    o[5] = E->zone_x; o[6] = E->zone_y; o[7] = E->zone_r;
    float* r = o + 8;
    float* ob = r + MAX_ROBOTS * 10;
    float* pa = ob + MAX_OBJECTS * 8;
    for (int i = 0; i < nr; i++, r += 10) {
        const Engine::Body& b = E->bodies[i];
        const Brain& br = E->brains[i];
        unsigned mask = 0;
        for (auto& p : br.peers) if (p.t && now - p.t <= 1000 && p.id >= 1 && p.id <= MAX_ROBOTS) mask |= 1u << (p.id - 1);
        int pts = 0;
        if (!b.dead && br.state == ST_GOTO && br.path_n_ > 0) {
            int n = br.path_n_, take = n < 24 ? n : 24;
            for (int q = 0; q < take; q++) {   // path_ runs goal -> first step; emit first step -> goal
                int idx = n - 1 - (int)((long)q * (n - 1) / (take > 1 ? take - 1 : 1));
                int c = br.path_[idx];
                pa[i * 48 + q * 2] = (c % br.path_w_ + 0.5f) * br.path_cell_;
                pa[i * 48 + q * 2 + 1] = (c / br.path_w_ + 0.5f) * br.path_cell_;
            }
            pts = take;
        }
        r[0] = b.x; r[1] = b.y; r[2] = b.th; r[3] = b.dead; r[4] = br.state;
        r[5] = br.task == NONE ? 0 : br.task; r[6] = br.help == NONE ? 0 : br.help;
        r[7] = br.neighbors(now); r[8] = (float)mask; r[9] = pts;
    }
    for (int j = 0; j < no; j++, ob += 8) {
        const Engine::Thing& t = E->things[j];
        const World::Obj* w = E->world_obj(t);
        int team = 0;
        for (int i = 0; i < nr; i++) if (!E->bodies[i].dead && E->brains[i].task == t.id) team++;
        ob[0] = t.id; ob[1] = t.x; ob[2] = t.y; ob[3] = t.r; ob[4] = t.weight;
        ob[5] = t.delivered ? 1 : (w && w->status == OBJ_STUCK ? 2 : 0);
        ob[6] = w ? w->demand : 1; ob[7] = team;
    }
    return OUT;
}
