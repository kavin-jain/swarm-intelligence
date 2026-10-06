// Browser build of the simulator: the robots' real core/brain.h + core/world.h,
// compiled to WebAssembly. JS drives it through these exports and reads state() out
// of wasm memory. Build: bash web/build.sh
#include "engine.h"

using namespace swarm;

static Engine* E = nullptr;
static float OUT[8 + MAX_ZONES * 3 + MAX_ROBOTS * 10 + MAX_OBJECTS * 9 + MAX_ROBOTS * 48];

#define EXPORT(name) extern "C" __attribute__((export_name(#name)))

EXPORT(reset) void reset(float w, float h, float zx, float zy, float zr, unsigned seed, float ship_after) {
    delete E;
    E = new Engine(w, h, zx, zy, zr, seed);
    E->ship_after = ship_after;
}
EXPORT(add_robot) int add_robot(float x, float y, float th) { return E->add_robot(x, y, th); }
EXPORT(add_dock) int add_dock(float x, float y, float r) { E->add_dock(x, y, r); return (int)E->docks.size() - 1; }
EXPORT(add_object) int add_object(float x, float y, int weight, int kind) { return E->add_object(x, y, weight, weight > 1 ? 60.0f : 40.0f, (uint8_t)kind); }
EXPORT(toggle_robot) int toggle_robot(int i) {
    if (i < 0 || i >= (int)E->bodies.size()) return -1;
    E->bodies[i].dead = !E->bodies[i].dead;
    return E->bodies[i].dead ? 0 : 1;
}
EXPORT(set_dock) void set_dock(int i, float x, float y) {
    if (i >= 0 && i < (int)E->docks.size()) { E->docks[i].x = (int16_t)x; E->docks[i].y = (int16_t)y; }
}
EXPORT(step) void step(int n) { for (int i = 0; i < n; i++) E->step(); }
// Gripper fitted (carry) or not (push): applies to every robot, now and added later.
EXPORT(set_carry) void set_carry(int on) { E->tune.carry = on != 0; for (auto& b : E->brains) b.t.carry = on != 0; }
EXPORT(energy_wh) float energy_wh() { return (float)(E->energy_j / 3600); }   // whole floor so far (model)

// What the floor manager in each robot is doing, for the overlay:
//   [map cells per side (HM), then MAX_ROBOTS x3: note (0 -, 1 waiting by a busy bay, 2 staging
//   at a full dock), x, y of the waiting spot; then robot 1's learned arrival map, HM*HM cells]
static float MGR[1 + MAX_ROBOTS * 3 + Brain::HM * Brain::HM];
EXPORT(manager) float* manager() {
    MGR[0] = Brain::HM;
    for (size_t i = 0; i < E->brains.size() && i < MAX_ROBOTS; i++) {
        const Brain& b = E->brains[i];
        bool waiting = !E->bodies[i].dead && b.state == ST_IDLE && b.has_wait_;
        MGR[1 + i * 3] = E->bodies[i].dead ? 0 : waiting ? 1 : b.staging_ ? 2 : 0;
        MGR[2 + i * 3] = b.wait_.x; MGR[3 + i * 3] = b.wait_.y;
    }
    for (int c = 0; c < Brain::HM * Brain::HM; c++) MGR[1 + MAX_ROBOTS * 3 + c] = E->brains.empty() ? 0 : E->brains[0].heat_[c];
    return MGR;
}

// Layout: [t, robots, objects, shipped, help_events, docks, MAX_ROBOTS, MAX_OBJECTS],
//   then MAX_ZONES docks x3: x, y, r   (JS derives the offsets below from the header)
//   robots  x10: x, y, th, off, state, task, help, neighbours, hear_mask, path_points
//   objects x9 : id, x, y, r, weight, status (0 open, 1 delivered, 2 stuck), demand, team, kind (dock)
//   paths   x48: up to 24 (x, y) points per robot, robot -> goal
EXPORT(state) float* state() {
    float* o = OUT;
    uint32_t now = (uint32_t)(E->time() * 1000) + 1;
    int nr = (int)E->bodies.size(), no = (int)E->things.size();
    o[0] = E->time(); o[1] = nr; o[2] = no; o[3] = E->shipped; o[4] = E->help_events;
    o[5] = (float)E->docks.size(); o[6] = MAX_ROBOTS; o[7] = MAX_OBJECTS;
    for (size_t d = 0; d < E->docks.size(); d++) { o[8 + d * 3] = E->docks[d].x; o[9 + d * 3] = E->docks[d].y; o[10 + d * 3] = E->docks[d].r; }
    float* r = o + 8 + MAX_ZONES * 3;
    float* ob = r + MAX_ROBOTS * 10;
    float* pa = ob + MAX_OBJECTS * 9;
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
    for (int j = 0; j < no; j++, ob += 9) {
        const Engine::Thing& t = E->things[j];
        const World::Obj* w = E->world_obj(t);
        int team = 0;
        for (int i = 0; i < nr; i++) if (!E->bodies[i].dead && E->brains[i].task == t.id) team++;
        ob[0] = t.id; ob[1] = t.x; ob[2] = t.y; ob[3] = t.r; ob[4] = t.weight;
        ob[5] = t.delivered ? 1 : (w && w->status == OBJ_STUCK ? 2 : 0);
        ob[6] = w ? w->demand : 1; ob[7] = team; ob[8] = t.kind;
    }
    return OUT;
}
