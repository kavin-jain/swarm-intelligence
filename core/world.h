// Gateway bookkeeping: merges camera frames and robot heartbeats into the snapshot
// every robot receives. It decides nothing for the robots; it only keeps the facts
// consistent: who is alive, where things are, which objects are delivered, and how
// many robots an object is known to need (raised when a robot reports a stall). If a load
// still won't move with every robot on it, it is marked stuck so the swarm moves on.
#pragma once
#include "proto.h"
#include <math.h>

namespace swarm {

struct World {
    static constexpr uint32_t ROBOT_SEEN_MS = 500;    // camera must have seen it this recently
    static constexpr uint32_t ROBOT_HB_MS = 1000;     // and its radio must have been heard
    static constexpr uint32_t OBJECT_KEEP_MS = 1500;  // forget objects the camera lost for longer
    static constexpr uint32_t HELP_COOLDOWN_MS = 4000;
    static constexpr uint8_t MAX_DEMAND = 2;          // two robots side by side is all that fits behind one load
    static constexpr float DELIVER_DEPTH = 70;        // mm inside the zone edge: noise can't fake a delivery, and loads get pushed deep so the entrance stays clear

    struct Rob { uint8_t id; float x, y, th; uint32_t seen, hb; uint8_t task = NONE, state = ST_IDLE, neighbors = 0; bool used = false; };
    struct Obj { uint8_t id; float x, y, stuck_x = 0, stuck_y = 0; uint32_t seen, cooldown = 0; uint8_t demand = 1, status = OBJ_OPEN, stuck_robots = 0, kind = 0; uint16_t r = 0; bool used = false; };

    Rob rob[MAX_ROBOTS];
    Obj obj[MAX_OBJECTS];
    uint8_t nz = 0; Zone z[MAX_ZONES] = {};
    uint16_t arena_w = 0, arena_h = 0;
    uint16_t seq = 0;

    template <class T> static T* find(T* arr, int n, uint8_t id, bool create) {
        for (int i = 0; i < n; i++) if (arr[i].used && arr[i].id == id) return &arr[i];
        if (!create) return nullptr;
        for (int i = 0; i < n; i++) if (!arr[i].used) { arr[i] = T(); arr[i].used = true; arr[i].id = id; return &arr[i]; }
        return nullptr;
    }

    void on_vision(const Vision& v, uint32_t now) {
        nz = v.nz; for (int i = 0; i < nz; i++) z[i] = v.z[i];
        arena_w = v.arena_w; arena_h = v.arena_h;
        for (int i = 0; i < v.nr; i++) {
            Rob* r = find(rob, MAX_ROBOTS, v.r[i].id, true);
            if (!r) continue;
            r->x = v.r[i].x; r->y = v.r[i].y; r->th = v.r[i].th / 1000.0f; r->seen = now;
        }
        for (int i = 0; i < v.no; i++) {
            Obj* o = find(obj, MAX_OBJECTS, v.o[i].id, true);
            if (!o) continue;
            o->x = v.o[i].x; o->y = v.o[i].y; o->seen = now; o->kind = v.o[i].kind; o->r = (uint16_t)(v.o[i].r * 2);
            if (nz) {   // delivered only into the dock for its kind (sorting)
                const Zone& d = z[o->kind % nz];
                float dx = o->x - d.x, dy = o->y - d.y, in = d.r - DELIVER_DEPTH, rr = (float)d.r;
                // Pushed loads must be well inside (noise can't fake it); a carried load counts
                // once its robot reports it set down anywhere in the dock.
                if (dx * dx + dy * dy <= in * in || (dx * dx + dy * dy <= rr * rr && placed(o->id, now))) o->status = OBJ_DELIVERED;  // sticky
            }
            // A stuck load that has been moved (by a person, or knocked free) gets another go.
            if (o->status == OBJ_STUCK && hypotf(o->x - o->stuck_x, o->y - o->stuck_y) > 100) { o->status = OBJ_OPEN; o->demand = 1; }
            // A delivered load seen well outside its dock was dragged or knocked out (delivery is sticky
            // against camera noise, not against that): it needs picking up again.
            if (o->status == OBJ_DELIVERED && nz) {
                const Zone& d = z[o->kind % nz];
                if (hypotf(o->x - d.x, o->y - d.y) > d.r + 10) o->status = OBJ_OPEN;   // camera noise is ~3 mm
            }
        }
        // Forget what the camera lost -- delivered loads too: a delivered load that disappears has been
        // shipped, and a ghost of it left in the map would block its dock slot for good.
        for (int i = 0; i < MAX_OBJECTS; i++)
            if (obj[i].used && now - obj[i].seen > OBJECT_KEEP_MS) obj[i].used = false;
    }

    void on_heartbeat(const Heartbeat& h, uint32_t now) {
        Rob* r = find(rob, MAX_ROBOTS, h.id, true);
        if (!r) return;
        r->hb = now; r->task = h.task; r->state = h.state; r->neighbors = h.neighbors;
        if (h.help != NONE) {
            Obj* o = find(obj, MAX_OBJECTS, h.help, false);
            // Ant-style recruitment: a robot that can't move its load raises the load's
            // headcount. Cooldown stops one long stall from escalating it every heartbeat.
            if (o && o->status == OBJ_OPEN && now >= o->cooldown) {
                int alive = 0;
                for (int i = 0; i < MAX_ROBOTS; i++) if (rob[i].used && robot_alive(rob[i], now)) alive++;
                int cap = alive < MAX_DEMAND ? alive : MAX_DEMAND;
                if (o->demand < cap) o->demand++;
                else { o->status = OBJ_STUCK; o->stuck_x = o->x; o->stuck_y = o->y; o->stuck_robots = (uint8_t)alive; }  // every robot we have couldn't move it
                o->cooldown = now + HELP_COOLDOWN_MS;
            }
        }
    }

    bool placed(uint8_t oid, uint32_t now) const {
        for (int i = 0; i < MAX_ROBOTS; i++)
            if (rob[i].used && rob[i].task == oid && rob[i].state == ST_PLACE && now - rob[i].hb <= ROBOT_HB_MS) return true;
        return false;
    }

    bool robot_alive(const Rob& r, uint32_t now) const {
        return r.seen && r.hb && now - r.seen <= ROBOT_SEEN_MS && now - r.hb <= ROBOT_HB_MS;
    }

    Snapshot snapshot(uint32_t now) {
        // A stuck load gets another go when more robots are online than tried it, now
        // needing one more than that (we know that many weren't enough).
        int alive = 0;
        for (int i = 0; i < MAX_ROBOTS; i++) if (rob[i].used && robot_alive(rob[i], now)) alive++;
        for (int i = 0; i < MAX_OBJECTS; i++) {
            Obj& o = obj[i];
            if (o.used && o.status == OBJ_STUCK && alive > o.stuck_robots && o.stuck_robots < MAX_DEMAND) {
                o.status = OBJ_OPEN; o.demand = o.stuck_robots + 1; o.cooldown = now + HELP_COOLDOWN_MS;
            }
        }
        Snapshot s{};
        s.seq = ++seq;
        s.nz = nz; for (int i = 0; i < nz; i++) s.z[i] = z[i];
        s.arena_w = arena_w; s.arena_h = arena_h;
        for (int i = 0; i < MAX_ROBOTS; i++) {
            const Rob& r = rob[i];
            if (!r.used || !r.seen) continue;           // heard on radio but never seen: nothing to report yet
            SnapRobot& o = s.r[s.nr++];
            o.id = r.id; o.x = clamp16(r.x); o.y = clamp16(r.y); o.th = clamp16(r.th * 1000.0f);
            o.alive = robot_alive(r, now); o.task = r.task; o.state = r.state;
        }
        for (int i = 0; i < MAX_OBJECTS; i++) {
            const Obj& b = obj[i];
            if (!b.used) continue;
            SnapObject& o = s.o[s.no++];
            o.id = b.id; o.x = clamp16(b.x); o.y = clamp16(b.y); o.demand = b.demand; o.status = b.status; o.kind = b.kind; o.r = b.r;
        }
        return s;
    }
};

}  // namespace swarm
