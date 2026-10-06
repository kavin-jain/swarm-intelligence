#!/usr/bin/env python3
"""KPIs from a run log, the same ones the simulator reports, so real runs and simulated runs
compare like for like. Judged from camera geometry only: what the robots report about
themselves is logged but not trusted.

  python kpi.py run.csv            (a log from `satellite.py --log run.csv` or `build/sim --log`)

Log format (CSV, one row per thing per camera frame), after '#' header lines:
  # arena W H
  # dock X Y R             (one line per dock, in dock order)
  t,what,id,x,y,th,kind,status,state
what = robot | parcel; th = robot heading (rad); kind = the parcel's dock; status/state as
reported by the swarm (-1 if unknown).
"""
import csv
import math
import sys

ROBOT_R = 60          # mm, chassis radius (Tuning::robot_radius)
PICKED_MM = 20        # a parcel that moved this far from where it landed has been picked up
CONTACT_MM = 15       # robots overlapping by more than this = a collision (same as the simulator)
GONE_S = 2.0          # a tag unseen this long and seen again is a new parcel (tags are reused)
REST_S = 1.0          # delivered = in its dock and still this long (one still being set down when the log ends isn't)


def load(path):
    with open(path) as f:
        lines = f.read().splitlines()
    arena, docks, frames = None, [], {}
    for line in lines:
        if line.startswith("# arena"):
            arena = tuple(float(v) for v in line.split()[2:4])
        elif line.startswith("# dock"):
            docks.append(tuple(float(v) for v in line.split()[2:5]))
    for r in csv.DictReader(l for l in lines if not l.startswith("#")):
        frames.setdefault(float(r["t"]), []).append(r)
    return arena, docks, sorted(frames.items())


def kpis(path):
    arena, docks, frames = load(path)
    parcels = []          # one dict per parcel (an id that reappears after GONE_S is a new one)
    live = {}             # id -> its current parcel dict
    collisions, touching, wall_s, travel, last_xy = 0, set(), 0.0, {}, {}
    t_prev = None
    for t, rows in frames:
        dt = 0 if t_prev is None else t - t_prev
        robots = []
        for r in rows:
            x, y = float(r["x"]), float(r["y"])
            if r["what"] == "robot":
                rid = int(r["id"]); robots.append((rid, x, y))
                px, py = last_xy.get(("r", rid), (x, y))
                travel[rid] = travel.get(rid, 0) + math.hypot(x - px, y - py)
                last_xy[("r", rid)] = (x, y)
                continue
            pid, kind = int(r["id"]), int(r["kind"])
            p = live.get(pid)
            if p is None or t - p["seen"] > GONE_S:
                p = live[pid] = {"t0": t, "x0": x, "y0": y, "picked": None, "settled": None, "seen": t, "at": (x, y), "still": t}
                parcels.append(p)
            px, py = last_xy.get(("p", pid), (x, y))
            step = math.hypot(x - px, y - py)
            last_xy[("p", pid)] = (x, y)
            if math.hypot(x - p["at"][0], y - p["at"][1]) > PICKED_MM:
                p["at"], p["still"] = (x, y), t                # moved (more than camera noise): still since now
            if p["picked"] is None and math.hypot(x - p["x0"], y - p["y0"]) > PICKED_MM:
                p["picked"] = t - p["t0"]
            dx, dy, dr = docks[kind % len(docks)]
            inside = math.hypot(x - dx, y - dy) <= dr
            if inside and (p["settled"] is None or step > 2):
                p["settled"] = t - p["t0"]            # delivered = came to rest inside its dock
            elif not inside:
                p["settled"] = None
            p["seen"] = t
            if arena and step > 0.5 and (x < 45 or y < 45 or x > arena[0] - 45 or y > arena[1] - 45):
                wall_s += dt                           # moving while touching a wall: scraping
        now = {(a, b) for i, (a, ax, ay) in enumerate(robots) for b, bx, by in robots[i + 1:]
               if math.hypot(ax - bx, ay - by) < 2 * ROBOT_R - CONTACT_MM}
        collisions += len(now - touching)              # each contact counted once, when it starts
        touching, t_prev = now, t
    hours = (frames[-1][0] - frames[0][0]) / 3600 if frames else 0
    done = [p["settled"] for p in parcels if p["settled"] is not None and p["seen"] - p["still"] >= REST_S]
    waits = sorted(p["picked"] for p in parcels if p["picked"] is not None)
    return {
        "parcels seen": len(parcels),
        "delivered": len(done),
        "delivered %": 100.0 * len(done) / len(parcels) if parcels else 0.0,
        "parcels per hour": len(done) / hours if hours else 0.0,
        "wait for pickup mean s": sum(waits) / len(waits) if waits else 0.0,
        "wait for pickup p90 s": waits[len(waits) * 9 // 10] if waits else 0.0,
        "arrival to dock mean s": sum(done) / len(done) if done else 0.0,
        "collisions": collisions,
        "parcel scraping a wall, s": wall_s,
        "robot travel m": {k: round(v / 1000, 1) for k, v in sorted(travel.items())},
    }


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for k, v in kpis(sys.argv[1]).items():
        print(f"{k:28s} {v:.1f}" if isinstance(v, float) else f"{k:28s} {v}")
