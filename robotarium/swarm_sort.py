"""Leaderless parcel sorting, run on the Robotarium (Georgia Tech's remote robot swarm).

Kavin Jain's swarm (github.com/kavin-jain/swarm-intelligence) sorts parcels with no central
planner: every robot runs the same deterministic planner on the same shared snapshot of the
floor, so they agree on who does what without negotiating. This script runs that coordination
layer on the Robotarium's real robots:

  real here     robot motion, tracking, timing, the robots' own dynamics and lag
  virtual here  parcels, docks and bays: drawn by this script and projected onto the floor;
                a parcel "gripped" by a robot is drawn riding in front of it
  theirs        collision avoidance (the Robotarium's barrier certificates, as required)
  ours          per-robot brains: lockstep job allocation with dock admission control,
                pairs recruited for a heavy crate, staging beside full docks, a robot failing
                mid-shift, and deadlock recovery by preemption (ported from core/brain.h)

One brain per robot. Each brain receives the snapshot with the robots listed in its own order
(as if it heard the radio in a different order) and must still compute the identical plan; the
script checks that every snapshot and counts any disagreement.
"""
import os

import numpy as np
import matplotlib.patches as patches

from cvxopt import matrix, sparse
from cvxopt.solvers import options, qp

import rps.robotarium as robotarium
from rps.utilities.misc import determine_font_size
from rps.utilities.transformations import create_si_to_uni_dynamics, create_si_to_uni_mapping, create_uni_to_si_mapping

FAST = bool(os.environ.get("SWARM_FAST"))      # CI preview: run faster than real time
FRAMES = os.environ.get("SWARM_FRAMES")        # CI preview: save a frame every FRAME_EVERY steps here
FRAME_EVERY = 10

# ---- the floor: our 3 x 2 m benchmark floor, scaled so every length is the same in robot-lengths ---
# Our robots are 12 cm across (Tuning::robot_radius = 60 mm); the Robotarium's are 11 cm. Every length
# below is our simulator's value (mm, floor origin bottom-left) times S, placed in the Robotarium frame
# (metres, origin at the centre, x in [-1.6, 1.6], y in [-1, 1]). Our 3 x 2 m floor becomes 2.75 x 1.83 m.
S = 0.11 / 0.12


def floor(x_mm, y_mm):
    return np.array([(x_mm - 1500) * S / 1000, (y_mm - 1000) * S / 1000])


N = 8                                    # robots
DURATION = 300.0                         # s of sorting after the robots reach their start poses
DT = robotarium.Robotarium.TIME_STEP if hasattr(robotarium.Robotarium, "TIME_STEP") else 0.033
SNAP_EVERY = 3                           # a new shared snapshot every 3 steps (~10 Hz, like the radio)
DOCKS = [floor(2640, 1460), floor(2640, 540)]   # dock A (kind 0), dock B (kind 1): the shift benchmark's docks
DOCK_R = 0.150 * S
BAYS = [(floor(1500, 1300), 0.6), (floor(1400, 600), 0.4)]   # where trucks unload, share of parcels
HOME = [np.array([-1.35 if i < 5 else -1.00, -0.70 + 0.35 * (i % 5)]) for i in range(N)]
# the charging wall, 5 a column, as in our simulator, but 35 cm apart both ways (NOT to scale: the
# Robotarium's start-up routine keeps robots 25 cm apart, so start poses closer than that are never reached
# and the experiment is rejected)

REACH = 0.105 * S     # robot centre to the centre of a parcel on its gripper (robot_radius + object_radius)
PRE = 0.080 * S       # line up this far behind the grip point, then drive straight in (pre_dock)
PARCEL = 0.045 * S    # parcel half-size (object_radius)
CRATE = 0.060 * S     # crate half-size: needs two robots
PAIR = 0.24           # NOT to scale: our pairs stand 12 cm apart, but the Robotarium keeps robots >= 13.5 cm apart (its rule)
SLOTS = 3             # set-down spots per dock: our dock geometry gives 6 round the ring; these are the 3 facing the floor
OVERBOOK = 2      # carriers allowed beyond a dock's free slots (they stage beside it)
SHIP_S = 5.0      # a delivered parcel leaves its slot after this long
STICKY = 0.15     # m of cost a robot saves by keeping its current job (anti-thrash)
CRUISE = 0.15         # m/s, NOT to scale: ours drive 0.26 m/s (2.2 body-lengths/s); the Robotarium's max is 0.2,
                      # so times here run about 1.6x longer than on our floor
FAIL_AT, FAIL_ROBOT = 150.0, 2           # robot 3's battery "dies" mid-shift
CRATE_AT = 45.0                          # a heavy crate arrives at bay 1
DEADLOCK_S, COOLDOWN_S = 20.0, 30.0      # deadlock recovery: see watch_carriers()

IDLE, GOTO, ALIGN, WAIT, DOCK, GRIP, CARRY, INSERT, BACKOFF, DEAD = range(10)
NAMES = ["idle", "goto", "align", "wait", "dock", "grip", "carry", "insert", "backoff", "dead"]
HOLDING = (GRIP, CARRY, INSERT)
NONE = -1


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.array([1.0, 0.0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---- collision avoidance: the Robotarium's barrier certificate, with dead robots held still ------
# Same maths, gains and solver as rps' create_uni_barrier_certificate_with_boundary(safety_radius=0.13):
# every pair of robots keeps h = |p_i - p_j|^2 - r^2 >= 0 (p = a point 3 cm ahead of each robot) through
# dh/dt >= -gain * h^3, plus the arena walls and a speed limit, solved as one QP. The one change: the
# rps version treats every robot (and every "obstacle") as able to move, splitting each avoidance between
# the two. A dead robot can't do its share; there its velocity is not a variable, the live robot takes the
# whole of it. (Measured: zeroing the dead robot after the rps barrier left 3,331 too-close steps.)
PROJ, SAFE, GAIN, VMAX = 0.03, 0.13 + 2 * 0.03, 150.0, 0.2
WALLS = np.array([-1.6, 1.6, -1.0, 1.0])
options["show_progress"] = False
options["reltol"] = options["feastol"] = 1e-2
options["maxiters"] = 50
si_to_uni_proj, uni_to_si_states = create_si_to_uni_mapping(projection_distance=PROJ)
uni_to_si_dyn, _ = create_uni_to_si_mapping(projection_distance=PROJ)


def barrier(dxu, x, fixed):
    n_all = x.shape[1]
    live = [i for i in range(n_all) if i not in fixed]
    if not live:
        return np.zeros((2, n_all))
    p, want = uni_to_si_states(x), uni_to_si_dyn(dxu, x)
    col = {i: k for k, i in enumerate(live)}
    n = len(live)
    A, b = [], []
    for a in range(n_all):
        for c in range(a + 1, n_all):
            if a not in col and c not in col:
                continue
            d = p[:, a] - p[:, c]
            row = np.zeros(2 * n)
            if a in col:
                row[2 * col[a]:2 * col[a] + 2] = -2 * d
            if c in col:
                row[2 * col[c]:2 * col[c] + 2] = 2 * d
            A.append(row); b.append(GAIN * (d @ d - SAFE ** 2) ** 3)
    for i in live:
        k = 2 * col[i]
        for e, bound, sign in ((1, WALLS[3], 1), (1, WALLS[2], -1), (0, WALLS[1], 1), (0, WALLS[0], -1)):
            row = np.zeros(2 * n); row[k + e] = sign
            A.append(row); b.append(0.4 * GAIN * (sign * (bound - p[e, i]) - SAFE / 2) ** 3)
        for ang in np.arange(8) * np.pi / 4:   # |v| <= VMAX, as an octagon
            row = np.zeros(2 * n); row[k:k + 2] = [np.cos(ang), np.sin(ang)]
            A.append(row); b.append(VMAX * np.cos(np.pi / 8))
    vhat = want[:, live]
    out = np.zeros((2, n_all))
    try:
        sol = qp(sparse(matrix(2.0 * np.eye(2 * n))), matrix(-2.0 * vhat.reshape(-1, order="F")), matrix(np.array(A)), matrix(np.array(b, dtype=float)))
        if sol["status"] != "optimal":
            return out   # no safe velocity found: everyone stops (always safe)
        v = np.reshape(np.array(sol["x"]), (2, n), order="F")
    except Exception:
        return out
    out[:, live] = si_to_uni_proj(v, x[:, live])
    return out


# ---- dock geometry: slots on the half of each dock facing the floor ----------------------------
def slots(k):
    """Slot centres and the direction a carrier faces to set a parcel down there."""
    out = []
    for a in np.linspace(np.radians(120), np.radians(240), SLOTS):
        u = np.array([np.cos(a), np.sin(a)])
        out.append((DOCKS[k] + u * min(DOCK_R - PARCEL - 0.010 * S, 2 * PARCEL + 0.020 * S), -u))
    return out


SLOT_GEOM = [slots(0), slots(1)]


# ---- the shared snapshot: what the camera + radio give every robot -----------------------------
class Snap:
    def __init__(self, t, robots, parcels):
        self.t = t
        self.robots = robots      # list of dicts: id, pos, th, alive, state, task
        self.parcels = parcels    # list of dicts: id, pos, kind, demand, status ('open'/'held'/'placed'), slot


def grip_face(p, parcels):
    """Which side to grip parcel p from (port of grip_dir() in core/brain.h): the side facing its dock
    if the line-up spot there is clear of other parcels and walls, else the next face round, 45 degrees
    at a time, alternating sides. Only parcel positions go in, so every robot picks the same face."""
    z = unit(DOCKS[p["kind"]] - p["pos"])
    back = REACH + PRE + (CRATE - PARCEL if p["demand"] > 1 else 0)
    for k in range(8):
        ang = (1 if k % 2 else -1) * ((k + 1) // 2) * np.pi / 4
        d = np.array([z[0] * np.cos(ang) - z[1] * np.sin(ang), z[0] * np.sin(ang) + z[1] * np.cos(ang)])
        a = p["pos"] - d * back
        inside = abs(a[0]) < 1.6 - 0.10 and abs(a[1]) < 1.0 - 0.10
        clear = all(np.linalg.norm(q["pos"] - a) >= 0.055 + PARCEL + 0.03 for q in parcels
                    if q["id"] != p["id"] and q["status"] == "open")
        if inside and clear:
            return d
    return z


def lineup(p, team_n=1, slot_i=0, parcels=()):
    """Where a robot lines up to grip parcel p, and the direction it then drives in."""
    d = grip_face(p, parcels) if p["demand"] == 1 else unit(DOCKS[p["kind"]] - p["pos"])
    a = p["pos"] - d * (REACH + PRE + (CRATE - PARCEL if p["demand"] > 1 else 0))
    if p["demand"] > 1:   # a pair stands side by side across the crate's face
        left = np.array([-d[1], d[0]])
        a = a + left * (slot_i - (team_n - 1) * 0.5) * PAIR
    return a, d


def job_cost(r, p, parcels):
    a, _ = lineup(p, parcels=parcels)
    return np.linalg.norm(a - r["pos"]) - (STICKY if r["task"] == p["id"] else 0.0)


def allocate(snap, banned):
    """Job allocation, the same on every robot (port of allocate() in core/brain.h).
    Admission control: a pickup starts only if its dock has a free slot nobody has claimed yet,
    plus OVERBOOK staged beside it. A robot holding a parcel keeps it. Then, repeatedly, the
    open parcel whose k cheapest free robots cost least on average gets them (k = robots it
    needs). Ties break by id, so the order robots are listed in can't change the answer."""
    robots = sorted(snap.robots, key=lambda r: r["id"])
    plan = {r["id"]: NONE for r in robots}
    free = {r["id"] for r in robots if r["alive"]}
    team = {p["id"]: 0 for p in snap.parcels}
    room = [0, 0]
    for k in range(2):
        taken = sum(1 for p in snap.parcels if p["status"] == "placed" and p["kind"] == k)
        room[k] = max(0, SLOTS - taken) + OVERBOOK
    for r in robots:
        if r["id"] in free and r["state"] in HOLDING and r["task"] != NONE:
            p = next((p for p in snap.parcels if p["id"] == r["task"] and p["status"] != "placed" and r["id"] in p["holders"]), None)
            if p:
                plan[r["id"]] = p["id"]; free.discard(r["id"]); team[p["id"]] += 1
    for p in snap.parcels:   # carriers already heading to a dock have claimed their room
        if team[p["id"]] and p["status"] != "placed":
            room[p["kind"]] -= 1
    by_id = {r["id"]: r for r in robots}
    while True:
        best = None
        for p in sorted(snap.parcels, key=lambda p: p["id"]):
            need = p["demand"] - team[p["id"]]
            if p["status"] != "open" or need <= 0 or need > len(free) or p["id"] in banned:
                continue
            if not team[p["id"]] and room[p["kind"]] <= 0:
                continue
            cand = sorted(free, key=lambda i: (round(job_cost(by_id[i], p, snap.parcels), 6), i))[:need]
            avg = sum(job_cost(by_id[i], p, snap.parcels) for i in cand) / need
            if best is None or avg < best[0] - 1e-6:
                best = (avg, p, cand)
        if best is None:
            break
        _, p, cand = best
        if not team[p["id"]]:
            room[p["kind"]] -= 1
        for i in cand:
            plan[i] = p["id"]; free.discard(i)
        team[p["id"]] += len(cand)
    return plan


# ---- one robot's brain ---------------------------------------------------------------------------
class Brain:
    def __init__(self, rid):
        self.id = rid
        self.state = IDLE
        self.task = NONE
        self.since = 0.0
        self.plan = {}
        self.slot = None          # (position, inward direction) while carrying
        self.team_slot = 0        # left/right place in a pair

    def set(self, s, t):
        if s != self.state:
            self.state, self.since = s, t


def my_slot(snap, plan, brain, parcel):
    """The slot this carrier fills: carriers to one dock take its free slots in parcel-id order,
    so every robot computes the same assignment. None = dock full: stage beside it."""
    k = parcel["kind"]
    used = {p["slot"] for p in snap.parcels if p["status"] == "placed" and p["kind"] == k}
    free = [i for i in range(SLOTS) if i not in used]
    carried = sorted({p["id"] for p in snap.parcels if p["kind"] == k and p["status"] == "held"})
    rank = carried.index(parcel["id"]) if parcel["id"] in carried else len(carried)
    return free[rank] if rank < len(free) else None


# ---- the experiment --------------------------------------------------------------------------------
def main():
    rng = np.random.default_rng(7)
    init = np.array([[h[0] for h in HOME], [h[1] for h in HOME], [0.0] * N])
    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=init,
                              sim_in_real_time=not FAST)
    si_to_uni = create_si_to_uni_dynamics(linear_velocity_gain=1.0, angular_velocity_limit=1.6)
    ax = r._axes_handle
    fs = determine_font_size(r, 0.05)

    # The floor drawing (what the projector shows): docks, bays, labels.
    colours = ["#2f6fdf", "#e07a1f"]
    for k, d in enumerate(DOCKS):
        ax.add_patch(patches.Circle(d, DOCK_R, fill=False, lw=3, ec=colours[k], zorder=0))
        ax.text(d[0] + 0.05, d[1], "ABCD"[k], color=colours[k], fontsize=fs * 1.4, fontweight="bold", va="center", zorder=0)
    for b, (c, share) in enumerate(BAYS):
        ax.add_patch(patches.Rectangle(c - 0.2, 0.40, 0.40, fill=False, ls="--", lw=2, ec="#777777", zorder=0))
        ax.text(c[0], c[1] + 0.25, f"bay {b + 1}", color="#777777", fontsize=fs, ha="center", zorder=0)
    hud = ax.text(-1.55, 0.92, "", fontsize=fs, va="top", family="monospace", zorder=10)
    rings = [ax.plot([], [], "o", ms=22, mfc="none", mew=3, zorder=3)[0] for _ in range(N)]
    boxes = {}

    brains = [Brain(i + 1) for i in range(N)]
    parcels = []                  # the world's ground truth (the camera's view of it)
    queue = []                    # parcels waiting on a truck for room to land
    next_id, next_truck = 1, 2.0
    crate_done = False
    best, moved_at, cooldown = {}, {}, {}
    stats = dict(delivered=0, landed=0, waits=[], cycles=[], disagreements=0, checks=0, preempted=0,
                 failed_parcel_redelivered=None)
    t, step, snap = 0.0, 0, None
    dead = set()

    def drop_ok(pos, poses):   # nobody drops a parcel within 25 cm of a robot (a bay light on a real floor)
        if any(np.linalg.norm(poses[:2, i] - pos) < 0.25 for i in range(N)):
            return False
        return all(np.linalg.norm(p["pos"] - pos) > 2.6 * CRATE for p in parcels if p["status"] != "placed")

    while t < DURATION:
        x = r.get_poses()

        # Trucks: a parcel every ~6 s at a bay chosen by share, one heavy crate at CRATE_AT.
        if t >= next_truck:
            b = 0 if rng.random() < BAYS[0][1] else 1
            queue.append(dict(bay=b, kind=int(rng.integers(2)), demand=1, t_arr=t))
            next_truck += rng.exponential(6.0)
        if t >= CRATE_AT and not crate_done:
            queue.append(dict(bay=0, kind=1, demand=2, t_arr=t)); crate_done = True
        for q in list(queue):
            if sum(p["status"] != "placed" for p in parcels) >= 14:
                break
            c = BAYS[q["bay"]][0] + rng.uniform(-0.16, 0.16, 2)
            if drop_ok(c, x):
                parcels.append(dict(id=next_id, pos=c, kind=q["kind"], demand=q["demand"], status="open",
                                    slot=None, t_land=t, t_pick=None, placed_at=None, holders=[]))
                next_id += 1; stats["landed"] += 1; queue.remove(q)

        # A robot's battery dies: it stops where it is and drops what it holds.
        if t >= FAIL_AT and FAIL_ROBOT not in dead:
            dead.add(FAIL_ROBOT)
            for p in parcels:
                if FAIL_ROBOT + 1 in p["holders"]:
                    p["status"], p["holders"] = "open", []
                    stats["failed_parcel_redelivered"] = p["id"]

        # The shared snapshot, ~10 Hz. Every brain plans from it.
        if step % SNAP_EVERY == 0:
            rob = [dict(id=b.id, pos=x[:2, b.id - 1].copy(), th=x[2, b.id - 1], alive=(b.id - 1) not in dead,
                        state=b.state, task=b.task) for b in brains]
            snap = Snap(t, rob, [dict(p) for p in parcels if p["status"] != "gone"])
            # Deadlock recovery by preemption (Coffman's four conditions: break "no preemption").
            # Every carrier 20 s without getting 3 cm nearer its dock -> the farthest sets down.
            carriers = [rb for rb in rob if rb["alive"] and rb["state"] in (CARRY,)]
            for rb in carriers:
                p = next(p for p in parcels if p["id"] == rb["task"])
                d = np.linalg.norm(p["pos"] - DOCKS[p["kind"]])
                key = (rb["id"], rb["task"])
                if key not in best or d < best[key] - 0.03:
                    best[key], moved_at[key] = d, t
            stuck = [rb for rb in carriers if t - moved_at[(rb["id"], rb["task"])] > DEADLOCK_S]
            if len(carriers) >= 2 and len(stuck) == len(carriers):
                def from_dock(rb):
                    q = next(p for p in parcels if p["id"] == rb["task"])
                    return np.linalg.norm(q["pos"] - DOCKS[q["kind"]])
                far = max(stuck, key=lambda rb: (from_dock(rb), -rb["id"]))
                p = next(p for p in parcels if p["id"] == far["task"])
                p["status"], p["holders"] = "open", []
                cooldown[p["id"]] = t + COOLDOWN_S
                stats["preempted"] += 1
                for key in moved_at:
                    moved_at[key] = t
            banned = {pid for pid, until in cooldown.items() if t < until}
            plans = []
            for b in brains:   # each brain gets the robots in its own order: itself first
                order = rob[b.id - 1:] + rob[:b.id - 1]
                b.plan = allocate(Snap(t, order, snap.parcels), banned)
                plans.append(b.plan)
            stats["checks"] += 1
            stats["disagreements"] += sum(pl != plans[0] for pl in plans[1:])

        # Each live brain turns its plan into a velocity.
        dxu = np.zeros((2, N))
        si_want = {}
        for b in brains:
            i = b.id - 1
            if i in dead:
                b.set(DEAD, t); continue
            pos, th = x[:2, i], x[2, i]
            hd = np.array([np.cos(th), np.sin(th)])
            nxt = b.plan.get(b.id, NONE)
            if b.state in HOLDING and b.task != NONE and any(p["id"] == b.task and b.id in p["holders"] for p in parcels):
                nxt = b.task                       # a parcel in hand is kept whatever the plan says
            if nxt != b.task:
                b.task = nxt
                b.set(BACKOFF if b.state == INSERT else (GOTO if nxt != NONE else IDLE), t)
            p = next((p for p in parcels if p["id"] == b.task), None)
            lost = b.state in (GRIP, CARRY) and p is not None and b.id not in p["holders"]   # set down to break a deadlock
            if b.task != NONE and (p is None or p["status"] in ("placed", "gone") or lost or
                                   (p["status"] == "held" and b.id not in p["holders"])):
                b.task, p = NONE, None
                b.set(BACKOFF if b.state in HOLDING or b.state == BACKOFF else IDLE, t)
            v, w = 0.0, 0.0
            if b.state == BACKOFF:
                v = -0.06
                if t - b.since > 0.8:
                    b.set(IDLE if b.task == NONE else GOTO, t)
            elif b.task == NONE or p is None:
                b.set(IDLE, t)
                if np.linalg.norm(HOME[i] - pos) > 0.03:
                    si_want[i] = HOME[i]           # park at home, out of the way
            elif b.state in (IDLE, GOTO, ALIGN, WAIT, DOCK):
                team = sorted(rid for rid, pid in b.plan.items() if pid == p["id"])
                b.team_slot = team.index(b.id) if b.id in team else 0
                a, d = lineup(p, max(len(team), p["demand"]), b.team_slot, snap.parcels)
                if b.state in (IDLE, GOTO):
                    b.set(GOTO, t)
                    if np.linalg.norm(a - pos) < 0.04:
                        b.set(ALIGN, t)
                    else:
                        si_want[i] = a
                if b.state == ALIGN:
                    e = wrap(np.arctan2(d[1], d[0]) - th)
                    w = np.clip(2.0 * e, -1.5, 1.5)
                    if abs(e) < 0.12:
                        b.set(WAIT if p["demand"] > 1 else DOCK, t)
                    if np.linalg.norm(a - pos) > 0.08:
                        b.set(GOTO, t)
                if b.state == WAIT:   # a pair docks together (partners' states come from the snapshot)
                    ready = [rb for rb in snap.robots if rb["alive"] and rb["task"] == p["id"] and rb["state"] in (WAIT, DOCK, GRIP)]
                    if len(ready) + (b.state == WAIT and not any(rb["id"] == b.id for rb in ready)) >= p["demand"]:
                        b.set(DOCK, t)
                    elif t - b.since > 25:   # partner never came: line up again
                        b.set(GOTO, t)
                if b.state == DOCK:   # straight in, slowly, until the parcel sits on the gripper
                    ahead = np.dot(p["pos"] - pos, d) - (REACH + (CRATE - PARCEL if p["demand"] > 1 else 0))
                    v = 0.05
                    if ahead < 0.01:
                        b.set(GRIP, t); v = 0.0
                        if b.id not in p["holders"]:
                            p["holders"].append(b.id)
            elif b.state == GRIP:   # gripper closing; a pair waits for both
                if b.id not in p["holders"]:
                    p["holders"].append(b.id)
                if len(p["holders"]) >= p["demand"] and t - b.since > 0.3:
                    if p["t_pick"] is None:
                        p["t_pick"] = t; stats["waits"].append(t - p["t_land"])
                    p["status"] = "held"
                    b.set(CARRY, t)
            if b.state in (CARRY, INSERT) and p is not None:
                if p["demand"] > 1 and b.team_slot != 0 and any(rb["task"] == p["id"] and rb["state"] == INSERT for rb in snap.robots):
                    b.set(INSERT, t)   # the partner started setting it down: go in together
                k = p["kind"]
                si = my_slot(snap, b.plan, b, p)
                if si is None:   # dock full: wait beside it on our own line out from it, ready when a slot frees
                    out = DOCK_R + REACH + PRE + 0.25 * S
                    stage = DOCKS[k] + unit(pos - DOCKS[k]) * out
                    target = stage if np.linalg.norm(pos - DOCKS[k]) < out - 0.02 else pos
                    into = unit(DOCKS[k] - stage)
                else:
                    sp, into = SLOT_GEOM[k][si]
                    target = sp - into * (REACH + PRE + (CRATE - PARCEL if p["demand"] > 1 else 0))
                if p["demand"] > 1:
                    left = np.array([-into[1], into[0]])
                    target = target + left * (b.team_slot - 0.5) * PAIR
                if b.state == CARRY:
                    if np.linalg.norm(target - pos) > 0.04:
                        si_want[i] = target
                    elif si is not None:
                        e = wrap(np.arctan2(into[1], into[0]) - th)
                        w = np.clip(2.0 * e, -1.5, 1.5)
                        if abs(e) < 0.08 and (p["demand"] == 1 or b.team_slot == 0):
                            b.set(INSERT, t)
                if b.state == INSERT and si is not None:
                    sp, into = SLOT_GEOM[k][si]
                    v = 0.05
                    if np.dot(sp - p["pos"], into) < 0.005 and b.team_slot == 0:
                        p["status"], p["slot"], p["placed_at"] = "placed", si, t
                        p["holders"] = []
                        stats["delivered"] += 1; stats["cycles"].append(t - p["t_land"])
                        b.set(BACKOFF, t)
            dxu[:, i] = [v, w]

        # Way-points become velocities: straight toward the goal, steering clear of loose parcels.
        if si_want:
            ids = sorted(si_want)
            dxi = np.zeros((2, N))
            for i in ids:
                pos = x[:2, i]
                to = si_want[i] - pos
                dist = np.linalg.norm(to)
                vel = unit(to) * min(CRUISE, 1.2 * dist + 0.02)
                for p in parcels:
                    if p["status"] == "open" and p["id"] != brains[i].task:
                        away = pos - p["pos"]; dd = np.linalg.norm(away)
                        if dd < 0.18:
                            vel = vel + unit(away) * 0.10 * (0.18 - dd) / 0.18
                dxi[:, i] = vel
            u = si_to_uni(dxi, x)
            for i in ids:
                dxu[:, i] = u[:, i]
        # Wheels cap the mix of driving and turning (|v| + half the axle x |w| within the wheel limit).
        lim = np.abs(dxu[0]) + 0.055 * np.abs(dxu[1])
        dxu = dxu * np.minimum(1.0, 0.18 / np.maximum(lim, 1e-9))
        safe = barrier(dxu, x, dead)
        r.set_velocities(np.arange(N), safe)
        # The Robotarium's own collision rule (centres 2.5 cm ahead, 13.5 cm apart): log any breach.
        c = x[:2] + 0.025 * np.vstack([np.cos(x[2]), np.sin(x[2])])
        for i in range(N):
            for j in range(i + 1, N):
                if np.linalg.norm(c[:, i] - c[:, j]) <= 0.135 and stats.setdefault("too_close", 0) < 12:
                    stats["too_close"] += 1
                    print(f"too close t {t:.1f}: robot {i + 1} ({NAMES[brains[i].state]}) / robot {j + 1} ({NAMES[brains[j].state]})", flush=True)

        # Parcels in hand ride in front of their robot(s).
        for p in parcels:
            if p["status"] in ("held",) or (p["holders"] and p["status"] == "open" and len(p["holders"]) >= p["demand"]):
                hs = [h - 1 for h in p["holders"] if (h - 1) not in dead]
                if hs:
                    c = np.mean([x[:2, h] for h in hs], axis=0)
                    th = x[2, hs[0]]
                    p["pos"] = c + np.array([np.cos(th), np.sin(th)]) * (REACH + (CRATE - PARCEL if p["demand"] > 1 else 0))
            if p["status"] == "placed" and t - p["placed_at"] > SHIP_S:
                p["status"] = "gone"
        if stats["failed_parcel_redelivered"] is not None and not isinstance(stats["failed_parcel_redelivered"], tuple):
            fp = next(p for p in parcels if p["id"] == stats["failed_parcel_redelivered"])
            if fp["status"] in ("placed", "gone"):
                stats["failed_parcel_redelivered"] = (fp["id"], round(t, 1))

        # Draw what the projector shows.
        for p in parcels:
            h = CRATE if p["demand"] > 1 else PARCEL
            if p["id"] not in boxes:
                boxes[p["id"]] = ax.add_patch(patches.Circle(p["pos"], h, fc=colours[p["kind"]],
                                                             ec="k" if p["demand"] > 1 else "none", lw=2, zorder=2))
            boxes[p["id"]].set_center(p["pos"])
            boxes[p["id"]].set_visible(p["status"] != "gone")
        state_col = {IDLE: "#9a9a9a", GOTO: "#d9b400", ALIGN: "#d9b400", WAIT: "#d9b400", DOCK: "#d9b400",
                     GRIP: "#22aa55", CARRY: "#22aa55", INSERT: "#22aa55", BACKOFF: "#9a9a9a", DEAD: "#dd2222"}
        for i, b in enumerate(brains):
            rings[i].set_data([x[0, i]], [x[1, i]])
            rings[i].set_color(state_col[b.state])
        if step % 6 == 0:
            on_floor = sum(p["status"] in ("open", "held") for p in parcels)
            hud.set_text(f"t {t:5.0f} s   delivered {stats['delivered']:3d}   on floor {on_floor:2d}\n"
                         f"plans agree: {N - len(dead)}/{N - len(dead)} robots, {stats['disagreements']} disagreements in {stats['checks']} snapshots"
                         + ("   robot 3: battery dead" if dead else ""))
        if FRAMES and step % FRAME_EVERY == 0:
            os.makedirs(FRAMES, exist_ok=True)
            r._fig.savefig(os.path.join(FRAMES, f"{step // FRAME_EVERY:05d}.png"), dpi=60)

        r.step()
        t += DT
        step += 1
        if step % 900 == 0:   # every ~30 s of experiment time
            print(f"t {t:5.0f} s: delivered {stats['delivered']}, landed {stats['landed']}, disagreements {stats['disagreements']}", flush=True)

    waits, cycles = stats["waits"], stats["cycles"]
    print(f"swarm: {stats['landed']} parcels landed, {stats['delivered']} delivered in {DURATION:.0f} s "
          f"({3600 * stats['delivered'] / DURATION:.0f}/h) | wait for pickup mean {np.mean(waits) if waits else 0:.1f} s | "
          f"landing to dock mean {np.mean(cycles) if cycles else 0:.1f} s | plan disagreements {stats['disagreements']} in "
          f"{stats['checks']} snapshots | deadlocks broken {stats['preempted']} | dropped parcel of the failed robot: "
          f"{stats['failed_parcel_redelivered']}")
    np.savez("swarm_results.npz", delivered=stats["delivered"], landed=stats["landed"], waits=np.array(waits),
             cycles=np.array(cycles), disagreements=stats["disagreements"], checks=stats["checks"],
             preempted=stats["preempted"])
    r.debug()


main()
