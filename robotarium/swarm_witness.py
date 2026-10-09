"""Every robot is a witness: catching a broken or hijacked robot with zero messages, on the Robotarium.

The sorting experiment of swarm_sort.py (same floor, parcels, planner and timeline: see that file),
plus a witness. Every robot already runs the same planner on the same shared snapshot, so every robot
can work out what every other robot's wheels were told to do, and so where each robot should be at
the next snapshot. A robot that keeps ending up somewhere else is broken or not following the plan,
even though the camera sees it and its heartbeat looks normal (all that robot_alive() checks).

Each snapshot, for every moving robot: residual = where it is - where its own commands put it.
Residuals are scaled by the fleet's pooled noise and fed to CUSUM change detectors (Page 1954); a
statistic over the threshold flags the robot. The inputs are the snapshot and the commands every
robot can compute from it, so every robot flags the same robot at the same snapshot: no messages and
no vote. (Here the 8 brains share one process, so the witness runs once for all of them.)

Faults, injected by this script (the brains aren't told; heartbeats stay normal):
   90 s  weak wheel: the first robot seen driving loses half its left wheel's power
  150 s  battery death, as in swarm_sort.py (the case heartbeats already catch)
  210 s  hijack: the first robot carrying a parcel drives it off to the far wall (a theft)

MODE  exact  no faults: the witness must predict every robot to within 0.1 mm (simulator check)
      off    faults; the witness only watches and reports (the control)
      on     faults; a flagged robot stops being trusted: no more work for it, and its position is
             treated like a dead robot's (dock slots it blocks are skipped). One that still obeys
             sets its parcel down and stops; what one that doesn't holds is reported stolen.
"""
import os
import time
from collections import deque

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
MODE = os.environ.get("SWARM_MODE", "off")    # before uploading: "off" for run W1, "on" for run W2
SIM_ONLY = FAST and MODE != "exact"            # simulator-only imperfections, never on the real robots:
LAG = float(os.environ.get("SWARM_LAG", 0)) if SIM_ONLY else 0.0       # wheels follow commands with this lag (s)
NOISE = float(os.environ.get("SWARM_NOISE", 0)) if SIM_ONLY else 0.0   # tracking noise (m; heading: NOISE / 0.1 rad)
GAIN_SD = float(os.environ.get("SWARM_GAIN", 0)) if SIM_ONLY else 0.0  # spread of the robots' wheel gains (SD)

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
SLOTS = 6             # set-down spots per dock, round the ring, as dock_slots() gives for a 150 mm dock
OVERBOOK = 2      # carriers allowed beyond a dock's free slots (they stage beside it)
SHIP_S = 5.0      # a delivered parcel leaves its slot after this long
STICKY = 0.15     # m of cost a robot saves by keeping its current job (anti-thrash)
CRUISE = 0.15         # m/s, NOT to scale: ours drive 0.26 m/s (2.2 body-lengths/s); the Robotarium's max is 0.2,
                      # so times here run about 1.6x longer than on our floor
FAIL_AT = 150.0      # from here the first robot seen carrying a parcel has its battery "die" (robot 3 if none by 210 s):
                     # it sets the parcel down, backs off 10 cm, and stops for good. (Backing off is forced by the
                     # Robotarium: its 19 cm bubble would keep every robot from reaching a parcel right at a dead robot.)
BACK_OFF_S = 1.7     # 10 cm at 6 cm/s
CRATE_AT = 45.0                          # a heavy crate arrives at bay 1
DEADLOCK_S, COOLDOWN_S = 20.0, 30.0      # deadlock recovery: see watch_carriers()
RESULTS_S = 10.0     # after the shift each results page is projected this long: the Robotarium returns only the video
FAULTS = MODE != "exact"
WEAK_AT = 90.0       # from here the first robot seen driving loses WEAK of its left wheel's power (robot 2 if none by 150 s)
WEAK = float(os.environ.get("SWARM_WEAK", 0.5))
HIJACK_AT = 210.0    # from here the first robot carrying a parcel is hijacked: it takes the parcel to the far wall
LOOT = np.array([0.40, -0.85])   # where the thief takes it: clear of the bays, docks and home poses

IDLE, GOTO, ALIGN, WAIT, DOCK, GRIP, CARRY, INSERT, BACKOFF, DEAD = range(10)
NAMES = ["idle", "goto", "align", "wait", "dock", "grip", "carry", "insert", "backoff", "dead"]
HOLDING = (GRIP, CARRY, INSERT)
NONE = -1


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.array([1.0, 0.0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def wheel_cap(dxu):
    """Scale each robot's (v, w) down so neither wheel passes the Robotarium's limit
    (|v| + half the axle x |w| <= 0.2 m/s, kept at 0.18). Scaling keeps the turn's shape, and a
    safe command scaled toward zero stays safe."""
    lim = np.abs(dxu[0]) + 0.055 * np.abs(dxu[1])
    return dxu * np.minimum(1.0, 0.18 / np.maximum(lim, 1e-9))


def drive(dxu):
    """What the Robotarium does to a command before it moves the robot (_threshold() in rps): each
    wheel's speed clamped to 0.2 m/s, wheels 0.11 m apart."""
    wl, wr = np.clip(dxu[0] - 0.055 * dxu[1], -0.2, 0.2), np.clip(dxu[0] + 0.055 * dxu[1], -0.2, 0.2)
    return np.vstack([(wl + wr) / 2, (wr - wl) / 0.11])


def integrate(x, vels):
    """Where poses x end up moving at these velocities, one step each, integrated as the Robotarium does."""
    x = x.copy()
    for u in vels:
        x[0] += DT * u[0] * np.cos(x[2]); x[1] += DT * u[0] * np.sin(x[2]); x[2] += DT * u[1]
    return x


def roll(x, cmds):
    """Where poses x end up after these commands."""
    return integrate(x, [drive(u) for u in cmds])


def weaken(u, loss):
    """Command u (v, w) as carried out by a robot whose left wheel lost `loss` of its power."""
    wl, wr = (u[0] - 0.055 * u[1]) * (1 - loss), u[0] + 0.055 * u[1]
    return np.array([(wl + wr) / 2, (wr - wl) / 0.11])


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


def barrier(dxu, x, fixed, moving=None):
    """moving: the commands the fixed robots will carry out (default: they stand still). A robot solved
    alone against the others' known velocities then takes the whole of every avoidance with them."""
    n_all = x.shape[1]
    live = [i for i in range(n_all) if i not in fixed]
    if not live:
        return np.zeros((2, n_all))
    p, want = uni_to_si_states(x), uni_to_si_dyn(dxu, x)
    w = np.zeros((2, n_all)) if moving is None else uni_to_si_dyn(moving, x)
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
            A.append(row); b.append(GAIN * (d @ d - SAFE ** 2) ** 3 - (2 * d @ w[:, c] if c not in col else 0)
                                    + (2 * d @ w[:, a] if a not in col else 0))
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
    """Slot centres and the direction a carrier faces to set a parcel down there (dock_slots(): a ring
    starting on the side facing -x)."""
    rho = min(DOCK_R - PARCEL - 0.010 * S, 2 * PARCEL + 0.020 * S)
    out = []
    for i in range(SLOTS):
        u = np.array([np.cos(np.pi + i * 2 * np.pi / SLOTS), np.sin(np.pi + i * 2 * np.pi / SLOTS)])
        out.append((DOCKS[k] + u * rho, -u))
    return out


SLOT_GEOM = [slots(0), slots(1)]


def usable(k, i, dead_at):
    """A slot can be filled if its line-up point is on the floor and no dead robot sits on the way in.
    (Found here: a robot that died beside dock A left carriers assigned slots they could never reach.
    The C++ brain's my_slot() already skips such slots; its admission count doesn't yet.)"""
    sp, into = SLOT_GEOM[k][i]
    stage = sp - into * (REACH + PRE)
    if abs(stage[0]) > 1.6 - 0.10 or abs(stage[1]) > 1.0 - 0.10:
        return False
    return all(np.linalg.norm(c - q) >= SAFE + 0.02 for c in dead_at for q in (stage, (stage + sp) / 2))


# ---- the shared snapshot: what the camera + radio give every robot -----------------------------
class Snap:
    def __init__(self, t, robots, parcels):
        self.t = t
        self.robots = robots      # list of dicts: id, pos, th, alive, state, task
        self.parcels = parcels    # list of dicts: id, pos, kind, demand, status ('open'/'held'/'placed'), slot


def grip_face(p, parcels, dead_at=()):
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
                    if q["id"] != p["id"] and q["status"] == "open") and \
            all(np.linalg.norm(c - a) >= SAFE + 0.02 for c in dead_at)   # a dead robot is an obstacle, as in the brain
        if inside and clear:
            return d
    return z


def lineup(p, team_n=1, slot_i=0, parcels=(), dead_at=()):
    """Where a robot lines up to grip parcel p, and the direction it then drives in."""
    d = grip_face(p, parcels, dead_at) if p["demand"] == 1 else unit(DOCKS[p["kind"]] - p["pos"])
    a = p["pos"] - d * (REACH + PRE + (CRATE - PARCEL if p["demand"] > 1 else 0))
    if p["demand"] > 1:   # a pair stands side by side across the crate's face
        left = np.array([-d[1], d[0]])
        a = a + left * (slot_i - (team_n - 1) * 0.5) * PAIR
    return a, d


def job_cost(r, p, a):
    """Distance from robot r to parcel p's line-up point a, less STICKY if p is r's current job."""
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
    dead_at = [r["pos"] for r in robots if not r["alive"]]
    for k in range(2):
        taken = {p["slot"] for p in snap.parcels if p["status"] == "placed" and p["kind"] == k}
        room[k] = sum(1 for i in range(SLOTS) if i not in taken and usable(k, i, dead_at)) + OVERBOOK
    for r in robots:
        if r["id"] in free and r["state"] in HOLDING and r["task"] != NONE:
            p = next((p for p in snap.parcels if p["id"] == r["task"] and p["status"] != "placed" and r["id"] in p["holders"]), None)
            if p:
                plan[r["id"]] = p["id"]; free.discard(r["id"]); team[p["id"]] += 1
    for p in snap.parcels:   # carriers already heading to a dock have claimed their room
        if team[p["id"]] and p["status"] != "placed":
            room[p["kind"]] -= 1
    by_id = {r["id"]: r for r in robots}
    look = {}   # each parcel's line-up point: the same for every robot, so worked out once per plan

    def cost(i, p):
        if p["id"] not in look:
            look[p["id"]] = lineup(p, parcels=snap.parcels, dead_at=dead_at)[0]
        return job_cost(by_id[i], p, look[p["id"]])

    while True:
        best = None
        for p in sorted(snap.parcels, key=lambda p: p["id"]):
            need = p["demand"] - team[p["id"]]
            if p["status"] != "open" or need <= 0 or need > len(free) or p["id"] in banned:
                continue
            if not team[p["id"]] and room[p["kind"]] <= 0:
                continue
            cand = sorted(free, key=lambda i: (round(cost(i, p), 6), i))[:need]
            avg = sum(cost(i, p) for i in cand) / need
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
    # Robots still free, lowest id first, each join the cheapest load still short of hands: a crate
    # gets its first robot now, lining up to wait for the second, so a stream of 1-robot parcels
    # can't starve it (the second pass of allocate() in core/brain.h; seen: crates never picked up).
    for rid in sorted(free):
        best = None
        for p in sorted(snap.parcels, key=lambda p: p["id"]):
            if p["status"] != "open" or team[p["id"]] >= p["demand"] or p["id"] in banned:
                continue
            if not team[p["id"]] and room[p["kind"]] <= 0:
                continue
            c = cost(rid, p)
            if best is None or c < best[0] - 1e-6:
                best = (c, p)
        if best:
            p = best[1]
            if not team[p["id"]]:
                room[p["kind"]] -= 1
            plan[rid] = p["id"]; team[p["id"]] += 1
    return plan


# ---- one robot's brain ---------------------------------------------------------------------------
class Brain:
    def __init__(self, rid):
        self.id = rid
        self.state = IDLE
        self.task = NONE
        self.since = 0.0
        self.plan = {}
        self.slot = None          # slot index locked while setting a parcel down (slot_ in core/brain.h)
        self.team_slot = 0        # left/right place in a pair

    def set(self, s, t):
        if s != self.state:
            self.state, self.since = s, t


def my_slot(snap, plan, brain, parcel):
    """The slot this carrier fills: carriers to one dock take its free slots in parcel-id order,
    so every robot computes the same assignment. None = dock full: stage beside it."""
    k = parcel["kind"]
    used = {p["slot"] for p in snap.parcels if p["status"] == "placed" and p["kind"] == k}
    dead_at = [r["pos"] for r in snap.robots if not r["alive"]]
    free = [i for i in range(SLOTS) if i not in used and usable(k, i, dead_at)]
    carried = sorted({p["id"] for p in snap.parcels if p["kind"] == k and p["status"] == "held"})
    rank = carried.index(parcel["id"]) if parcel["id"] in carried else len(carried)
    return free[rank] if rank < len(free) else None


# ---- the witness: every robot predicts every robot and tests the gap --------------------------------
SNAP_S = SNAP_EVERY * DT


def arl(d, h):
    """Average run length of a CUSUM, in samples, by Siegmund's approximation. Each sample adds d on
    average (in noise units, K already taken off): d < 0 gives samples to a false alarm, d > 0 to detection."""
    b = h + 1.166
    return b * b if abs(d) < 1e-9 else (np.exp(-2 * d * b) + 2 * d * b - 1) / (2 * d * d)


K = 0.5              # reference value, tuned to a shift of 1 noise unit (K = shift / 2)
CLIP = 4.0           # one residual counts at most 4 noise units: a single tracking glitch can't flag a robot
# Threshold: 4 statistics per robot (along-track and heading, each up and down), under 1 false alarm per robot
# per 8 h of driving if the noise were independent and Gaussian. Run W1 measures how true that is.
H = next(h / 100 for h in range(100, 5000) if arl(-K, h / 100) >= 4 * 8 * 3600 / SNAP_S)
SIG_MIN = np.array([0.001, np.radians(0.5)])   # noise floor (m, rad): the simulator has no noise at all
WARM_S = 10.0        # the first 10 s only measure the noise
WINDOW_S = 30.0      # pooled noise: the robust spread of the last 30 s of residuals
GHOST_S = 1.5        # a robot's ghost: where its own commands of the last 1.5 s would have put it
TAUS = (0.0, 0.03, 0.06, 0.1, 0.15, 0.2, 0.3)   # wheel lags tried (s): the witness keeps the one that best explains
                     # the whole fleet's motion (wheels follow commands as a first-order lag), the same on every robot


def weak_theory(sig):
    """Snapshots theory expects a weak wheel to go unflagged, for a robot driving straight at cruise:
    it falls short by WEAK / 2 of its speed and turns WEAK x speed / wheel spacing too far left."""
    mu = np.array([WEAK / 2 * CRUISE * SNAP_S, WEAK * CRUISE * SNAP_S / 0.11]) / sig
    return min(arl(min(m, CLIP) - K, H) for m in mu)


class Witness:
    def __init__(self):
        self.prev, self.cmds = None, []          # poses at the last snapshot, every robot's commands since
        self.v = np.zeros((len(TAUS), 2, N))     # what each lag says the wheels are doing now
        self.vels = []                           # and did at each step since the last snapshot
        self.tau = 0                             # index of the lag that explains the fleet best
        self.S = np.zeros((N, 4))                # CUSUM statistics: along-track +/-, heading +/-
        self.wins = [deque(maxlen=int(WINDOW_S / SNAP_S)) for _ in TAUS]
        self.flagged = {}                        # robot index -> (time, statistic that crossed H)
        self.peak = np.zeros(N)                  # each robot's highest statistic
        self.raw = [[] for _ in range(N)]        # each robot's (time, residual m, rad, highest statistic): calibration
        self.worst = np.zeros(2)                 # largest residual (m, rad): the exact-mode check

    def spread(self, j):
        """Robust spread (1.4826 x median absolute deviation) of lag j's residuals over the window."""
        if not self.wins[j]:
            return np.zeros(2)
        e = np.concatenate(self.wins[j])
        return 1.4826 * np.median(np.abs(e - np.median(e, axis=0)), axis=0)

    def scale(self):
        return np.maximum(self.spread(self.tau), SIG_MIN)

    def command(self, u):
        """Every step: the commands every robot can work out from the snapshot."""
        self.cmds.append(u.copy())
        u = drive(u)
        for j, tau in enumerate(TAUS):
            self.v[j] += DT / (tau + DT) * (u - self.v[j])
        self.vels.append(self.v.copy())

    def update(self, x, t, alive):
        """x: this snapshot's poses. alive: indices of robots whose heartbeat is heard."""
        if self.prev is not None and len(self.cmds) == SNAP_EVERY:
            hd = np.vstack([np.cos(self.prev[2]), np.sin(self.prev[2])])
            u = np.max(np.abs(np.array(self.cmds)), axis=0)
            moving = [i for i in alive if (u[0, i] > 0.03 or u[1, i] > 0.3) and i not in self.flagged]   # parked: no evidence
            es = []
            for j in range(len(TAUS)):
                pred = integrate(self.prev, [v[j] for v in self.vels])
                es.append(np.vstack([np.sum((x[:2] - pred[:2]) * hd, axis=0), wrap(x[2] - pred[2])]))   # along m, heading rad
                if moving:
                    self.wins[j].append(es[j][:, moving].T)
            self.tau = min(range(len(TAUS)), key=lambda j: np.sum(self.spread(j) / SIG_MIN))
            e, sig = es[self.tau], self.scale()
            for i in moving:
                self.worst = np.maximum(self.worst, np.abs(e[:, i]))
                z = np.clip(e[:, i] / sig, -CLIP, CLIP)
                if t >= WARM_S:
                    self.S[i] = np.maximum(0, self.S[i] + np.array([z[0], -z[0], z[1], -z[1]]) - K)
                    self.peak[i] = max(self.peak[i], self.S[i].max())
                    if self.S[i].max() > H:
                        self.flagged[i] = (t, int(np.argmax(self.S[i])))
                self.raw[i].append((t, e[0, i], e[1, i], self.S[i].max()))
        self.prev, self.cmds, self.vels = x.copy(), [], []


# ---- the experiment --------------------------------------------------------------------------------
def main():
    rng = np.random.default_rng(7)   # the trucks: the same parcels at the same times in every run
    if os.environ.get("SWARM_SEED"):   # the simulator's own randomness (where robots start): one per batch run
        np.random.seed(int(os.environ["SWARM_SEED"]))
    init = np.array([[h[0] for h in HOME], [h[1] for h in HOME], [0.0] * N])
    r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=init,
                              sim_in_real_time=not FAST)
    si_to_uni = create_si_to_uni_dynamics(linear_velocity_gain=1.0, angular_velocity_limit=1.6)
    ax = r._axes_handle
    fs = determine_font_size(r, 0.05)

    # The floor drawing (what the projector shows): docks, bays, labels.
    colours = ["#2f6fdf", "#e07a1f"]
    PURPLE = "#8e44ad"            # a robot the witness flagged, and its ghost
    for k, d in enumerate(DOCKS):
        ax.add_patch(patches.Circle(d, DOCK_R, fill=False, lw=3, ec=colours[k], zorder=0))
        ax.text(d[0] + 0.05, d[1], "ABCD"[k], color=colours[k], fontsize=fs * 1.4, fontweight="bold", va="center", zorder=0)
    for b, (c, share) in enumerate(BAYS):
        ax.add_patch(patches.Rectangle(c - 0.2, 0.40, 0.40, fill=False, ls="--", lw=2, ec="#777777", zorder=0))
        ax.text(c[0], c[1] + 0.25, f"bay {b + 1}", color="#777777", fontsize=fs, ha="center", zorder=0)
    hud = ax.text(-1.55, 0.92, "", fontsize=fs, va="top", family="monospace", zorder=10)
    rings = [ax.plot([], [], "o", ms=22, mfc="none", mew=3, zorder=3)[0] for _ in range(N)]
    ghosts = [ax.add_patch(patches.Circle((0, 0), 0.055, fill=False, ls="--", lw=1.5, ec="#888888", zorder=1)) for _ in range(N)]
    boxes = {}

    brains = [Brain(i + 1) for i in range(N)]
    parcels = []                  # the world's ground truth (the camera's view of it)
    queue = []                    # parcels waiting on a truck for room to land
    next_id, next_truck = 1, 2.0
    crate_done = False
    best, moved_at, cooldown = {}, {}, {}
    stall = {}                    # robot -> [where it last made 10 cm of progress, when, detour until]
    stats = dict(delivered=0, landed=0, waits=[], cycles=[], disagreements=0, checks=0, preempted=0,
                 crate_at=np.nan, crate_by=[], failed=None, failed_at=np.nan, failed_parcel=None, redelivered_at=np.nan)
    t, step, snap = 0.0, 0, None
    dead = set()
    wit = Witness()
    past = deque(maxlen=int(GHOST_S / DT))   # every step's (poses, commands) for the last GHOST_S: the ghosts
    retire = {}                   # robot id -> when it began backing off to stop for good (battery, or flagged and still obeying)
    stats.update(weak=None, weak_at=np.nan, hijack=None, hijack_at=np.nan, sig_at_weak=SIG_MIN)
    imperfect = np.random.default_rng(1000 + int(os.environ.get("SWARM_SEED", 0)))   # simulator-only noise
    gain = 1 + GAIN_SD * imperfect.standard_normal(N)
    wheels = np.zeros((2, N))     # what the wheels are doing (lags the command by LAG in the simulator)
    distrust = set()              # robot ids the swarm stopped trusting (MODE on)

    def drop_ok(pos, poses):   # nobody drops a parcel within 25 cm of a robot (a bay light on a real floor)
        if any(np.linalg.norm(poses[:2, i] - pos) < 0.25 for i in range(N)):
            return False
        return all(np.linalg.norm(p["pos"] - pos) > 2.6 * CRATE for p in parcels if p["status"] in ("open", "held"))

    while t < DURATION:
        t0 = time.perf_counter()
        xt = r.get_poses()            # where the robots are
        x = xt.copy()                 # what the tracking reports: the same, except for the simulator's added noise
        if NOISE:
            x[:2] += imperfect.normal(0, NOISE, (2, N)); x[2] += imperfect.normal(0, NOISE / 0.1, N)

        # Trucks: a parcel every ~6 s at a bay chosen by share, one heavy crate at CRATE_AT.
        if t >= next_truck:
            b = 0 if rng.random() < BAYS[0][1] else 1
            queue.append(dict(bay=b, kind=int(rng.integers(2)), demand=1, t_arr=t))
            next_truck += rng.exponential(6.0)
        if t >= CRATE_AT and not crate_done:
            queue.insert(0, dict(bay=0, kind=1, demand=2, t_arr=t)); crate_done = True
        crate_waiting = any(q["demand"] > 1 for q in queue)
        for q in list(queue):
            if crate_waiting and q["demand"] == 1 and q["bay"] == 0:
                continue   # a truck unloading a crate holds its bay until the crate is down
            if sum(p["status"] in ("open", "held") for p in parcels) >= 14:   # the snapshot holds 16 loads
                break
            c = BAYS[q["bay"]][0] + rng.uniform(-0.16, 0.16, 2)
            if drop_ok(c, xt):
                parcels.append(dict(id=next_id, pos=c, kind=q["kind"], demand=q["demand"], status="open",
                                    slot=None, t_land=t, t_pick=None, placed_at=None, holders=[]))
                next_id += 1; stats["landed"] += 1; queue.remove(q)

        # A robot's battery dies: it stops where it is and drops what it holds.
        # (Not a robot already given a fault, or stopped: each fault gets its own robot.)
        if t >= FAIL_AT and stats["failed"] is None:
            out = {stats["weak"], stats["hijack"]} | set(retire)
            carrying = [b.id for b in brains if b.state == CARRY and b.id not in out and
                        any(p["id"] == b.task and p["demand"] == 1 for p in parcels)]
            if carrying or t >= FAIL_AT + 60:
                stats["failed"], stats["failed_at"] = (carrying or [i for i in (3, 4, 5) if i not in out])[0], t
                retire[stats["failed"]] = t
                for p in parcels:
                    if stats["failed"] in p["holders"]:
                        p["status"], p["holders"] = "open", []
                        stats["failed_parcel"] = p["id"]

        # The faults the witness has to find. The brains aren't told, and the heartbeats stay normal.
        if FAULTS and t >= WEAK_AT and stats["weak"] is None:
            driving = [b.id for b in brains if b.state in (GOTO, CARRY) and b.id not in retire]
            if driving or t >= WEAK_AT + 60:
                stats["weak"], stats["weak_at"], stats["sig_at_weak"] = (driving or [2])[0], t, wit.scale()
        if FAULTS and t >= HIJACK_AT and stats["hijack"] is None:
            out = {stats["weak"], stats["failed"]} | set(retire)
            thief = [b.id for b in brains if b.state == CARRY and b.id not in out and
                     any(p["id"] == b.task and p["demand"] == 1 for p in parcels)]
            if not thief and t >= HIJACK_AT + 60:   # nobody carrying: one drives off empty-handed
                thief = [b.id for b in brains if b.state not in HOLDING and b.id not in out]
            if thief:
                stats["hijack"], stats["hijack_at"] = thief[0], t
                stats["stolen"] = next((p["id"] for p in parcels if thief[0] in p["holders"]), None)

        # The shared snapshot, ~10 Hz. Every brain plans from it.
        if step % SNAP_EVERY == 0:
            # The witness first: every robot checks every robot heard on the radio against its own commands.
            wit.update(x, t, [b.id - 1 for b in brains if b.id != stats["failed"] and b.id - 1 not in dead])
            distrust = {i + 1 for i in wit.flagged} if MODE == "on" else set()
            for rid in distrust - set(retire):
                if rid != stats["hijack"]:   # a flagged robot that still obeys sets its parcel down and stops
                    retire[rid] = t
                    for p in parcels:
                        if rid in p["holders"]:
                            p["status"], p["holders"] = "open", []
            for p in parcels:   # one that doesn't: what it holds is reported stolen, and no dock slot waits for it
                if p["status"] == "held" and any(h in distrust and h not in retire for h in p["holders"]):
                    p["status"] = "stolen"
            rob = [dict(id=b.id, pos=x[:2, b.id - 1].copy(), th=x[2, b.id - 1],
                        alive=b.id != stats["failed"] and b.id not in distrust, state=b.state, task=b.task) for b in brains]
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
                if far["id"] != stats["hijack"]:   # a thief told to set its parcel down doesn't
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
            if b.id in retire:   # battery dying, or flagged: back away from the parcel just set down, then stop
                b.task = NONE; b.set(DEAD, t)   # (a flagged robot can still drive: it goes home to be repaired first)
                if t - retire[b.id] < BACK_OFF_S:
                    dxu[:, i] = [-0.06, 0.0]
                elif b.id == stats["failed"] or np.linalg.norm(HOME[i] - x[:2, i]) < 0.04:
                    dead.add(i)
                else:
                    si_want[i] = HOME[i]
                continue
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
                a, d = lineup(p, max(len(team), p["demand"]), b.team_slot, snap.parcels,
                              [rb["pos"] for rb in snap.robots if not rb["alive"]])
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
                if b.state == DOCK:   # straight in, slowly, holding the line, until the parcel sits on the gripper
                    ahead = np.dot(p["pos"] - pos, d) - (REACH + (CRATE - PARCEL if p["demand"] > 1 else 0))
                    v = 0.05
                    w = np.clip(2.0 * wrap(np.arctan2(d[1], d[0]) - th), -1.5, 1.5)
                    if t - b.since > 6.0:   # ~1.5 s when it works: pushed off the line, so line up again
                        b.set(GOTO, t); v, w = 0.0, 0.0
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
                k = p["kind"]
                si = b.slot if b.state == INSERT else my_slot(snap, b.plan, b, p)   # locked once setting down
                if p["demand"] > 1 and b.team_slot != 0 and b.state == CARRY and si is not None and \
                        any(rb["task"] == p["id"] and rb["state"] == INSERT for rb in snap.robots):
                    b.set(INSERT, t); b.slot = si   # the partner started setting it down: go in together
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
                            b.set(INSERT, t); b.slot = si
                if b.state == INSERT and si is not None:
                    sp, into = SLOT_GEOM[k][si]
                    v = 0.05
                    w = np.clip(2.0 * wrap(np.arctan2(into[1], into[0]) - th), -1.5, 1.5)   # hold the line in
                    if np.dot(sp - p["pos"], into) < 0.005 and b.team_slot == 0:
                        p["status"], p["slot"], p["placed_at"] = "placed", si, t
                        if p["demand"] > 1:
                            stats["crate_at"], stats["crate_by"] = t, sorted(p["holders"])
                        if p["id"] == stats["failed_parcel"]:
                            stats["redelivered_at"] = t
                        p["holders"] = []
                        stats["delivered"] += 1; stats["cycles"].append(t - p["t_land"])
                        b.set(BACKOFF, t)
            dxu[:, i] = [v, w]

        # Way-points become velocities: straight toward the goal, steering clear of loose parcels.
        # Barrier certificates can freeze two robots face to face for good (a known deadlock of the method;
        # the Robotarium's own initialize() breaks it with random way-points; seen here in 3 of 8 runs).
        # Ours: a robot that hasn't made 10 cm in 5 s while driving veers 70 degrees right for 2 s:
        # everyone keeps right, so two frozen robots pass each other.
        for i in [i for i in stall if i not in si_want]:
            del stall[i]
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
                ref = stall.setdefault(i, [pos.copy(), t, -1.0])
                if np.linalg.norm(pos - ref[0]) > 0.10:
                    ref[0], ref[1] = pos.copy(), t
                elif t - ref[1] > 5.0:
                    ref[1], ref[2] = t, t + 2.0
                    stats["detours"] = stats.get("detours", 0) + 1
                if t < ref[2]:
                    c, sn = np.cos(-1.22), np.sin(-1.22)
                    vel = np.array([c * vel[0] - sn * vel[1], sn * vel[0] + c * vel[1]])
                dxi[:, i] = vel
            u = si_to_uni(dxi, x)
            for i in ids:
                dxu[:, i] = u[:, i]
        # Wheels cap the mix of driving and turning, before the barrier and after it (its turns can be sharp).
        # A distrusted robot that won't stop is an obstacle: the others take the whole of each avoidance.
        fixed = dead | {rid - 1 for rid in distrust if rid not in retire}
        safe = wheel_cap(barrier(wheel_cap(dxu), x, fixed))
        wit.command(safe)              # the commands every robot can work out from the snapshot
        past.append((x.copy(), safe.copy()))
        sent = safe.copy()
        # A faulty robot moves as it moves. Its own barrier, solved against everyone else's actual velocities, makes
        # it take the whole of every avoidance: the Robotarium's safety rule holds and nobody else's command changes.
        act = sent.copy()                 # how every robot will actually move

        def alone(i, want):
            u = np.zeros((2, N)); u[:, i] = want
            return wheel_cap(barrier(wheel_cap(u), x, set(range(N)) - {i}, moving=act))[:, i]
        if stats["weak"] is not None:     # weak wheel: these robots' wheels are fine, so it is sent what a weak one would do
            i = stats["weak"] - 1
            act[:, i] = sent[:, i] = alone(i, weaken(sent[:, i], WEAK))
        if stats["hijack"] is not None:   # hijacked: takes its parcel to the far wall
            i = stats["hijack"] - 1
            to = LOOT - x[:2, i]
            want = np.zeros((2, N))
            if np.linalg.norm(to) > 0.03:
                want[:, i] = unit(to) * min(CRUISE, 1.2 * np.linalg.norm(to))
            act[:, i] = sent[:, i] = alone(i, si_to_uni(want, x)[:, i])
        if LAG:
            wheels += DT / (LAG + DT) * (sent - wheels)
            sent = wheels.copy()
        if GAIN_SD:
            sent = sent * gain
        r.set_velocities(np.arange(N), sent)
        # The Robotarium's own collision rule (centres 2.5 cm ahead, 13.5 cm apart): count every breach,
        # log the first 12, and keep the closest any two robots came.
        c = xt[:2] + 0.025 * np.vstack([np.cos(xt[2]), np.sin(xt[2])])
        for i in range(N):
            for j in range(i + 1, N):
                gap = np.linalg.norm(c[:, i] - c[:, j])
                stats["min_gap"] = min(stats.get("min_gap", 9.0), gap)
                if gap <= 0.135:
                    stats["too_close"] = stats.get("too_close", 0) + 1
                    if stats["too_close"] <= 12:
                        print(f"too close t {t:.1f}: robot {i + 1} ({NAMES[brains[i].state]}) / robot {j + 1} ({NAMES[brains[j].state]})", flush=True)

        # Parcels in hand ride in front of their robot(s).
        for p in parcels:
            if p["status"] in ("held", "stolen") or (p["holders"] and p["status"] == "open" and len(p["holders"]) >= p["demand"]):
                hs = [h - 1 for h in p["holders"] if (h - 1) not in dead]
                if hs:
                    c = np.mean([xt[:2, h] for h in hs], axis=0)
                    th = xt[2, hs[0]]
                    p["pos"] = c + np.array([np.cos(th), np.sin(th)]) * (REACH + (CRATE - PARCEL if p["demand"] > 1 else 0))
            if p["status"] == "placed" and t - p["placed_at"] > SHIP_S:
                p["status"] = "gone"

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
        ghost = roll(past[0][0], [u for _, u in list(past)[:-1]])   # where each robot's commands put it, open loop
        for i, b in enumerate(brains):
            rings[i].set_data([xt[0, i]], [xt[1, i]])
            rings[i].set_color(PURPLE if i in wit.flagged else state_col[b.state])
            ghosts[i].set_center(ghost[:2, i])
            ghosts[i].set_edgecolor(PURPLE if i in wit.flagged else "#888888")
            ghosts[i].set_visible(i not in dead and b.id != stats["failed"])
        if step % 6 == 0:
            on_floor = sum(p["status"] in ("open", "held") for p in parcels)
            hud.set_text(f"t {t:5.0f} s   delivered {stats['delivered']:3d}   on floor {on_floor:2d}\n"
                         f"plans agree: {N - len(dead)}/{N - len(dead)} robots, {stats['disagreements']} disagreements in {stats['checks']} snapshots"
                         + (f"\nrobot {stats['failed']}: battery dead" if stats["failed"] and stats["failed"] - 1 in dead else "")
                         + "".join(f"\nrobot {stats[k]}: {what} at {stats[k + '_at']:.0f} s" for k, what in
                                   (("weak", "wheel fault injected"), ("hijack", f"hijacked, took parcel {stats.get('stolen')}")) if stats[k])
                         + "\nwitness: " + (", ".join(f"robot {i + 1} flagged at {f[0]:.1f} s" for i, f in sorted(wit.flagged.items()))
                                             or "every robot doing what it was told"))
        stats.setdefault("loop", []).append(time.perf_counter() - t0)   # our compute per step; the Robotarium steps every 33 ms
        if FRAMES and step % FRAME_EVERY == 0:
            os.makedirs(FRAMES, exist_ok=True)
            r._fig.savefig(os.path.join(FRAMES, f"{step // FRAME_EVERY:05d}.png"), dpi=60)

        r.step()
        t += DT
        step += 1
        if step % 900 == 0:   # every ~30 s of experiment time
            print(f"t {t:5.0f} s: delivered {stats['delivered']}, landed {stats['landed']}, disagreements {stats['disagreements']}", flush=True)
            print("    " + " | ".join(f"r{b.id} {NAMES[b.state]} {'p' + str(b.task) if b.task != NONE else '-'} "
                                     f"({x[0, b.id - 1]:+.2f},{x[1, b.id - 1]:+.2f})" for b in brains), flush=True)

    waits, cycles = stats["waits"], stats["cycles"]
    print(f"swarm: {stats['landed']} parcels landed, {stats['delivered']} delivered in {DURATION:.0f} s "
          f"({3600 * stats['delivered'] / DURATION:.0f}/h) | wait for pickup mean {np.mean(waits) if waits else 0:.1f} s | "
          f"landing to dock mean {np.mean(cycles) if cycles else 0:.1f} s | plan disagreements {stats['disagreements']} in "
          f"{stats['checks']} snapshots | deadlocks broken {stats['preempted']} | detours {stats.get('detours', 0)}")
    ms = 1000 * np.array(stats["loop"])
    print(f"compute per step: mean {ms.mean():.1f} ms, 99th percentile {np.percentile(ms, 99):.1f} ms, max {ms.max():.1f} ms "
          f"(the Robotarium steps every {1000 * DT:.0f} ms)")
    print(f"crate: delivered at {stats['crate_at']:.0f} s by robots {stats['crate_by']} | failed robot: {stats['failed']} "
          f"at {stats['failed_at']:.0f} s, its parcel {stats['failed_parcel']} delivered by another robot at "
          f"{stats['redelivered_at']:.0f} s")
    # The witness: what it flagged, and the calibration the next run's threshold is set from (every robot's
    # residuals before any fault of its own).
    fault_at = {stats[k]: stats[k + "_at"] for k in ("weak", "hijack") if stats[k]}
    false = sorted(i + 1 for i, f in wit.flagged.items() if f[0] < fault_at.get(i + 1, np.inf))

    def caught(k):
        f = wit.flagged.get(stats[k] - 1)
        return f"flagged {f[0] - stats[k + '_at']:.1f} s later" if f and f[0] >= stats[k + "_at"] else "NOT flagged"
    cal = [np.array(wit.raw[i]).reshape(-1, 4) for i in range(N)]
    cal = [a[a[:, 0] < fault_at.get(i + 1, np.inf)] for i, a in enumerate(cal)]
    every = np.concatenate(cal)[:, 1:3] if sum(map(len, cal)) else np.zeros((1, 2))
    sig = np.maximum(1.4826 * np.median(np.abs(every - np.median(every, axis=0)), axis=0), SIG_MIN)

    def lag1(a):   # lag-1 autocorrelation of one robot's residuals, both channels (0: independent, as CUSUM's theory assumes)
        a = a[:, 1:3] - a[:, 1:3].mean(axis=0)
        den = (a * a).sum(axis=0)
        return (a[1:] * a[:-1]).sum(axis=0) / np.maximum(den, 1e-30) if len(a) > 2 else np.zeros(2)
    n = np.array([len(a) for a in cal], dtype=float)
    ac = sum(n[i] * lag1(cal[i]) for i in range(N)) / max(n.sum(), 1)
    tail = np.percentile(np.abs(every) / sig, 99.9, axis=0)
    bias = [a[:, 1:3].mean(axis=0) / sig if len(a) else np.full(2, np.nan) for a in cal]
    peak = [a[:, 3].max() if len(a) else np.nan for a in cal]
    deg = np.degrees
    if FAULTS:
        wl = [f"witness ({MODE}): flags a robot whose motion doesn't match its commands",
              (f"  wheel fault, robot {stats['weak']} at {stats['weak_at']:.0f} s: {caught('weak')} "
               f"(theory, driving: {weak_theory(stats['sig_at_weak']) * SNAP_S:.1f} s)") if stats["weak"] else "  wheel fault: not injected",
              (f"  hijack, robot {stats['hijack']} at {stats['hijack_at']:.0f} s: {caught('hijack')} "
               f"(fastest possible {arl(CLIP - K, H) * SNAP_S:.1f} s)") if stats["hijack"] else "  hijack: not injected",
              f"  parcel {stats.get('stolen')} stolen: " + ("reported" if any(p['status'] == 'stolen' for p in parcels)
                                                           else "the heartbeats still say it is being delivered")]
    else:
        wl = [f"witness (exact): largest residual {1000 * wit.worst[0]:.1e} mm, {deg(wit.worst[1]):.1e} deg"]
    wl.append(f"  false alarms {len(false)}" + (f" (robots {false})" if false else "") + f", threshold {H:.1f}")
    # The results, projected for RESULTS_S per page with every robot stopped, so the camera records them.
    def at(v):
        return "not delivered" if np.isnan(v) else f"at {v:.0f} s"
    lines = [f"RESULTS  {DURATION:.0f} s, {N} robots, no leader",
             f"delivered        {stats['delivered']} of {stats['landed']} landed ({3600 * stats['delivered'] / DURATION:.0f}/h)",
             f"wait for pickup  mean {np.mean(waits) if waits else 0:.1f} s, 90th pct {np.percentile(waits, 90) if waits else 0:.1f} s",
             f"landing to dock  mean {np.mean(cycles) if cycles else 0:.1f} s",
             f"crate, 2 robots  delivered {at(stats['crate_at'])}",
             f"robot {stats['failed']} died at {stats['failed_at']:.0f} s, its parcel delivered {at(stats['redelivered_at'])}",
             f"plan disagreements {stats['disagreements']} in {stats['checks']} snapshots",
             f"closest robots   {100 * stats.get('min_gap', 0):.1f} cm (limit 13.5), too-close steps {stats.get('too_close', 0)}",
             f"deadlocks broken {stats['preempted']}, detours {stats.get('detours', 0)}",
             f"compute per step mean {ms.mean():.1f} ms, max {ms.max():.1f} ms"] + wl
    cal_lines = [f"WITNESS CALIBRATION  {int(n.sum())} snapshots of moving robots, before any fault",
                 f"noise per snapshot  along-track {1000 * sig[0]:.2f} mm, heading {deg(sig[1]):.2f} deg; wheel lag learned {TAUS[wit.tau]:.2f} s",
                 f"lag-1 correlation   along {ac[0]:+.2f}, heading {ac[1]:+.2f}  (0 = independent)",
                 f"99.9th pct |resid.| along {tail[0]:.1f}, heading {tail[1]:.1f} noise units (Gaussian 3.3)",
                 "mean residual per robot, noise units (robots 1-8)",
                 "  along   " + " ".join(f"{b[0]:+5.1f}" for b in bias),
                 "  heading " + " ".join(f"{b[1]:+5.1f}" for b in bias),
                 f"highest statistic before any fault (threshold {H:.1f})",
                 "          " + " ".join(f"{p:5.1f}" for p in peak),
                 f"largest residual {1000 * wit.worst[0]:.1f} mm, {deg(wit.worst[1]):.1f} deg; mode {MODE}"]
    print("\n".join(lines + [""] + cal_lines), flush=True)
    box = ax.text(0, 0, "", fontsize=fs * 1.1, family="monospace", ha="center", va="center", zorder=20,
                  bbox=dict(facecolor="white", edgecolor="black", alpha=0.95, boxstyle="round,pad=0.6"))
    k = 0
    for page in (lines, cal_lines):
        box.set_text("\n".join(page))
        for _ in range(int(RESULTS_S / DT)):
            r.get_poses()
            r.set_velocities(np.arange(N), np.zeros((2, N)))
            if FRAMES and (step + k) % FRAME_EVERY == 0:
                r._fig.savefig(os.path.join(FRAMES, f"{(step + k) // FRAME_EVERY:05d}.png"), dpi=60)
            r.step()
            k += 1
    np.savez("swarm_results.npz", delivered=stats["delivered"], landed=stats["landed"], waits=np.array(waits),
             cycles=np.array(cycles), disagreements=stats["disagreements"], checks=stats["checks"],
             preempted=stats["preempted"], crate_at=stats["crate_at"], crate_by=np.array(stats["crate_by"]),
             failed_robot=stats["failed"] or 0, failed_at=stats["failed_at"], redelivered_at=stats["redelivered_at"],
             min_gap=stats.get("min_gap", np.nan), too_close=stats.get("too_close", 0), step_ms=ms.mean(),
             mode=MODE, H=H, K=K, tau=TAUS[wit.tau], weak_loss=WEAK, lag=LAG, noise=NOISE, gain_sd=GAIN_SD,
             weak=stats["weak"] or 0, weak_at=stats["weak_at"], hijack=stats["hijack"] or 0, hijack_at=stats["hijack_at"],
             sig_at_weak=stats["sig_at_weak"], sig=sig, stolen=stats.get("stolen") or 0,
             stolen_reported=any(p["status"] == "stolen" for p in parcels), worst=wit.worst, false_alarms=np.array(false, dtype=int),
             flagged=np.array([(i + 1, f[0], f[1]) for i, f in sorted(wit.flagged.items())]).reshape(-1, 3),
             residuals=np.array([(i + 1, *row) for i in range(N) for row in wit.raw[i]]).reshape(-1, 5))
    r.debug()
    if MODE == "exact":   # the prediction must match the simulator exactly, or every number above is suspect
        assert wit.worst[0] < 1e-4 and wit.worst[1] < 1e-4 and not wit.flagged, f"witness exact check FAILED: {wit.worst}"
        print("witness exact check passed", flush=True)


main()
