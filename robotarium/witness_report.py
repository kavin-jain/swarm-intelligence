#!/usr/bin/env python3
"""The witness experiment's numbers from simulator batches (artifacts of .github/workflows/witness.yml).

  python3 robotarium/witness_report.py DIR       DIR/witness-<mode>-<weak>-<seed>/swarm_results.npz
  python3 robotarium/witness_report.py selftest

  detection     each fault flagged in how many runs (Wilson 95% CI), after how many snapshots of driving,
                against Siegmund's prediction: a priori (driving straight at cruise) and for the shift measured
  false alarms  flags on a robot before any fault of its own; with none, the rule-of-three bound per robot-hour
  A/B           parcels delivered with the witness acting (on) vs only watching (off), paired by seed
  safety        the closest two robots came, too-close steps, compute per step
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sim"))
from stats import bootstrap_ratio, rule_of_three, wilson  # noqa: E402

SNAP_S, CRUISE, CLIP = 3 * 0.033, 0.15, 4.0   # as in swarm_witness.py


def arl(d, h):
    """Average run length of a CUSUM in samples, by Siegmund's approximation (as in swarm_witness.py)."""
    b = h + 1.166
    return b * b if abs(d) < 1e-9 else (math.exp(-2 * d * b) + 2 * d * b - 1) / (2 * d * d)


def load(root):
    runs = []
    for f in sorted(Path(root).glob("witness-*/swarm_results.npz")):
        _, mode, weak, seed = f.parent.name.split("-")
        d = dict(np.load(f))
        d.update(mode=mode, weak_loss=float(weak), seed=int(seed))
        runs.append(d)
    return runs


def fault_times(d):
    return {int(d[k]): float(d[k + "_at"]) for k in ("weak", "hijack") if int(d[k])}


def detection(d, k):
    """None if fault k wasn't injected; else when it was flagged and what theory expected."""
    rid, at = int(d[k]), float(d[k + "_at"])
    if not rid:
        return None
    hit = [f for f in d["flagged"] if int(f[0]) == rid and f[1] >= at]
    out = dict(caught=bool(hit))
    if k == "weak":   # a priori: a robot driving straight at cruise, the pooled noise when the wheel failed
        mu = np.array([d["weak_loss"] / 2 * CRUISE * SNAP_S, d["weak_loss"] * CRUISE * SNAP_S / 0.11]) / d["sig_at_weak"]
        out["prior"] = min(arl(min(m, CLIP) - d["K"], d["H"]) for m in mu)
    if hit:
        tf = hit[0][1]
        res = d["residuals"]   # robot, t, along-track m, heading rad, highest statistic
        mine = res[(res[:, 0] == rid) & (res[:, 1] >= at) & (res[:, 1] <= tf)]
        z = np.clip(mine[:, 2:4] / d["sig"], -CLIP, CLIP)
        out.update(delay=tf - at, snaps=len(mine), given=arl(np.abs(z.mean(axis=0)).max() - d["K"], d["H"]))
    return out


def healthy_hours(d):
    """Robot-hours of driving on which a false alarm could have happened: before each robot's own fault."""
    res, ft = d["residuals"], fault_times(d)
    ok = [t < ft.get(int(r), np.inf) for r, t in res[:, :2]]
    return sum(ok) * SNAP_S / 3600


def fmt(lo_hi):
    return f"{100 * lo_hi[0]:.0f}–{100 * lo_hi[1]:.0f}%"


def report(root):
    runs = load(root)
    if not runs:
        sys.exit(f"no witness-*/swarm_results.npz under {root}")
    print(f"# Witness: {len(runs)} simulator runs from {root}\n")
    d0 = next((d for d in runs if d["mode"] != "exact"), runs[0])   # exact runs have the imperfections off
    print(f"Threshold H = {float(d0['H']):.2f}, K = {float(d0['K'])}; simulator-only lag {float(d0['lag'])} s, "
          f"tracking noise {1000 * float(d0['noise']):.1f} mm, wheel-gain SD {float(d0['gain_sd'])}\n")

    print("## Detection\n")
    print("| Fault | Mode | Power lost | Flagged | 95% CI | Delay, median (max) | Snapshots driving, median | "
          "Siegmund a priori | Siegmund for the shift measured |")
    print("|---|---|---|---|---|---|---|---|---|")
    groups = {}
    for d in runs:
        for k in ("weak", "hijack"):
            r = detection(d, k)
            if r is not None:
                groups.setdefault((k, d["mode"], d["weak_loss"] if k == "weak" else 0), []).append(r)
    for (k, mode, loss), rs in sorted(groups.items()):
        hit = [r for r in rs if r["caught"]]
        delays = [r["delay"] for r in hit]
        prior = [r["prior"] for r in rs if "prior" in r]
        cells = [k, mode, f"{100 * loss:.0f}%" if k == "weak" else "–", f"{len(hit)}/{len(rs)}", fmt(wilson(len(hit), len(rs))),
                 f"{np.median(delays):.1f} s ({max(delays):.1f} s)" if hit else "–",
                 f"{np.median([r['snaps'] for r in hit]):.0f}" if hit else "–",
                 f"{np.median(prior):.1f}" if prior else "–",
                 f"{np.median([r['given'] for r in hit]):.1f}" if hit else "–"]
        print("| " + " | ".join(cells) + " |")

    print("\n## False alarms\n")
    for mode in sorted({d["mode"] for d in runs}):
        ds = [d for d in runs if d["mode"] == mode]
        n = sum(len(d["false_alarms"]) for d in ds)
        hours = sum(healthy_hours(d) for d in ds)
        bound = f", so under {rule_of_three(hours):.2f} per robot-hour (95%)" if n == 0 and hours else ""
        print(f"- {mode}: {n} in {hours:.2f} robot-hours of driving ({len(ds)} runs){bound}")

    on = {d["seed"]: d for d in runs if d["mode"] == "on" and d["weak_loss"] == 0.5}
    off = {d["seed"]: d for d in runs if d["mode"] == "off" and d["weak_loss"] == 0.5}
    seeds = sorted(set(on) & set(off))
    if seeds:
        base, new = [int(off[s]["delivered"]) for s in seeds], [int(on[s]["delivered"]) for s in seeds]
        ratio, lo, hi = bootstrap_ratio(base, new)
        print(f"\n## A/B: the witness acting (on) vs only watching (off), {len(seeds)} paired seeds\n")
        print(f"- delivered: off {np.mean(base):.1f}, on {np.mean(new):.1f} per 300 s; ratio {ratio:.3f} (95% CI {lo:.3f}–{hi:.3f})")
        print("- per seed (off → on): " + ", ".join(f"{b}→{n}" for b, n in zip(base, new)))
        for mode, ds in (("off", off), ("on", on)):
            stolen = [d for d in ds.values() if int(d["stolen"])]
            print(f"- {mode}: a parcel stolen in {len(stolen)}/{len(ds)} runs, reported in "
                  f"{sum(bool(d['stolen_reported']) for d in stolen)} (with only heartbeats: never)")

    print("\n## Safety and compute\n")
    for mode in sorted({d["mode"] for d in runs}):
        ds = [d for d in runs if d["mode"] == mode]
        print(f"- {mode}: closest {100 * min(float(d['min_gap']) for d in ds):.1f} cm (limit 13.5), too-close steps "
              f"{sum(int(d['too_close']) for d in ds)}, compute per step {np.mean([float(d['step_ms']) for d in ds]):.1f} ms mean")
    for d in runs:
        if d["mode"] == "exact":
            print(f"- exact: largest residual {1000 * d['worst'][0]:.1e} mm, {np.degrees(d['worst'][1]):.1e} deg")


def selftest():
    assert abs(arl(0, 5) - 6.166 ** 2) < 1e-9
    assert 900 < arl(-0.5, 5) < 980      # in-control, k = 0.5, h = 5: tables give ~930 for one side
    assert 10.0 < arl(0.5, 5) < 10.8     # a 1-sigma shift: tables give 10.4
    rng = np.random.default_rng(1)       # and the update used in swarm_witness.py agrees, by simulation
    lengths = []
    for _ in range(2000):
        s, n = 0.0, 0
        while s <= 5:
            s, n = max(0.0, s + rng.normal(1.0) - 0.5), n + 1
        lengths.append(n)
    assert abs(np.mean(lengths) - arl(0.5, 5)) < 1.0, np.mean(lengths)
    print("selftest ok")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "selftest":
        selftest()
    elif len(sys.argv) == 2:
        report(sys.argv[1])
    else:
        sys.exit(__doc__)
