#!/usr/bin/env python3
"""Is a change really better? Statistics for two versions run on the same seeds (stdlib only).

  python3 sim/stats.py rate K N                  Wilson 95% interval for K successes out of N
                                                 (and the rule-of-three bound when nothing failed)
  python3 sim/stats.py runs BASE.txt NEW.txt     same runs, pass/fail paired: McNemar exact test
  python3 sim/stats.py shifts BASE.txt NEW.txt   same shifts, parcels delivered paired: ratio with a
                                                 bootstrap 95% CI, and Wald's SPRT for a >= 5% gain
  python3 sim/stats.py report DIR                 markdown comparison of a cloud run (.github/workflows/bench.yml):
                                                 DIR/<side>-<suite>/out.txt and DIR/cap-<side>-f<floor>-n<robots>/out.txt
  python3 sim/stats.py selftest

Inputs are simulator output with SIM_RUNS=1: lines "run <suite> <id> pass|fail <class> <stall s>"
and "shift <n> delivered <d> landed <a> ...". Pairing is by (suite, id) or shift number, so both
files must come from the same seeds.
"""
import math
import random
import sys


def wilson(k, n, z=1.96):
    """95% interval for a success rate (Wilson score: honest near 0 and 1, unlike p +- 2 SE)."""
    if n == 0:
        return 0.0, 1.0
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, c - h), min(1.0, c + h)


def rule_of_three(n):
    """Zero failures in n runs: the failure rate is below 3/n with 95% confidence."""
    return 3 / n


def mcnemar(b, c):
    """Exact two-sided p-value. b = runs only the old version passed, c = only the new one passed."""
    n = b + c
    if n == 0:
        return 1.0
    tail = sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def bootstrap_ratio(base, new, reps=10000, seed=1):
    """sum(new)/sum(base) over paired samples, with a percentile 95% CI from resampled pairs."""
    rng = random.Random(seed)
    n = len(base)
    ratios = []
    for _ in range(reps):
        idx = [rng.randrange(n) for _ in range(n)]
        sb = sum(base[i] for i in idx)
        ratios.append(sum(new[i] for i in idx) / sb if sb else float("inf"))
    ratios.sort()
    return sum(new) / sum(base), ratios[int(0.025 * reps)], ratios[int(0.975 * reps)]


def sprt(base, new, delta=0.05, alpha=0.05, beta=0.05):
    """Wald's sequential probability ratio test on paired relative differences d = (new-base)/mean(base).
    H0: mean d = 0 (no gain), H1: mean d = delta. Normal model, variance estimated from the sample.
    Returns (llr, lower bound, upper bound, verdict)."""
    m = sum(base) / len(base)
    d = [(y - x) / m for x, y in zip(base, new)]
    n = len(d)
    mu = sum(d) / n
    var = sum((x - mu) ** 2 for x in d) / (n - 1) if n > 1 else 0.0
    lo, hi = math.log(beta / (1 - alpha)), math.log((1 - beta) / alpha)
    if var == 0:
        llr = math.inf if mu >= delta / 2 else -math.inf
    else:
        llr = (delta * sum(d) - n * delta * delta / 2) / var
    verdict = "H1: better by >= {:.0%}".format(delta) if llr >= hi else "H0: not better" if llr <= lo else "undecided: run more"
    return llr, lo, hi, verdict


def read_runs(path):
    out = {}
    for line in open(path):
        f = line.split()
        if len(f) >= 4 and f[0] == "run":
            out[(f[1], f[2])] = f[3] == "pass"
    return out


def read_shifts(path):
    out = {}
    for line in open(path):
        f = line.split()
        if len(f) >= 4 and f[0] == "shift":
            out[int(f[1])] = int(f[3])
    return out


def summary(path, key):
    """The suite's own summary line starting with `key` (e.g. 'inbound shifts:'), or ''."""
    for line in open(path):
        if line.startswith(key):
            return line.strip()
    return ""


def report(d):
    import os
    import re
    out = ["# Benchmark: dev vs base", ""]
    for side in ("dev", "base"):   # dense runs in slices (dense0..dense4): merge them into one suite
        parts = sorted(x for x in os.listdir(d) if x.startswith(f"{side}-dense") and x != f"{side}-dense")
        if parts:
            os.makedirs(os.path.join(d, f"{side}-dense"), exist_ok=True)
            with open(os.path.join(d, f"{side}-dense", "out.txt"), "w") as out_f:
                for x in parts:
                    out_f.write(open(os.path.join(d, x, "out.txt")).read())
    f = lambda side, shard: os.path.join(d, f"{side}-{shard}", "out.txt")
    suites = [s for s in ("seeds", "random", "walls", "dense") if os.path.exists(f("dev", s)) and os.path.exists(f("base", s))]
    if suites:
        out += ["## Delivery (same seeds, paired)", "", "| Suite | base pass | dev pass | only base passed | only dev passed | McNemar p |", "|---|---|---|---|---|---|"]
        tb, td, ob, od, n_all = 0, 0, 0, 0, 0
        for s_ in suites:
            a, b = read_runs(f("base", s_)), read_runs(f("dev", s_))
            keys = sorted(set(a) & set(b))
            pa, pb = sum(a[k] for k in keys), sum(b[k] for k in keys)
            x, y = sum(a[k] and not b[k] for k in keys), sum(b[k] and not a[k] for k in keys)
            tb, td, ob, od, n_all = tb + pa, td + pb, ob + x, od + y, n_all + len(keys)
            out.append(f"| {s_} | {pa}/{len(keys)} | {pb}/{len(keys)} | {x} | {y} | {mcnemar(x, y):.3g} |")
        lb, hb = wilson(tb, n_all)
        ld, hd = wilson(td, n_all)
        out.append(f"| **all** | {tb}/{n_all} [{lb:.2%}, {hb:.2%}] | {td}/{n_all} [{ld:.2%}, {hd:.2%}] | {ob} | {od} | **{mcnemar(ob, od):.3g}** |")
        out += ["", "Failure classes and stalls:", ""]
        for s_ in suites:
            for side in ("base", "dev"):
                line = summary(f(side, s_), "seeded failure classes") or summary(f(side, s_), "random failure classes") or \
                       summary(f(side, s_), "walls failure classes") or summary(f(side, s_), "dense failure classes")
                if s_ == "dense":   # run in many slices: count the classes from the run lines
                    runs = [l.split() for l in open(f(side, s_)) if l.startswith("run ")]
                    by = {c: sum(r[3] == "fail" and r[4] == c for r in runs) for c in "ABCDEF"}
                    out.append(f"- {side} dense failure classes: " + ", ".join(f"{c} {n}" for c, n in by.items()) +
                               f" | longest stall {max((float(r[5]) for r in runs), default=0):.0f} s")
                elif line:
                    out.append(f"- {side} {line}")
        out.append("")
    shifts = [s for s in ("inbound1", "inbound2") if os.path.exists(f("dev", s)) and os.path.exists(f("base", s))]
    if shifts:
        out += ["## Shifts (same trucks, paired)", "", "| Floor | parcels dev/base [95% CI] | SPRT (>= 5% gain) | base wait | dev wait |", "|---|---|---|---|---|"]
        for s_ in shifts:
            a, b = read_shifts(f("base", s_)), read_shifts(f("dev", s_))
            keys = sorted(set(a) & set(b))
            r, lo, hi = bootstrap_ratio([a[k] for k in keys], [b[k] for k in keys])
            verdict = sprt([a[k] for k in keys], [b[k] for k in keys])[3]
            w = lambda side: (re.search(r"wait for pickup mean ([\d.]+ s)", summary(f(side, s_), "inbound shifts:")) or [None, "?"])[1]
            out.append(f"| {'1.5 x 1 m' if s_ == 'inbound1' else '3 x 2 m'} | {r:.3f} [{lo:.3f}, {hi:.3f}] | {verdict} | {w('base')} | {w('dev')} |")
        out.append("")
    caps = sorted({tuple(int(v) for v in re.findall(r"f(\d+)-n(\d+)", x)[0]) for x in os.listdir(d) if x.startswith("cap-dev-")}) if os.path.isdir(d) else []
    if caps:
        out += ["## Capacity (saturated floor, 24 ten-minute shifts per point)", "",
                "| Floor | Robots | base parcels/h | dev parcels/h | dev/base [95% CI] | SPRT | dev scaling efficiency |", "|---|---|---|---|---|---|---|"]
        one = {}
        for fl, n in caps:
            a, b = read_shifts(os.path.join(d, f"cap-base-f{fl}-n{n}", "out.txt")), read_shifts(os.path.join(d, f"cap-dev-f{fl}-n{n}", "out.txt"))
            keys = sorted(set(a) & set(b))
            if not keys:
                continue
            hours = len(keys) * 600 / 3600
            xb, xd = sum(a[k] for k in keys) / hours, sum(b[k] for k in keys) / hours
            r, lo, hi = bootstrap_ratio([a[k] for k in keys], [b[k] for k in keys])
            if n == 1:
                one[fl] = xd
            eff = f"{xd / (n * one[fl]):.0%}" if fl in one else "?"
            out.append(f"| {'1.5 x 1 m' if fl == 1 else '3 x 2 m'} | {n} | {xb:.0f} | {xd:.0f} | {r:.3f} [{lo:.3f}, {hi:.3f}] | {sprt([a[k] for k in keys], [b[k] for k in keys])[3]} | {eff} |")
        out.append("")
    return "\n".join(out)


def selftest():
    lo, hi = wilson(0, 10)
    assert lo == 0 and abs(hi - 0.2775) < 1e-3
    lo, hi = wilson(477, 480)
    assert 0.98 < lo < 0.99 < 0.997 < hi
    assert abs(rule_of_three(1220) - 0.00246) < 1e-5
    assert mcnemar(0, 6) == 2 / 64 and mcnemar(3, 3) == 1.0
    r, lo, hi = bootstrap_ratio([100] * 20, [110] * 20)
    assert abs(r - 1.1) < 1e-9 and abs(lo - 1.1) < 1e-9
    noisy = random.Random(3)
    base = [100 + noisy.gauss(0, 5) for _ in range(40)]
    assert sprt(base, [x * 1.10 + noisy.gauss(0, 2) for x in base])[3].startswith("H1")
    assert sprt(base, [x + noisy.gauss(0, 2) for x in base])[3].startswith("H0")
    print("stats selftest: all passed")


def main(argv):
    if len(argv) == 2 and argv[1] == "selftest":
        return selftest()
    if len(argv) == 3 and argv[1] == "report":
        print(report(argv[2]))
        return
    if len(argv) == 4 and argv[1] == "rate":
        k, n = int(argv[2]), int(argv[3])
        lo, hi = wilson(k, n)
        print(f"{k}/{n} = {k / n:.2%}, 95% CI [{lo:.2%}, {hi:.2%}]" + (f"; failure rate < {rule_of_three(n):.2%} (rule of three)" if k == n else ""))
        return
    if len(argv) == 4 and argv[1] == "runs":
        a, b = read_runs(argv[2]), read_runs(argv[3])
        keys = sorted(set(a) & set(b))
        only_old = sum(a[k] and not b[k] for k in keys)
        only_new = sum(b[k] and not a[k] for k in keys)
        pa, pb = sum(a[k] for k in keys), sum(b[k] for k in keys)
        print(f"{len(keys)} paired runs: old {pa} pass, new {pb} pass | only old passed {only_old}, only new passed {only_new} | McNemar p = {mcnemar(only_old, only_new):.3g}")
        for name, k in (("old", pa), ("new", pb)):
            lo, hi = wilson(k, len(keys))
            print(f"  {name}: {k / len(keys):.2%} [{lo:.2%}, {hi:.2%}]")
        return
    if len(argv) == 4 and argv[1] == "shifts":
        a, b = read_shifts(argv[2]), read_shifts(argv[3])
        keys = sorted(set(a) & set(b))
        base, new = [a[k] for k in keys], [b[k] for k in keys]
        r, lo, hi = bootstrap_ratio(base, new)
        llr, l0, l1, verdict = sprt(base, new)
        print(f"{len(keys)} paired shifts: parcels new/old = {r:.3f} [{lo:.3f}, {hi:.3f}] | SPRT LLR {llr:.2f} in ({l0:.2f}, {l1:.2f}): {verdict}")
        return
    sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv)
