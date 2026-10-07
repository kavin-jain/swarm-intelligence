#!/usr/bin/env python3
"""Queueing model of the saturated floor: how many parcels an hour N robots can sort, from first principles.

Each robot cycles: drive to a bay -> pick a parcel up -> carry it -> set it down in a dock slot -> again.
That is a closed queueing network (Zou, de Koster et al., Transportation Science 2021 model robotic
sorters the same way), solved exactly with Mean Value Analysis (Reiser & Lavenberg, J. ACM 1980);
multi-server stations use Seidmann's approximation. Stations:
  drive   delay (infinite server): empty + loaded driving, measured with ONE robot (no traffic)
  bay b   pickup at bay b (line up, dock, grip): c_bay robots can pick there at once; share_b of parcels
  dock k  set down in dock k: c_k usable slots; half the parcels each
Two capacity bounds sit on top (operational analysis, Denning & Buzen 1978):
  slots   a slot is held for the set-down plus the 5 s until the parcel ships: X <= c_k / hold / share_k
  inbound parcels can only be dropped at a bay while no robot is within 300 mm (the measured landing
          rate with robots crowding the bays); given, not derived
Every input is measured separately (one-robot timings from `SIM_BUDGET=1 build/sim`, slot counts from the
dock geometry); nothing is fitted to the curve it predicts.

  python3 sim/mva.py            prints the model for both benchmark floors
"""


def mva(n_max, delay, stations):
    """Exact MVA, single class. stations: list of (demand s, servers). Returns X(n) for n = 1..n_max."""
    q = [0.0] * len(stations)
    xs = []
    for n in range(1, n_max + 1):
        r = []
        for (d, c), qk in zip(stations, q):
            r.append(d / c * (1 + qk) + d * (c - 1) / c)   # Seidmann: a c-server station = 1 fast server + a delay
        x = n / (delay + sum(r))
        q = [x * rk for rk in r]
        xs.append(x)
    return xs


# Measured with one robot, 6 saturated shifts (SIM_BUDGET=1), seconds per parcel.
FLOORS = {
    "3 x 2 m": dict(drive=5.2 + 6.8 - 1.5, pick=1.15 + 2.08 + 1.14, drop=1.5 + 0.1 + 0.7, slots=(5, 5)),
    "1.5 x 1 m": dict(drive=2.3 + 5.9 - 1.5, pick=1.03 + 1.59 + 1.18, drop=1.5 + 0.1 + 0.7, slots=(2, 2), inbound=853),
}
# inbound: parcels/h that could land with robots on the floor, measured at saturation (853 landed with 4 robots,
# 841 with 10). The 3 x 2 m floor's bays never limited it in the measured range.
BAYS = (0.65, 0.30, 0.05)   # shares of parcels per bay, averaged over the shift layouts (0.6/0.3/0.1 and 0.7/0.3)
C_BAY = 2                   # robots that can pick at one bay at once (parcels land up to +-150 mm apart)
SHIP_S = 5                  # a delivered parcel leaves its slot this long after it lands there


def model(f, n_max=10):
    stations = [(s * f["pick"], C_BAY) for s in BAYS] + [(0.5 * f["drop"], c) for c in f["slots"]]
    xs = mva(n_max, f["drive"], stations)
    bound = min(c / (f["drop"] + SHIP_S) / 0.5 for c in f["slots"])
    if "inbound" in f:
        bound = min(bound, f["inbound"] / 3600)
    return xs, bound


if __name__ == "__main__":
    for name, f in FLOORS.items():
        xs, sb = model(f)
        print(f"{name}: one robot {3600 * xs[0]:.0f}/h; capacity bound (dock slots, inbound) {3600 * sb:.0f}/h")
        print("  robots  " + "  ".join(f"{n:5d}" for n in range(1, 11)))
        print("  model/h " + "  ".join(f"{3600 * min(x, sb):5.0f}" for x in xs))
