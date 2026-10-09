# Leaderless, infrastructure-free parcel sorting with low-cost robots

**Kavin Jain** · technical report, October 2026 · code, simulator and every benchmark in this repository

> **Status of the evidence.** Every number below comes from the simulator in `sim/`, run on GitHub's Linux runners (image pinned to ubuntu-24.04; §7.1 explains why the platform matters). It runs the *same* robot code as the firmware, with camera noise, radio loss, motor mismatch and grip failures modelled. Nothing here has yet been measured on the physical robots. The protocol for that run is in §9, and the tool that scores it (`satellite/kpi.py`) is already written and tested.

## Abstract

Robotic parcel sorters in today's warehouses are fast, but each one needs a central planning server, codes stuck to the floor, and induction stations where people place every parcel onto a robot. That puts them out of reach of small warehouses.

This report describes a sorting swarm that needs none of the three:

- **One overhead camera** (a phone) sees the floor.
- **One radio** relays a shared snapshot of it.
- **Every robot** runs the same deterministic planner on that snapshot, so the robots agree on who does what without a leader and without negotiating.
- **No induction station:** parcels are picked straight off the floor, wherever they land.

A small set of "floor manager" rules runs inside the same planner:

- staging beside full docks;
- learned arrival hotspots;
- recruiting a second robot for heavy crates.

**Results in simulation:**

- **99.35% of 1,380 benchmark runs** fully delivered: 480/480 seeded, 498/500 random, 200/200 against walls, 193/200 on dense floors.
  - The previous version (v2) scores 97.3% on the same floors; McNemar p = 8 × 10⁻⁷.
  - The gain comes from classifying every failure and fixing each cause (§7), then breaking carrier deadlocks by preemption (§7.1).
- zero robot–robot collisions in the 480 seeded runs. The 9 remaining failures: 6 runs where every parcel arrived but robots touched, 2 crates flagged for a person, and 1 gridlock;
- the manager rules cut the mean wait before pickup by **49%** (p90 by 61%) at equal energy.

**On real robots:** one run of the coordination layer on 8 robots at Georgia Tech's Robotarium [20] delivered 46 parcels in 300 s, where 8 runs of the Robotarium's simulator predicted 44.0 (SD 3.9). The 8 planners never disagreed in 3,029 snapshots (§9.1).

The report also covers what did *not* work:
- a benchmark artefact that made the swarm look like it collapses at high density;
- two rejected methods: docks that move toward demand (upper bound +20%), and capping the working fleet (saves energy, not throughput).

## 1. The problem

Three pieces of infrastructure make robotic sorting expensive.

1. **A central planner.** Kiva/Amazon Robotics fleets are coordinated by a central job manager (Wurman, D'Andrea & Mountz, *AI Magazine* 29(1), 2008). Recent gains still come from central optimisation:
   - deep reinforcement learning deciding which robot goes first gives about +25% throughput (Zheng, Ma, Araki, Chen & Wu, *JAIR*, March 2026);
   - tuning planner parameters online with extremum-seeking control gives +5–8% (Tokekar et al., Amazon Robotics, arXiv 2608.21533, August 2026).
2. **Floor infrastructure.** Kiva robots localise with 2-inch codes stuck to the floor every 40–60 inches, read by a downward camera (Robohub teardown of the Kiva robot).
3. **Induction stations.** Commercial sorting robots receive each parcel from a person at a fixed station. Research on these systems optimises which destination goes to which drop-off point, and robot routing:
   - Guo, Feng & Yu, arXiv 2310.17753;
   - dynamic destination reassignment, *Transportation Science*, doi 10.1287/trsc.2023.0458. The abstract reports +35% in one case company; the paper is paywalled, so only the abstract was read.

This project asks what is left when all three are removed. Parcels are dropped anywhere in a receiving area. One camera sees them. A swarm of cheap robots, with no leader, picks them up and sorts them into docks.

## 2. System

| Layer | What it does | Where |
|---|---|---|
| Sees | Phone camera + ArUco markers. Corner tags give the pixel→mm homography; one tag per robot gives position and heading. Parcels carry tags too (the tag id gives the parcel and its dock, read in the same detection pass); colour detection is a fallback. | `satellite/` |
| Relays | Gateway ESP32 merges camera frames and robot heartbeats into one snapshot (≤ 250 B, one ESP-NOW packet) and broadcasts it at 15 Hz. It makes no decisions. | `core/world.h` |
| Decides | Every robot runs `core/brain.h`. The same header compiles into the firmware, the unit tests, the simulator and the website (WebAssembly). | `core/brain.h` |

### 2.1 Lockstep coordination

Each robot runs the same deterministic allocator on the same snapshot. Ties are broken by fixed ids, and there are no random choices. So every robot computes the *whole* plan and acts on its own line of it. Video games use the same idea, deterministic lockstep, to keep thousands of units in sync over a network.

- **Radio traffic:** each robot only transmits a 10 Hz status heartbeat (state, current job, help request). There are no negotiation rounds, and no leader that can fail.
- **When robots disagree:** if radio loss gives two robots different snapshots, they can briefly disagree. The next snapshot reconciles them. Tested by a scenario that drops 25% of packets: 60/60 seeds pass.
- **Unit test:** the allocation is tested to be identical regardless of snapshot order.

### 2.2 Picking from the floor, no induction

- **Grip side:** a robot chooses a grip side reachable with room to turn, and lines up.
- **Tug check:** it grips, then backs off a few centimetres. A held parcel follows; a missed one doesn't. This lets the camera tell holding from pushing without a gripper sensor.
- **Reachability:** only parcels reachable from some live robot are assigned (a BFS over a 5 cm grid, with parcels and dead robots as obstacles). A packed area is therefore cleared from the outside in.
- **Heavy crates:** a robot whose wheels drive while the crate doesn't move asks for help. The crate's headcount rises and two robots carry it side by side, ant-style. If every robot has tried, the crate is flagged for a person.

### 2.3 Floor-manager rules (all inside the same deterministic planner)

| Rule | What it does | Measured effect |
|---|---|---|
| Dock admission + staging | Start a pickup only if its dock has a free slot. Up to 2 extra carriers wait beside a full dock, ready the moment it clears. | Mean wait for pickup −54% on the 1.5 m² floor (Table 2). 3+ extra carriers caused collisions on dense floors, so 2 is the limit. |
| Learned arrival hotspots | Each robot keeps a decaying map of where parcels land. The nearest idle robot waits beside the busiest spot if that saves ≥ 5 s of driving. | Neutral on the 1.5 m² floor; helps on a 6 m² floor (Table 2). |
| Pair timeouts | A robot waiting for a partner more than 6 s lets go and lines up again. | Removed two-robot deadlocks: seeded 477 → 480/480 when introduced. 477/480 today, at the faster drive speeds adopted since. |
| Motor slew limit | Wheels speed up at most 8 command units/s (0 to full in 125 ms); slowing is instant. | 45,576 → 0 hard motor starts. Delivery unchanged. |

## 3. Evaluation method

- **Simulator realism:**
  - the real wire format;
  - camera noise of 3 mm / ~1°;
  - 80 ms camera latency;
  - ±8% motor mismatch;
  - radio loss (0–25% by scenario);
  - a 5% gripper miss rate;
  - docks that ship parcels 5 s after delivery.
- **Energy model:** electronics 0.5 W, TT motors ~2.5 W each (×1.6 when stalled), L298N at 75% efficiency, gripper 3 W to grab and 1.05 W to hold. It is a model until calibrated with a power meter.
- **Fixed seeds:** every variant runs on the same seeds. In the shift benchmark, truck arrivals and parcel landing positions use separate random streams, so a variant can't change which trucks arrive. That separation fixed one early false positive.
- **Shift benchmark (`--inbound N`):** ten-minute shifts with Poisson truck arrivals at 2–3 receiving bays, one busier than the rest. A bay unloads only where there is room.
- **Capacity:** the same benchmark with trucks arriving 20× as often. That's more than the robots can clear, so the floor is always saturated.
  - Floors: 1.5 × 1 m and 3 × 2 m.
  - 24 shifts per point; 95% confidence intervals are taken over 2-shift chunks.
- **Rule:** a change is kept only if it improves its metric on these seeds without breaking a gate. Rejected changes are logged with their numbers (§6).
- **Statistics** (`sim/stats.py`): pass rates paired by floor and compared with McNemar's exact test; rates given with Wilson 95% intervals; throughput as a paired bootstrap ratio, with Wald's sequential test (SPRT) for a ≥ 5% gain.
- **Where it runs:** every benchmark runs on GitHub-hosted Linux runners (`.github/workflows/bench.yml`, both versions on the same floors), never on the laptop.

## 4. Results

**Table 1. Gates** (current code; `bash run_all.sh` runs all of them).

| Benchmark | Carry (gripper) | Push (no gripper) |
|---|---|---|
| 8 scenarios × 60 seeds | **480/480** | 462/480 |
| 500 random floors | **498/500** | 468/500 |
| 200 floors with parcels against the walls | **200/200** | 118/200 |
| 200 dense floors (6–10 robots, 10–14 parcels, 2 docks) | **193/200** (1 gridlock, 1 crate flagged, 5 with collisions) | 15/200 (42,992 collision events) |
| Robot–robot collisions, seeded runs | **0** | 2 |
| Energy per delivered parcel, seeded runs (model) | **19.9 mWh** | 28.9 mWh |

**Table 2. Floor manager on vs off**, 40 shifts each, the same trucks. "Off" means no staging and no hotspot learning.

| Floor | Manager | Mean wait for pickup | p90 wait | Arrival → dock | mWh/parcel | Collisions |
|---|---|---|---|---|---|---|
| 1.5 × 1 m, 2–5 robots | off | 14.8 s | 33.7 s | 21.7 s | 22.3 | 0 |
| | **on** | **7.5 s (−49%)** | **13.2 s (−61%)** | 15.3 s | 22.0 | 0 |
| 3 × 2 m, 2–5 robots | off | 10.0 s | 17.9 s | 19.7 s | 32.3 | 0 |
| | **on** | **8.7 s (−13%)** | **15.8 s (−12%)** | 18.4 s | 31.5 | 0 |

("Off" = `SIM_MANAGER=0`.) Waiting beside a bay has to happen *outside* the area where parcels are dropped. Measured on the earlier harness: at 250 mm from the bay's centre, the waiting robot blocked the unloading: wait 11.4 s, worse than no manager. At 400 mm the wait is 8.5 s (500 mm: 8.8 s; 600 mm: 9.1 s). Staging on its own changes nothing on this floor at this load, because docks rarely fill.

**Table 3. Capacity and scaling efficiency** (saturated floor). Scaling efficiency is throughput ÷ (robots × one robot's throughput). 100% means robots never get in each other's way. Good planners in Guo et al. scale near-linearly; Kuckling et al. (ICRA 2024, arXiv 2310.11843) show that many multi-robot systems instead peak and collapse.

| Robots | 3 × 2 m: parcels/h | Scaling efficiency | mWh/parcel | Collisions | 1.5 × 1 m: parcels/h | Scaling efficiency | mWh/parcel | Collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | 220 ± 13 | 100% | 18.3 | 0 | 255 ± 45 | 100% | 12.9 | 0 |
| 2 | 443 ± 18 | 101% | 18.8 | 0 | 569 ± 16 | 112% | 12.5 | 0 |
| 4 | 823 ± 24 | **93%** | 21.4 | 0 | 782 ± 29 | 77% | 18.2 | 2 |
| 6 | 1124 ± 23 | 85% | 24.1 | 0 | 799 ± 12 | 52% | 26.1 | 12 |
| 8 | 1344 ± 29 | 76% | 27.3 | 0 | 808 ± 22 | 40% | 30.6 | 4 |
| 10 | **1502 ± 27** | **68%** | 30.5 | **0** | 814 ± 14 | 32% | 31.7 | 7 |

Each point is 24 ten-minute shifts (4 hours of floor time); ± is a 95% interval over 2-shift chunks. One robot's rate is known to about ±6% (big floor) and ±18% (small floor), so efficiency figures carry that much noise; 112% at 2 robots on the small floor is inside it. Collisions on the small saturated floor are contact events in 4 hours, with 4 or more robots on 1.5 m².

## 5. Findings

1. **The robots aren't the bottleneck on a small floor; the floor is.** From 4 robots up, the 1.5 m² floor is limited by how fast bays can be unloaded and docks can clear (≈ 780–815 parcels/h). More robots add no throughput there. They add energy, 18.2 → 31.7 mWh per parcel from 4 to 10 robots, and contacts.
2. **Picking off the floor has a failure mode stations don't.** If parcels are dropped close around a robot, it can be boxed in for good. In the first version of the capacity benchmark, parcels landed as close as 130 mm to a robot, 160 mm apart from each other. That left 80 mm gaps for 120 mm robots, and robots inside a bay froze.
   - It produced an apparent "collapse": 421 parcels/h at 10 robots against 569 at 4.
   - One layout delivered 36 parcels in a shift; with a 300 mm "don't drop next to a robot" rule, it delivered 134.
   - On a real floor this rule is an interlock: the camera already knows where every robot is, so a light at each bay can tell the person unloading where not to drop.
   - **Lesson:** an induction-free system needs its drop rules modelled. Benchmarks that ignore them can show collapses that aren't the swarm's fault.
3. **Leaderless coordination scales on a big floor.** On 3 × 2 m, scaling efficiency is 101% at 2 robots, 93% at 4, 85% at 6 and 68% at 10, with zero collisions (1,502 parcels/h). Profiling 10 robots against 4 (earlier harness) shows where the rest goes:
   - **Traffic:** carrying takes longer, 8.0 → 11.3 s per parcel.
   - **Waiting for parcels:** robots idle while parcels wait to land, 0.8% → 11% of the time. Bays can't unload while robots stand in them, so this is the floor, not the robots.
   - **Job switching:** robots change job mid-drive more often, 1.37 → 2.63 drives per parcel.

   A stronger "keep your job" bonus didn't help (§6). Head-on conflicts already pass on a fixed side.

## 6. Methods tried and rejected (with numbers)

| Idea | Result | Verdict |
|---|---|---|
| **Docks that move toward where parcels arrive** (destinations on carts). Upper bound: place docks next to the bays. | 3 × 2 m floor, 4 robots: 824 → 987/h (+20%). 1.5 × 1 m floor, 4 robots: 579 → 194/h (docks crowd the bays). Moving the carts would cost more on top. Six shifts each, measured before the drop-rule fix in §5. | Rejected: small, fragile gain |
| **Cap on working robots** (fixed number; CONWIP-style) | Worse or no better at every cap tried (3–6 robots on duty, 10 robots, 1.5 m²). For example, cap 4 gave 210–296/h against 388–516/h uncapped on the same shifts. Measured before the drop-rule fix. | Rejected; replaced by ramp metering (stable duty order, nearest the work first) |
| **Ramp metering** (`work_density`; on duty = density × floor area, nearest the work first, always one free hand beyond the robots holding parcels). Named after freeway on-ramp lights (ALINEA, Papageorgiou, Hadj-Salem & Blosseville, *TRR* 1320, 1991). | 1.5 m², 10 robots, saturated: 742 ± 63 → 692–765/h (no throughput gain). Energy −18% to −32% per parcel. Dense floors 36 → 39/40, but one dense layout had 9 collision events. | Kept, **off by default**: saves energy, not throughput. Parked robots need wall parking before it's safe as a default. |
| Stronger job commitment (keep-your-job bonus 150 → 400 or 800 mm) | 3 × 2 m floor, 10 robots: 1479 ± 28 → 1508 ± 26 / 1476 ± 19 /h. Waits unchanged. | Rejected: inside the noise |
| **Optimal assignment** (Hungarian method, Kuhn 1955) of single-robot jobs, re-solved each frame | 1218 → 1208 of 1220 runs (McNemar p = 0.002); 13 collision events on dense floors; throughput unchanged (ratio 1.001). The global optimum reshuffles jobs whenever costs shift. | Rejected: greedy is stabler and just as productive |
| **Right of way by priority** (PIBT's rule) on continuous paths: higher priority plans through lower; lower steps aside | Dense floors finish 14% faster, but 53–135 collision events; with a motor-level bumper, slower and 4 gridlocks | Rejected: safe priority needs discrete coordination (PIBT on a graph) |
| **Pickup admission by deliverability**: start a pickup only if a flood fill from its dock's line-up points reaches the parcel at carrier clearance | Dense floors 182 vs 187/200 (McNemar p = 0.36); every other suite identical | Rejected |
| **Banker's order + prioritised lanes** (Dijkstra; Habermann 1969; Čáp et al. 2015, Thm 2): rank carriers so each has a route while the later ones stand still; lower ranks keep off higher-ranked lanes | On dense floors, parcels 15 cm apart seal the docks at planning clearance, so even a lone carrier often has no route and no safe order exists. The theorem's precondition rarely holds there | Not merged (branch `exp/lanes`) |
| **Buffered Voronoi cells** (Zhou et al. 2017, Thm 1) as a speed limit toward each neighbour | 20 mm margin froze robots packed at the start. 1 mm: dense collisions 7 → 1, but gridlocks 6 → 12; all suites 1358 vs 1364/1380; small-floor throughput 0.992× [0.979, 0.999]. Added on top of preemption: 1367 vs 1371/1380, throughput −0.8% | Rejected: trades collisions for standoffs, as the authors themselves warn |
| Staffing by arrival rate | Lost to always staffing on wait time. | Rejected |
| Overbook 3 or 4 at docks | 11–22 collision events on dense floors. | Rejected (kept 2) |
| Motion prediction between camera frames (commanded wheels or simulated encoders) | 128–138 vs 140/140 baseline. | Rejected: latency isn't the bottleneck |
| OpenCV ArUco3 fast detection | No speed-up at this resolution. | Rejected |

## 7. Getting stuck: every failure classified, then fixed (v3)

**Method.**
- The simulator now labels every failed run by what was left on the floor (`--classify`):
  - A: a crate pair that never assembled;
  - B: carriers in gridlock;
  - C: a crate flagged stuck;
  - D: the swarm's picture is wrong;
  - E: collisions;
  - F: a parcel never picked up.
- A liveness monitor counts **stalls**: no 30 mm of progress for 20 s, outside the designed waits.
- Each class was then traced to its root cause in the gateway log of a failing run.

| Class | Root cause found | Fix |
|---|---|---|
| A | The pair's grip face flipped every ~2 s: a room check at a threshold flickered with 3 mm of camera noise | The robots already on the crate count in the face's cost; the face they hold keeps a 15 mm allowance |
| B | **Hold-and-wait deadlock** (a Coffman condition): every robot held a parcel for dock slots whose lanes only a free robot could clear | Lane blockers are assigned first; no staging for a dock while a lane is blocked |
| C | A crate jammed at the dock was taken for "too heavy" and flagged | A load that has already moved is blocked, not heavy: re-approach (4 times; 8 used up the shift) |
| D | A parcel dragged out of its dock stayed "delivered" forever | The gateway reopens it 10 mm outside the dock (camera noise is ~3 mm) |

**Result**, v2 → v3 on the same 1,380 runs (re-measured on Linux, §7.1):
- 97.3% [96.3, 98.0] → **98.8% [98.1, 99.3]** (Wilson 95%);
- 25 runs fixed, 4 broken: McNemar exact p = 0.0001; dense floors 175 → 187/200;
- classes A, C, D: **0** on dense floors (v2: 5 A, 4 C). What remains there is traffic: 6 carrier gridlocks (not the dock deadlock) and 7 runs with collisions;
- throughput unchanged on the 3 × 2 m floor, +3% to +11% on the 1.5 × 1 m floor at 4–10 robots.

**Stalls that remain:** on dense floors most are carriers beside another robot (292 of 343 long stalls, before §7.1). Traffic again.

### 7.1 Re-measured on Linux, and deadlocks broken by preemption (v3.1)

**A harness bug, found while moving the benchmarks to GitHub's runners.**
- `std::uniform_real_distribution` and `std::normal_distribution` are implementation-defined. From the same seed, macOS (libc++) and Linux (libstdc++) drew *different floors and different sensor noise*. The "platform differences" reported earlier were partly different test floors, not only last-bit maths.
- The dense-floor generator could also loop forever when random packing jammed before all 14 parcels fit (a gdb backtrace on a runner showed it).
- Fixed: portable `u01()`/`gauss()` built on mt19937, one seed per floor, a jammed placement starts again.
- **Every number in this report is now from GitHub's Linux runners**, image pinned to ubuntu-24.04, and the dense suite has 200 floors instead of 40.

**Gridlock mechanism (class B, 6 of 200 dense floors).** On dense floor 158, all 9 robots hold a parcel by t = 45 s and none can move. The generator packs 14 parcels 15 cm apart across the middle of the floor, and a carrier is about 21 cm wide. Admission control never binds: each dock has about 6 slots plus 2 overbooked.

**Fix: deadlock recovery by preemption.**
- A deadlock needs all four of Coffman's conditions (Coffman, Elphick & Shoshani 1971): mutual exclusion, hold-and-wait, circular wait, no preemption. On a crowded floor the first three can't be ruled out, so the fourth is broken.
- When **every** carrier has gone 20 s without progress, the one farthest from its dock sets its parcel down and backs off. Nobody picks that parcel up for 30 s.
- Progress means the parcel is 30 mm nearer its dock than ever on this job. Measuring displacement instead hid a five-carrier deadlock behind one robot rocking 40 mm back and forth.
- Every robot sees the same snapshots, so all agree on the victim without a message.

**Result** (same 1,380 runs, paired):
- 1364 → **1371/1380** (99.35%, Wilson 95% CI [98.8, 99.7]); McNemar p = 0.039.
- Dense floors 187 → 193/200; gridlocks 6 → 1.
- The 80 shifts are bit-identical: on those floors it never triggers.

**The 9 failures that remain:**
- 6 with collisions, every parcel delivered. Every contact involves a robot carrying a parcel.
- 2 crates flagged for a person.
- 1 gridlock (dense floor 199).

## 8. A queueing model of the floor

`sim/mva.py` treats the saturated floor as a closed queueing network, solved with exact Mean Value Analysis (Reiser & Lavenberg 1980):
- N robots cycle through four stations: driving (a delay station), bay pickup (2 robots at once per bay), dock set-down (one server per usable slot), and back.
- Every input is measured separately: one-robot timings, plus slot counts from the dock geometry. Nothing is fitted to the curve.

| Robots | 1 | 2 | 4 | 6 | 8 | 10 |
|---|---|---|---|---|---|---|
| 3 × 2 m, model | 210 | 412 | 790 | 1130 | 1427 | 1675 |
| 3 × 2 m, simulation | 220 | 443 | 823 | 1124 | 1344 | 1502 |
| error | −5% | −7% | −4% | +1% | +6% | +12% |

(The model's one-robot timings were measured on the earlier harness; the simulation row is the current Linux run.)

**What the model says:**
- **Within ±7% up to 8 robots** on independent inputs.
- The 12% gap at 10 robots is the one thing it leaves out: traffic. Carrying gets 41% slower at 10 robots (8.0 → 11.3 s).
- That gap is also the **ceiling** for any traffic planner on this floor: about +12%.

**The small floor's limit is not the robots.** The same ~850 parcels/h land whether 4 or 10 robots work: a bay accepts a parcel only when no robot is within 30 cm. At 10 robots they stand idle 48% of the time. The model reproduces it once that measured inbound rate is given as a bound.

## 9. Real-world protocol (ready, not yet run)

1. `satellite.py --log run.csv` records every camera frame during a run.
2. `kpi.py run.csv` computes, **from camera geometry alone**, the same KPIs as the simulator: delivered %, collisions, wall scraping, parcels per hour, wait for pickup and arrival-to-dock time. What the robots report about themselves is ignored.
   - The scorer is tested against hand-made logs and against simulator logs; the simulator's ground truth and `kpi.py` agree.
3. For each of the 8 scenarios: 10 timed real trials, in push mode now and in carry mode once a gripper is fitted. The output is a sim-vs-real table.

### 9.1 First real-robot run: Georgia Tech's Robotarium (2026-10-08)

Our own push-only prototypes haven't run the swarm yet, so the coordination layer was ported to Python (`robotarium/swarm_sort.py`) and run on the Robotarium [20]: 8 real 11 cm robots, its overhead tracking as the shared snapshot, and our floor at 11/12 scale so every distance is the same in robot-lengths. Parcels, docks and bays are projected onto the arena. All 8 planners run in one script on the Robotarium's server, each given the robots in a different order, and their plans are compared at every snapshot. Collision avoidance is the Robotarium's barrier certificate [21], re-implemented so a stopped robot is a fixed obstacle.

**Prediction first.** Before the run, the same script ran 8 times in the Robotarium's own simulator on GitHub's runners, each with that simulator's randomness (where the robots start) seeded differently. Small differences grow, so one run is one sample. Those batches found five bugs in the port, each from the logs or the preview video:

- a carrier whose dock slot changed mid-insert drove across the floor (`core/brain.h` locks the slot; the port didn't);
- a parcel lying at a dead robot was unreachable inside the barrier's 19 cm bubble (now the dying robot backs off 10 cm);
- the barrier froze two robots face to face for 270 s in 3 of 8 runs, a known deadlock of the method (now a keep-right detour after 5 s without progress);
- the crate starved in 2 of 8 runs (the port lacked `allocate()`'s second pass, which recruits a partner early);
- the final approach was a blind straight drive (now heading-held, retried after 6 s).

**Result.**

| Measure | Predicted (8 simulator runs) | Real robots |
|---|---|---|
| Parcels delivered in 300 s | 44.0, SD 3.9 (95% prediction interval 34–54) | **46** |
| Plan disagreements | 0 in 24,248 snapshots | **0** in 3,029 |
| Crate delivered by a pair | 8 of 8 | yes |
| Robot "dies" at 150 s, the rest carry on | 8 of 8 | yes (22 delivered before, 24 after) |

46 is 0.5 SD above the simulated mean: consistent with the prediction. One run can't establish more than that. A second batch of 8 simulator runs with the same planner gave 40.3; over all 16: 42.1, SD 3.9. Against that, the real run is +1.0 SD (95% prediction interval 34–51). The crate was delivered in 14 of the 16 simulated runs: the port's known weak point. The repeat runs keep the planner fixed, so that they repeat run 1. Real steps averaged 33.5 ms against the nominal 33 ms (300 s took about 304.5 s). Video and the Robotarium's full recording: [release robotarium-2026-10-08](https://github.com/kavin-jain/swarm-intelligence/releases/tag/robotarium-2026-10-08).

## 10. Limits

- Our own robots: simulation only, until §9 is run. The coordination layer has run once on the Robotarium's robots (§9.1), with projected parcels and their collision avoidance.
- **Dense traffic.** On crowded floors robots can still touch (5 of 200 dense floors, every parcel delivered) or, rarely, gridlock (1 of 200). Every remaining contact involves a robot carrying a parcel. On a saturated 1.5 m² floor with 6–10 robots, there are 4–12 contact events in 4 hours. Collision-free motion with a proof (buffered Voronoi cells) cost standoffs on these floors (§6). Discrete coordination (PIBT on a graph) needs cells bigger than this floor allows.
- **Two-robot crates.** 2 of the 9 remaining failures in 1,380 runs are crates flagged for a person.
- **Robot-free drop zones.** The results assume nobody drops a parcel within 30 cm of a robot (§5).
- One camera bounds the floor size. On-robot localisation is the fix; the decisions already run on the robots.
- At most 10 robots, 16 parcels and 3 docks per snapshot (the 250-byte ESP-NOW packet).
- Robot speed is 0.26 m/s (hobby TT motors). Absolute throughput is therefore far below commercial sorters, which run at about 2 m/s. The comparable quantities are the dimensionless ones: delivery rate, collisions, scaling efficiency and wait reduction.
- The energy figures come from a model.

## 11. What would move the numbers most

Where a robot's time goes with 4 robots on the 3 × 2 m floor, saturated:

| Activity | Share of time |
|---|---|
| Carrying | 46% |
| Driving empty | 29% |
| Handling (line up, dock, grip, back off, place) | 24% |

The next large gain is therefore mechanical, not algorithmic:
- a self-centring funnel gripper that captures a parcel on the move would cut most of the handling time;
- faster motors would cut the driving time.

## References

1. P. R. Wurman, R. D'Andrea, M. Mountz. *Coordinating Hundreds of Cooperative, Autonomous Vehicles in Warehouses.* AI Magazine 29(1), 2008.
2. T. Guo, S. W. Feng, J. Yu. *Bin Assignment and Decentralized Path Planning for Multi-Robot Parcel Sorting.* arXiv 2310.17753.
3. J. Kuckling, R. Luckey, V. Avrutin, A. Vardy, A. Reina, H. Hamann. *Do We Run Large-scale Multi-Robot Systems on the Edge? More Evidence for Two-Phase Performance in System Size Scaling.* ICRA 2024, arXiv 2310.11843.
4. H. Zheng, Y. Ma, B. Araki, J. Chen, C. Wu. Deep-RL robot prioritisation for warehouse traffic. *JAIR*, March 2026 (MIT IDSS news, 2026-03-30).
5. P. Tokekar, M. Benosman, R. Chandan, A. O. G. Barbosa, M. Caldara, J. W. Durham. *Model-Free Adaptive Parameter Tuning for Efficient Multi-Robot Warehouse Operations.* arXiv 2608.21533, 2026.
6. *Dynamic Robot Routing and Destination Assignment Policies for Robotic Sorting Systems.* Transportation Science, doi 10.1287/trsc.2023.0458 (abstract only; paywalled).
7. Y. T. dos Passos, X. Duquesne, L. S. Marcolino. *Congestion control algorithms for robotic swarms with a common target based on the throughput of the target area.* arXiv 2201.09337, 2022.
8. M. Papageorgiou, H. Hadj-Salem, J.-M. Blosseville. *ALINEA: A local feedback control law for on-ramp metering.* Transportation Research Record 1320, 1991.
10. K. Okumura, M. Machida, X. Défago, Y. Tamura. *Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.* Artificial Intelligence, 2022 (arXiv 1901.11282).
11. K. Okumura, Y. Tamura, X. Défago. *Time-Independent Planning for Multiple Moving Agents.* AAAI 2021.
12. B. Zou, R. de Koster, Y. Gong, X. Xu, G. Shen. *Robotic Sorting Systems: Performance Estimation and Operating Policies Analysis.* Transportation Science 55(6), 2021.
13. M. Reiser, S. S. Lavenberg. *Mean-Value Analysis of Closed Multichain Queuing Networks.* J. ACM 27(2), 1980.
14. H. W. Kuhn. *The Hungarian Method for the Assignment Problem.* Naval Research Logistics Quarterly 2, 1955.
15. A. Wald. *Sequential Tests of Statistical Hypotheses.* Annals of Mathematical Statistics 16(2), 1945.
9. *Meet the drone that already delivers your packages: Kiva robot teardown.* Robohub.
16. E. G. Coffman, M. J. Elphick, A. Shoshani. *System Deadlocks.* ACM Computing Surveys 3(2), 1971.
17. D. Zhou, Z. Wang, S. Bandyopadhyay, M. Schwager. *Fast, On-line Collision Avoidance for Dynamic Vehicles Using Buffered Voronoi Cells.* IEEE Robotics and Automation Letters 2(2), 2017.
18. M. Čáp, P. Novák, A. Kleiner, M. Selecký. *Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots.* IEEE Transactions on Automation Science and Engineering 12(3), 2015.
19. A. N. Habermann. *Prevention of System Deadlocks.* Communications of the ACM 12(7), 1969.
20. D. Pickem, P. Glotfelter, L. Wang, M. Mote, A. Ames, E. Feron, M. Egerstedt. *The Robotarium: A Remotely Accessible Swarm Robotics Research Testbed.* IEEE ICRA, 2017.
21. L. Wang, A. D. Ames, M. Egerstedt. *Safety Barrier Certificates for Collisions-Free Multirobot Systems.* IEEE Transactions on Robotics 33(3), 2017.
