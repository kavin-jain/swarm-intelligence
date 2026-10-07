# Leaderless, infrastructure-free parcel sorting with low-cost robots

**Kavin Jain** · technical report, October 2026 · code, simulator and every benchmark in this repository

> **Status of the evidence.** Every number below comes from the simulator in `sim/`. It runs the *same* robot code as the firmware, with camera noise, radio loss, motor mismatch and grip failures modelled. Nothing here has yet been measured on the physical robots. The protocol for that run is in §9, and the tool that scores it (`satellite/kpi.py`) is already written and tested.

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

- **99.75% of 1,220 benchmark runs** fully delivered on macOS: 479/480 seeded, 499/500 random, 200/200 against walls, 40/40 dense. Up from 98.85% after a pattern analysis of every failure (§7), significant on paired seeds (McNemar p = 0.013). On Linux the same code scores 1,214/1,220. Crowded floors are chaotic enough that last-bit differences in the maths library change the outcome (dense: 35/40, with 2 gridlocks and 3 runs with collisions);
- zero robot–robot collisions in the seeded, random and dense sets;
- the manager rules cut the mean wait before pickup by **54%** (p90 by 63%) at equal energy.

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

## 4. Results

**Table 1. Gates** (current code; `bash run_all.sh` runs all of them).

| Benchmark | Carry (gripper) | Push (no gripper) |
|---|---|---|
| 8 scenarios × 60 seeds | **479/480** (99.8%) | 473/480 |
| 500 random floors | **499/500** | 481/500 |
| 200 floors with parcels against the walls | **200/200** | 129/200 |
| 40 dense floors (6–10 robots, 10–14 parcels, 2 docks) | **40/40** on macOS, 35/40 on Linux (2 gridlocks, 3 with collisions) | 3/40 (2,014 collision events) |
| Robot–robot collisions, seeded runs | **0** | 33 |
| Energy per delivered parcel, seeded runs (model) | **19.7 mWh** | 28.3 mWh |

**Table 2. Floor manager on vs off**, 40 shifts each, the same trucks. "Off" means no staging and no hotspot learning.

| Floor | Manager | Mean wait for pickup | p90 wait | Arrival → dock | mWh/parcel | Collisions |
|---|---|---|---|---|---|---|
| 1.5 × 1 m, 2–5 robots | off | 15.7 s | 34.5 s | 22.5 s | 22.4 | 0 |
| | **on** | **7.3 s (−54%)** | **12.8 s (−63%)** | 15.0 s | 22.0 | 0 |
| 3 × 2 m, 2–5 robots | off | 9.8 s | 17.1 s | 19.4 s | 32.2 | 0 |
| | **on** | **8.5 s (−13%)** | **15.2 s** | 18.2 s | 31.5 | 0 |

Waiting beside a bay has to happen *outside* the area where parcels are dropped. At 250 mm from the bay's centre, the waiting robot blocked the unloading: wait 11.4 s, worse than no manager. At 400 mm the wait is 8.5 s (500 mm: 8.8 s; 600 mm: 9.1 s). Staging on its own changes nothing on this floor at this load, because docks rarely fill.

**Table 3. Capacity and scaling efficiency** (saturated floor). Scaling efficiency is throughput ÷ (robots × one robot's throughput). 100% means robots never get in each other's way. Good planners in Guo et al. scale near-linearly; Kuckling et al. (ICRA 2024, arXiv 2310.11843) show that many multi-robot systems instead peak and collapse.

| Robots | 3 × 2 m: parcels/h | Scaling efficiency | mWh/parcel | Collisions | 1.5 × 1 m: parcels/h | Scaling efficiency | mWh/parcel | Collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | 206 ± 22 | 100% | 19.1 | 0 | 292 ± 22 | 100% | 11.9 | 0 |
| 2 | 438 ± 22 | 106% | 18.9 | 0 | 517 ± 52 | 89% | 13.2 | 0 |
| 4 | 821 ± 26 | **100%** | 21.4 | 0 | 775 ± 22 | 66% | 18.4 | 0 |
| 6 | 1114 ± 28 | 90% | 24.3 | 0 | 768 ± 50 | 44% | 26.5 | 0 |
| 8 | 1355 ± 22 | 82% | 27.1 | 0 | 711 ± 78 | 30% | 32.5 | 8 |
| 10 | **1496 ± 21** | **73%** | 30.5 | **0** | 742 ± 63 | 25% | 33.0 | 0 |

Each point is 24 ten-minute shifts (4 hours of floor time). One robot's rate is known to about ±10%, so efficiency figures carry about ±10 points; 106% at 2 robots is inside that noise.

## 5. Findings

1. **The robots aren't the bottleneck on a small floor; the floor is.** From 4 robots up, the 1.5 m² floor is limited by how fast bays can be unloaded and docks can clear (≈ 750–800 parcels/h). More robots add no throughput there. They add energy: 18.4 → 33.0 mWh per parcel from 4 to 10 robots.
2. **Picking off the floor has a failure mode stations don't.** If parcels are dropped close around a robot, it can be boxed in for good. In the first version of the capacity benchmark, parcels landed as close as 130 mm to a robot, 160 mm apart from each other. That left 80 mm gaps for 120 mm robots, and robots inside a bay froze.
   - It produced an apparent "collapse": 421 parcels/h at 10 robots against 569 at 4.
   - One layout delivered 36 parcels in a shift; with a 300 mm "don't drop next to a robot" rule, it delivered 134.
   - On a real floor this rule is an interlock: the camera already knows where every robot is, so a light at each bay can tell the person unloading where not to drop.
   - **Lesson:** an induction-free system needs its drop rules modelled. Benchmarks that ignore them can show collapses that aren't the swarm's fault.
3. **Leaderless coordination scales on a big floor.** On 3 × 2 m, scaling efficiency is 100% up to 4 robots, 90% at 6 and 73% at 10, with zero collisions (1,496 parcels/h). Profiling 10 robots against 4 shows where the remaining 27% goes:
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

**Result on the same 1,220 runs:**
- 98.85% [98.1, 99.3] → **99.75% [99.3, 99.9]** (Wilson 95%);
- 14 runs fixed, 3 broken: McNemar exact p = 0.013;
- classes A, B, D, E: **0** on macOS. On Linux, dense floors still show 2 carrier gridlocks (traffic, not the dock deadlock) and 3 runs with collisions. Dense traffic is the open problem;
- throughput unchanged (paired ratio 0.999).

**Stalls that remain** (~300 episodes in all suites):
- On dense floors, 55 of 68 are beside another robot: traffic.
- On 2–4 robot floors, they're mostly at the dock (39 of 99) and waiting for a partner (29).

## 8. A queueing model of the floor

`sim/mva.py` treats the saturated floor as a closed queueing network, solved with exact Mean Value Analysis (Reiser & Lavenberg 1980):
- N robots cycle through four stations: driving (a delay station), bay pickup (2 robots at once per bay), dock set-down (one server per usable slot), and back.
- Every input is measured separately: one-robot timings, plus slot counts from the dock geometry. Nothing is fitted to the curve.

| Robots | 1 | 2 | 4 | 6 | 8 | 10 |
|---|---|---|---|---|---|---|
| 3 × 2 m, model | 210 | 412 | 790 | 1130 | 1427 | 1675 |
| 3 × 2 m, simulation | 206 | 438 | 821 | 1114 | 1355 | 1496 |
| error | +2% | −6% | −4% | +1% | +5% | +12% |

**What the model says:**
- **Within ±6% up to 8 robots** on independent inputs.
- The 12% gap at 10 robots is the one thing it leaves out: traffic. Carrying gets 41% slower at 10 robots (8.0 → 11.3 s).
- That gap is also the **ceiling** for any traffic planner on this floor: about +12%.

**The small floor's limit is not the robots.** The same ~850 parcels/h land whether 4 or 10 robots work: a bay accepts a parcel only when no robot is within 30 cm. At 10 robots they stand idle 48% of the time. The model reproduces it once that measured inbound rate is given as a bound.

## 9. Real-world protocol (ready, not yet run)

1. `satellite.py --log run.csv` records every camera frame during a run.
2. `kpi.py run.csv` computes, **from camera geometry alone**, the same KPIs as the simulator: delivered %, collisions, wall scraping, parcels per hour, wait for pickup and arrival-to-dock time. What the robots report about themselves is ignored.
   - The scorer is tested against hand-made logs and against simulator logs; the simulator's ground truth and `kpi.py` agree.
3. For each of the 8 scenarios: 10 timed real trials, in push mode now and in carry mode once a gripper is fitted. The output is a sim-vs-real table.

## 10. Limits

- Simulation only, until §9 is run.
- **Dense traffic.** Robots on crowded floors can still gridlock or touch: 35/40 dense floors on Linux, 40/40 on macOS, so 75/80 combined (93.8%, 95% CI [86.2%, 97.3%]). PIBT on a graph is the planned fix (§11).
- **Two-robot crates.** 2 of the 3 remaining failures in 1,220 runs are crates flagged stuck. In heavy_box seed 12 the pair falls out of step: one grips while the other is still lining up.
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
