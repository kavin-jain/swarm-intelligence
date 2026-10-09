# Robotarium experiment: leaderless parcel sorting on real robots

The swarm's coordination layer, run on the [Robotarium](https://www.robotarium.gatech.edu/), Georgia Tech's remotely accessible robot swarm (free for education and research). Our own robots are push-only ESP32 prototypes that haven't run the swarm yet, and there's no budget for more; this is the cheapest honest way to run the coordination on real robots: real motors, real tracking, real lag.

## What is real, what is virtual

| Part | In this experiment | In the full system |
|---|---|---|
| Robots | **Real**: 8 Robotarium robots (11 cm, 0.2 m/s max) | ESP32 robots (12 cm, 0.26 m/s) |
| Seeing the floor | **Real**: the Robotarium's tracking (`get_poses()`), shared as one snapshot ~10 times a second | One overhead phone camera, shared by radio (ESP-NOW) ~10 times a second |
| Deciding who does what | **Ours**, ported from `core/brain.h`: one brain per robot, each planning from the same snapshot (all 8 run in the one script on the Robotarium's server) | The same planner, compiled into each robot's firmware |
| Avoiding collisions | **Theirs**: the Robotarium's barrier certificates, which the Robotarium requires (plus a keep-right detour when they freeze two robots face to face) | Our path planner plus steering |
| Parcels, docks, bays | **Virtual**: drawn by the script and projected onto the arena floor; a gripped parcel is drawn riding in front of its robot | Real boxes, real docks, a gripper |

What this tests: the leaderless allocation on real robots (does every robot still agree, and does the work still get done when real motion is slower and noisier than planned), pairs recruited for a heavy crate, staging beside a full dock, a robot dying mid-shift, and deadlock recovery.
What it can't test: the camera pipeline, the radio, gripping, and pushing or carrying real objects.

## The floor (projected): our floor at 11/12 scale

Their robots are 11 cm and ours are 12 cm, so the projected floor is our simulator's 3 × 2 m shift-benchmark floor at **11/12 scale** (2.75 × 1.83 m inside their 3.2 × 2 m arena). Every distance stays the same number of robot-lengths: docks, bays, parcels, grip reach, line-up distance, the start column. A bigger robot of ours would use the same layout at a bigger scale; the ratios are what carry over.

| Thing | Our floor (sim) | Robotarium (× 11/12) |
|---|---|---|
| Floor | 3.0 × 2.0 m | 2.75 × 1.83 m (projected) |
| Robot | 12 cm | 11 cm |
| Dock A, dock B | (2640, 1460), (2640, 540) mm, r 150 mm | (1.045, +0.422), (1.045, −0.422) m, r 0.138 m |
| Set-down slots per dock | 6 round the ring | 6 round the ring (a slot whose approach a dead robot blocks is skipped) |
| Bay 1 (60%), bay 2 (40%) | (1500, 1300), (1400, 600) mm | (0.000, +0.275), (−0.092, −0.367) m |
| Parcel / crate | 9 cm / 12 cm discs | 8.3 cm / 11 cm discs |
| Grip reach, line-up distance | 105 mm, 80 mm | 96 mm, 73 mm |
| Start / home poses | charging wall, 5 a column, y 240…1760 mm | x −1.35 and −1.00 m, y −0.70…+0.70 m (35 cm apart), facing +x |

**Not to scale** (each forced by a Robotarium rule or limit):
- **Speed:** ours drive 0.26 m/s (2.2 body-lengths/s), and the Robotarium caps at 0.2 m/s. Cruise is 0.15 m/s, so everything takes about 1.6× longer here.
- **Minimum spacing:** the Robotarium keeps robots at least 13.5 cm apart (its barrier certificate: about 19 cm). Ours may pass closer. So start poses are 35 cm apart (their start-up routine needs 25 cm or it never finishes), a carrying pair stands 24 cm apart instead of 12 cm, and crowding costs more here than on our floor.
- **Collision avoidance** is theirs, not ours (required). Their method can freeze two robots face to face for good (seen in 3 of 8 simulator runs), so a robot that makes no 10 cm of progress in 5 s while driving veers right for 2 s.
- **The dying robot backs off 10 cm** after setting its parcel down. Their bubble keeps robots about 19 cm apart, so nobody could get close enough to grip a parcel lying right at a dead robot.

Colours: blue parcels go to dock A, orange to dock B; the crate has a black outline. Robot rings: grey idle, yellow heading to a parcel, green carrying, red dead. The counter at top left shows time, parcels delivered, parcels on the floor, and the plan check ("disagreements in … snapshots").

## Timeline (300 s after the robots reach their start poses)

| Time | Event |
|---|---|
| 0 s | Shift starts. A parcel lands about every 6 s (Poisson), at bay 1 or bay 2 (60/40) |
| 45 s | A heavy crate lands at bay 1. Two robots line up across its face, grip together and carry it to dock B |
| from 150 s | The first robot seen carrying a parcel has its battery "die" (robot 3 if nobody is carrying by 210 s): it sets the parcel down, backs off 10 cm and stops for good. The others route round it and deliver that parcel |
| any time | If every carrier goes 20 s without getting 3 cm nearer its dock, the one farthest from its dock sets its parcel down (deadlock recovery by preemption; that parcel waits 30 s before anyone picks it up again) |
| 300 s | End. The results are projected for 10 s with every robot stopped (the Robotarium returns only the video, so the camera records them), printed, and saved to `swarm_results.npz` |

## What success looks like (checked in the results)

1. **Zero collisions and nobody outside the arena**: the Robotarium's own checker (`r.debug()`).
2. **0 plan disagreements**: every snapshot, all 8 brains plan from the robots listed in a different order, and must agree.
3. **Throughput and waits**: parcels delivered, mean wait from landing to pickup, mean landing-to-dock time. These are compared with the Robotarium's own simulator and with our simulator.
4. **The crate delivered by a pair, and the dead robot's parcel delivered by someone else** (the script's last line: when, and by which robots).

## The prediction (Robotarium simulator, 8 seeded runs, commit 150fec6)

What the real run gets compared with. Every run passed the simulator's checker with no errors or warnings.

| Measure | 8 runs | Range |
|---|---|---|
| Parcels delivered in 300 s | **44.0** (95% CI 40.8–47.2), 528/h | 37–49 |
| Mean wait from landing to pickup | 25.2 s | 19.6–33.6 s |
| Crate delivered by a pair | 8 of 8 | at 83–210 s |
| Dead robot's parcel delivered by another robot | 8 of 8 | 15–101 s after the failure |
| Plan disagreements | **0** in 24,248 snapshots | |
| Our compute per step (the Robotarium steps every 33 ms) | mean 2.6–6.5 ms | worst single step 33.8 ms |

A second batch of 8 with the same planner (commit f47ed02 adds only the results screen and counters) delivered 40.3 (range 36–46). The crate was delivered in 6 of 8, and the closest any two robots came was 20.8 cm (limit 13.5), with 0 too-close steps. **Over all 16 runs: 42.1 delivered, SD 3.9; the crate in 14 of 16.** The two batches differ by more than their seeds explain: the simulator isn't bit-for-bit repeatable on the runners, so each run is a fresh sample.

## The real run (2026-10-08)

One run on 8 Robotarium robots, with the script the prediction above used. [Video: 70 s at 5×, captioned](https://github.com/kavin-jain/swarm-intelligence/releases/download/robotarium-2026-10-08/robotarium-real-run-edited.mp4) · [the Robotarium's full recording](https://github.com/kavin-jain/swarm-intelligence/releases/download/robotarium-2026-10-08/robotarium-real-run-original.mp4).

[![The real run at the Robotarium](real_run.jpg)](https://github.com/kavin-jain/swarm-intelligence/releases/download/robotarium-2026-10-08/robotarium-real-run-edited.mp4)

| Measure | Predicted (8 simulator runs) | Real robots |
|---|---|---|
| Parcels delivered in 300 s | 44.0 (range 37–49; 95% prediction interval for one run 34–54) | **46** |
| Plan disagreements | 0 in 24,248 snapshots | **0** in 3,029 snapshots |
| Crate delivered by a pair | 8 of 8 | yes: carried to dock B at about 100–115 s |
| A robot's battery "dies" | at 150 s, 8 of 8 | robot 1 at 150 s; the other 7 kept sorting (22 delivered before it, 24 after) |
| Time per step | 33 ms | 300 s of experiment took about 304.5 s: 33.5 ms per step on average |

Read from the projected counters in the recording. 46 is 0.5 standard deviations above the 8-run prediction made before the run, and +1.0 SD against all 16 simulator runs (95% prediction interval 34–51). One run is consistent with the prediction, not proof of it.

**Repeats:** runs 2–5 use `swarm_sort.py` from commit f47ed02. Its behaviour is unchanged before 300 s, so they repeat run 1; it also projects every result for the last 10 s, because the Robotarium returns only the video. Not read yet: the collisions the Robotarium logged, wait times, and when the dead robot's parcel was delivered. Those are in the run's output log and `swarm_results.npz` on the Robotarium experiment page.

## Running it

- **In the Robotarium simulator (on GitHub runners, not the laptop):** every push to `robotarium/` runs `.github/workflows/robotarium.yml`: 8 runs, each with the simulator's own randomness (where the robots start) seeded differently, since small differences grow and one run is one sample. Each gives the simulator's verdict, the metrics and compute per step (artifacts `robotarium-1` … `robotarium-8`); run 1 also gives a 3× preview video.
- **On the Robotarium:** fill in New Experiment with the text below and upload `swarm_sort.py`.

## Submission form

- **Title:** Leaderless parcel sorting: lockstep task allocation with dock admission control
- **Estimated Duration (seconds):** 400 (300 s of sorting plus driving to the start poses)
- **Number of Robots:** 8
- **Files:** `swarm_sort.py`
- **Experiment Description:**

> Eight robots sort virtual parcels (projected discs) from two unloading bays to two docks, with no central planner. Every robot runs its own copy of the same deterministic planner on the same shared snapshot (poses plus parcel states, about 10 Hz), so they agree on who takes which parcel without negotiating; the script checks this every snapshot by giving each brain the robots in a different order. The planner includes dock admission control (start a pickup only if a dock slot is free, or one of 2 staging places beside it), a pair recruited for a heavy crate, a robot that "fails" at about 150 s (it backs off and stops; its parcel is reallocated), and deadlock recovery by preemption. Collision avoidance is the same QP as rps create_uni_barrier_certificate_with_boundary (safety radius 0.13 m, same gains), reimplemented so the stopped robot is a fixed obstacle instead of a robot expected to move, plus a keep-right detour when the barrier freezes two robots. It passed the Robotarium simulator with no errors or warnings in 8 of 8 runs. It is a port of the coordination layer of an ESP32 swarm (github.com/kavin-jain/swarm-intelligence) whose own prototype robots have not run it yet. Output: printed metrics and swarm_results.npz.

## Second experiment: every robot is a witness (`swarm_witness.py`)

**The problem.** A robot counts as alive if the camera sees it and its heartbeat arrives (`robot_alive()` in `core/world.h`). A robot with a weak wheel, or one hijacked to ignore its plan, passes both checks, keeps its parcel and keeps being given work.

**The idea.** Every robot already runs the same planner on the same snapshot. So every robot can work out what every robot's wheels were told to do, and from that, where each robot should be at the next snapshot. A robot that keeps ending up somewhere else gets flagged by every robot, at the same snapshot, with no messages and no vote.

**The test.**
- **Residual:** where a robot is, minus where its own commands should have put it, per snapshot. It has two parts: along its track, and in its heading.
- **Scaling:** each residual is divided by the fleet's pooled noise (1.4826 × the median absolute deviation over the last 30 s) and clipped at ±4. The clip means one tracking glitch can't flag a robot.
- **Detector:** a two-sided CUSUM change detector on each part (Page 1954), with reference value K = 0.5.
- **Threshold:** H = 12.11. That gives under 1 false alarm per robot per 8 h of driving, by Siegmund's approximation of the average run length: ARL = (e^(−2Δb) + 2Δb − 1)/(2Δ²), with b = H + 1.166 (Siegmund 1985). The same formula predicts, before the run, how long each fault takes to catch.
- **Only moving robots count:** a parked robot gives no evidence about its wheels.
- **The fleet learns its own wheel lag.** Wheels follow commands with a delay, so a robot starting or stopping a turn would look wrong for a moment. The witness tries 7 lags (0 to 0.3 s), modelling the wheels as a first-order lag, and keeps the one that best explains every robot's motion over the last 30 s. Every robot computes the same choice from the same data. In the simulator it recovers the 0.1 s lag we put in. Without it, 1 run in 16 flagged a healthy robot after a sharp turn.

**What it improves on:**
- Millard, Timmis & Winfield (2013): each robot simulates the others on board, guessing what they sensed.
- Tarapore et al. (2017): robots vote.
- Carminati et al. (2024): a model learns from labelled faults, tested in simulation only.

Here the inputs are exact (the shared snapshot), so the only gap between prediction and motion is the robots' own imperfection. Mode `exact` checks this in the noise-free simulator, where the prediction must match to 0.1 mm.

**Faults** (injected by the script; the brains aren't told, and heartbeats look normal):

| Time | Fault | Caught by heartbeats? |
|---|---|---|
| 90 s | Weak wheel: the first robot seen driving loses half its left wheel's power (these robots' wheels are fine, so it is sent what a weak one would do) | No |
| 150 s | Battery death, as in the first experiment | Yes |
| 210 s | Hijack (a theft): the first robot carrying a parcel drives it to the far wall instead of its dock. Its heartbeat keeps saying "carrying". It has its own barrier, solved against everyone else's actual velocities, so it can't hit anyone | No: it looks like a delivery in progress |

**Modes:**

| Mode | What happens |
|---|---|
| `exact` | No faults: the witness must predict every robot to within 0.1 mm (simulator only) |
| `off` | Faults injected; the witness only watches and reports. This is the control |
| `on` | Faults injected; a flagged robot gets no more work, and its position counts as an obstacle (dock slots it blocks are skipped). A flagged robot that still obeys sets its parcel down and drives home for repair; a parcel held by one that doesn't is reported stolen |

**On the projector:**
- **Ghost:** a dashed circle per robot, where its own commands of the last 1.5 s put it. A healthy robot sits on its ghost; a faulty one leaves it.
- **Purple ring:** a robot the witness flagged.
- **Two results pages:** first the results, then the calibration. The calibration page shows the noise, how strongly each residual follows the one before it (lag-1 correlation; the theory assumes independent residuals), the tails, each robot's mean residual, and the highest statistic each robot reached.

**Real runs:**
- **W1, mode `off`:** measures the real robots' noise, the harm the faults do, and when the witness would have flagged them. H for W2 is then fixed from W1, and the W2 prediction is written here before W2 is submitted.
- **W2, mode `on`:** the threshold fixed from W1.

**Safety with faulty robots.** A faulty robot doesn't do its half of a dodge. So each faulty robot gets its own barrier, solved against everyone else's actual velocities, and takes the whole of every dodge. Nobody else's commands change, and the Robotarium's 13.5 cm rule holds. (The first version split each dodge as usual and broke the rule in 8 of 8 runs.)

**In the simulator**, `.github/workflows/witness.yml` runs `on` and `off` × 8 seeds plus `exact`. Since the simulator is perfect, it adds a wheel lag (0.1 s), tracking noise (2 mm; heading 1.1°) and a spread of wheel gains (SD 3%), in the simulator only. These are stand-ins until W1 measures the real values. `python3 robotarium/witness_report.py DIR` reports on the downloaded artifacts.

### The prediction for W1, written before the run (simulator, commit 402787c)

8 runs in mode `off`, with the simulator-only stand-ins: wheel lag 0.1 s, tracking noise 2 mm, wheel-gain spread 3%. All 17 runs of the batch (`exact`, `off`, `on`) passed the simulator's checker.

| Measure | Prediction (8 runs, mode `off`) | Range |
|---|---|---|
| Parcels delivered in 300 s | 41.0; 95% prediction interval for one run 27–55 | 29–46 |
| Theft flagged | 8 of 8; median **0.8 s** after it starts | 0.4–1.5 s |
| Weak wheel flagged | 8 of 8; median **1.5 s** after it starts | up to 21.8 s, when the robot had no job and stood still |
| False alarms on healthy robots | **0** in 3.65 robot-hours of driving (under 0.82 per robot-hour, 95%) | |
| Stolen parcel, heartbeats only | never noticed (8 of 8) | |
| Closest two robots | 20.7 cm, 0 too-close steps (limit 13.5 cm) | |

Siegmund's formula expects 0.7 s for the weak wheel, for a robot driving straight at cruise speed. Real driving includes turning, lining up and docking slowly, where a weak wheel shows less, so the measured times are longer. In mode `on`: theft median 0.6 s, weak wheel 2.0 s, a stolen parcel reported in 8 of 8, 0 false alarms. Delivered parcels on vs off: 1.03× (95% CI 0.94–1.14), so acting on the witness costs no throughput we could measure.

**What W1 can change:** the stand-ins above. The real noise, wheel lag and the highest statistic on healthy robots are on its calibration page. If the real noise is larger, detection is slower. If the residuals are more correlated than in the simulator, the threshold gets raised before W2.

### Submission form, run W1

- **Title:** Every robot is a witness (run W1): catching a broken or hijacked robot without messages
- **Estimated Duration (seconds):** 420 (300 s of sorting, 20 s of results, plus driving to the start poses)
- **Number of Robots:** 8
- **Files:** `swarm_witness.py` from commit 402787c (`MODE = "off"`)
- **Experiment Description:**

> The same leaderless parcel sorting as our earlier experiment: 8 robots, virtual parcels projected on the floor, and every robot planning from the same shared snapshot. This run adds a "witness". Every robot runs the same planner on the same snapshot, so every robot can compute every robot's wheel commands, and from them where each robot should be next. Every snapshot, the script compares each robot's tracked motion with that prediction and runs a CUSUM change detector on the gap. The detector learns the fleet's wheel lag from the same data. To test it, the script injects two faults the planners aren't told about. At 90 s, one robot is sent what a robot with half its left wheel's power would do. At 210 s, one robot carrying a parcel drives it to the far wall instead of its dock, a "hijacked" robot. The simulated battery death at 150 s is kept from the earlier experiment. Every faulty robot's velocity goes through a barrier certificate of its own, solved against every other robot's actual velocity, so it does all the avoiding. In this run (W1) the witness only watches. Its verdicts and the measured tracking noise are projected at the end (two 10 s pages), and will set the threshold for a second run in which the swarm acts on them. Collision avoidance for the other robots is the same reimplementation of create_uni_barrier_certificate_with_boundary as before. It passed the Robotarium simulator with no errors or warnings in 17 of 17 runs (8 of them in this run's mode). Output: printed metrics and swarm_results.npz.
