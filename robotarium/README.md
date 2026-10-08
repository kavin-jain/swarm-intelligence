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
| 300 s | End. The script prints the results and saves `swarm_results.npz` |

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

Read from the projected counters in the recording. 46 is 0.5 standard deviations above the simulator's mean: one run is consistent with the prediction, not proof of it. Not read yet: the collisions the Robotarium logged, wait times, and when the dead robot's parcel was delivered. Those are in the run's output log and `swarm_results.npz` on the Robotarium experiment page.

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
