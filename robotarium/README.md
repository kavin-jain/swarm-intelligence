# Robotarium experiment: leaderless parcel sorting on real robots

The swarm's coordination layer, run on the [Robotarium](https://www.robotarium.gatech.edu/), Georgia Tech's remotely accessible robot swarm (free for education and research). Our own robots don't exist in hardware yet; this is the cheapest honest way to run the coordination on real robots: real motors, real tracking, real lag.

## What is real, what is virtual

| Part | In this experiment | In the full system |
|---|---|---|
| Robots | **Real**: 8 Robotarium robots (11 cm, 0.2 m/s max) | ESP32 robots (12 cm, 0.26 m/s) |
| Seeing the floor | **Real**: the Robotarium's tracking (`get_poses()`), shared as one snapshot ~10 times a second | One overhead phone camera, shared by radio (ESP-NOW) ~10 times a second |
| Deciding who does what | **Ours**, ported from `core/brain.h`: one brain per robot, each planning from the same snapshot | The same planner, compiled into each robot's firmware |
| Avoiding collisions | **Theirs**: the Robotarium's barrier certificates, which the Robotarium requires | Our path planner plus steering |
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
- **Collision avoidance** is theirs, not ours (required).

Colours: blue parcels go to dock A, orange to dock B; the crate has a black outline. Robot rings: grey idle, yellow heading to a parcel, green carrying, red dead. The counter at top left shows time, parcels delivered, parcels on the floor, and the plan check ("disagreements in … snapshots").

## Timeline (300 s after the robots reach their start poses)

| Time | Event |
|---|---|
| 0 s | Shift starts. A parcel lands about every 6 s (Poisson), at bay 1 or bay 2 (60/40) |
| 45 s | A heavy crate lands at bay 1. Two robots line up across its face, grip together and carry it to dock B |
| 150 s | Robot 3's battery "dies": it stops where it is and drops what it carries. The others route round it and pick up its parcel |
| any time | If every carrier goes 20 s without getting 3 cm nearer its dock, the one farthest from its dock sets its parcel down (deadlock recovery by preemption; that parcel waits 30 s before anyone picks it up again) |
| 300 s | End. The script prints the results and saves `swarm_results.npz` |

## What success looks like (checked in the results)

1. **Zero collisions and nobody outside the arena**: the Robotarium's own checker (`r.debug()`).
2. **0 plan disagreements**: every snapshot, all 8 brains plan from the robots listed in a different order, and must agree.
3. **Throughput and waits**: parcels delivered, mean wait from landing to pickup, mean landing-to-dock time. These are compared with the Robotarium's own simulator and with our simulator.
4. **The crate delivered by a pair, and the dead robot's parcel delivered by someone else.**

## Running it

- **In the Robotarium simulator (on a GitHub runner, not the laptop):** every push to `robotarium/` runs `.github/workflows/robotarium.yml`. It gives the simulator's verdict, the metrics and a 3× preview video (artifact `robotarium`).
- **On the Robotarium:** fill in New Experiment with the text below and upload `swarm_sort.py`.

## Submission form

- **Title:** Leaderless parcel sorting: lockstep task allocation with dock admission control
- **Estimated Duration (seconds):** 400 (300 s of sorting plus driving to the start poses)
- **Number of Robots:** 8
- **Files:** `swarm_sort.py`
- **Experiment Description:**

> Eight robots sort virtual parcels (projected squares) from two unloading bays to two docks, with no central planner in the decision-making. Every robot runs its own copy of the same deterministic planner on the same shared snapshot (poses plus parcel states, about 10 Hz), so they agree on who takes which parcel without negotiating. The script checks this each snapshot by giving every brain the robots in a different order. The planner includes dock admission control (start a pickup only if a dock slot is free or one of 2 staging places beside it), a pair recruited for a heavy crate, a robot that "fails" at 150 s (its parcel is reallocated), and deadlock recovery by preemption. Collision avoidance uses the Robotarium's unicycle barrier certificates with boundary. It is a port of the coordination layer of an ESP32 swarm (github.com/kavin-jain/swarm-intelligence), whose own robots are not built yet; this run tests it on real hardware. Output: printed metrics and swarm_results.npz (parcels delivered, wait times, plan disagreements).
