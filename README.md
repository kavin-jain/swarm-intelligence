# Swarm Intelligence: warehouse robots without a central brain

Amazon-style warehouse robots (Kiva) need a central planning server and codes stuck all over the floor. That puts them out of reach of a small warehouse. These are cheap ESP32 robots that **pick up parcels and carry them** to the right dock, and that **plan and coordinate on board**. There's no server deciding anything: every robot runs the same planner on the same shared picture, so they agree without negotiating. They split the work, call a partner for a crate too heavy for one, and keep going when a robot dies. A phone above the floor only *sees*.

![Swarm robots](./assets/preview.jpg)

**▶ Play with it: [kavinjain.in/swarm](https://kavinjain.in/swarm).** The robots' real firmware logic, compiled to WebAssembly and running in your browser.

> **Status:** proven in simulation. The robot brain, gateway, camera pipeline and firmware are complete and tested (below). **Carrying needs a gripper.** The physical prototype robots don't have one yet, so their firmware runs in push mode (`GRIPPER_PIN -1`) until one is fitted. Not yet run on the physical robots: [Bring-up](#bring-up-on-real-hardware) is the procedure.

---

## What it does

| Situation | What the swarm does |
|---|---|
| 5 parcels, 2 robots | Each robot grips a different parcel and carries it to a free dock slot; the work is split. |
| A parcel against a wall or in a corner | The robot grips it from the open side, backs it off the wall, and turns it round in the open. |
| A crate too heavy for one | The robot that tried feels it won't move and asks for help. Two robots line up side by side, grip one face together, and drive as one wide vehicle. |
| A robot's battery dies | Its gripper lets go. The others stop giving it work, route round it, and pick up its parcel. |
| Two colours, two docks | Each parcel goes to the dock for its colour. |
| A crowded floor | Robots only start a job whose dock has a free slot with a clear approach lane, and only take parcels they can actually get to, so a packed area is cleared from the outside in. |

### How the robots decide (all in [`core/brain.h`](core/brain.h))

1. **Count who's here.** Every robot broadcasts a heartbeat 10× per second over ESP-NOW and counts the ones it hears.
2. **Split the work, with no leader.** Each robot runs the same deterministic allocator on the same snapshot, so they all reach the identical plan without negotiating. This is unit-tested. The allocator also keeps a robot on the parcel it's holding, never over-fills a dock, and skips parcels nobody can reach.
3. **Grip it properly.** A robot lines up, drives straight in slowly, grips, then **tugs**, backing up a few cm: a held parcel follows and a missed one doesn't. Without a gripper sensor, that's how the camera can tell holding from pushing.
4. **Carry without touching anything.** Each robot plans its own route (BFS on a 5 cm grid, re-planned every tick). A held parcel gets the full clearance it can swing through from the walls, and steering guards keep the robot and its parcel off every other parcel.
5. **Recruit when it's heavy.** If the wheels drive but nothing moves, the robot asks for help, the parcel's headcount goes up, and a pair forms. If every robot has tried, the parcel is flagged for a human.

## How it's wired

```mermaid
flowchart LR
    CAM["Phone camera<br/>(overhead)"] --> SAT["satellite.py on laptop<br/>ArUco → mm, loads by colour + size"]
    SAT -- "USB serial<br/>camera frame" --> GW["Gateway ESP32<br/>core/world.h"]
    GW -- "ESP-NOW broadcast<br/>shared snapshot, 15 Hz" --> R1["Robot 1<br/>core/brain.h"] & R2["Robot 2"] & R3["Robot 3"]
    R1 & R2 & R3 -- "heartbeats 10 Hz<br/>(state, job, help, neighbours)" --> GW
    R1 -. "heartbeats = discovery" .- R2
```

- **Sees:** `satellite/` reads ArUco markers at the arena corners, which give a homography from pixels to millimetres, plus one marker on each robot for position and heading. Parcels are found by colour, sized (minimum enclosing circle) and tracked. Each dock in `config.json` owns one colour.
- **Relays:** the gateway ESP32 merges camera frames and heartbeats into one snapshot. It decides nothing.
- **Decides:** every robot. The same `core/` headers compile into the firmware, the unit tests, the simulator and the website, so what's tested is what runs.
- **Fails to stopped:** no snapshot for 0.5 s, an e-stop, or low battery, and the motors cut out. A held parcel stays held.

## Tested

`bash run_all.sh` runs everything. The simulator uses the real wire format, physics with camera noise (3 mm / ~1°), motor mismatch (±8%), radio loss, 80 ms camera lag and a 5% gripper miss rate. Carry and push are compared on the same seeds, with docks shipping parcels after 5 s in both modes.

| Benchmark | Carry (gripper) | Push (no gripper) |
|---|---|---|
| 8 scenarios × 60 seeds | **479/480** (99.8%) | 470/480 |
| 500 random floors | **492/500** | 489/500 |
| 200 floors with loads right against the walls | **198/200** | ~133/200 |
| 40 dense floors (6–10 robots, 10–14 loads, 2 docks) | **36/40**, 0 collision events | 2/40, 3,085 collision events |
| Robot–robot collisions, seeded runs | **0** | 0 |
| Loads scraped along a wall, seeded runs | **0** | 230 steps (sorting) |
| Loads knocked by robots | ~1.6 mm per run | n/a (pushing is the method) |
| Energy per delivered load (model) | **21.0 mWh** (dense: 49.6) | 28.6 mWh (dense: 101.0) |

| Other checks | Result |
|---|---|
| Core unit tests (ASan + UBSan): wire format, allocation, leaderless consensus, delivery/recruit/stuck, docks, safety stop | 12/12 |
| Satellite (Python): bytes match the C++ side; synthetic tilted-camera frames: position, heading, parcel size, colour → dock | 9/9 · ≤ 1.3 mm · 0.4° |
| Firmware: `robot1-3`, `gateway`, `motortest` for ESP32 DevKit | builds clean |
| WebAssembly: same engine in the browser carries the 5-parcel, heavy-crate and sorting jobs | 58 KB, passes |

The energy figures come from a model, not a meter: electronics 0.5 W, TT motors ~2.5 W each, L298N ~25% loss, gripper 3 W to grab and 1 W to hold. They're for comparison until calibrated on the real robots.

## Hardware

Per robot: **ESP32 DevKit v1**, **L298N or L293D** driver, 2 DC gear motors, 2×18650, and an ArUco marker on top (top edge of the marker = robot's front). For carry mode, a **gripper** on the front: an electromagnet (e.g. ZYE1-P20/15, 12 V, 25 N) switched by a logic-level MOSFET with a flyback diode, with a steel washer on each parcel. Set `GRIPPER_PIN` in `robot_config.h`. The firmware drives it peak-and-hold: full power to grab, ~35% to hold. Also needed: one spare ESP32 as the gateway, a phone as the overhead camera, and a laptop.

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| Left EN (PWM) | 14 | | Right EN (PWM) | 32 |
| Left IN1 / IN2 | 27 / 26 | | Right IN1 / IN2 | 25 / 33 |
| Status LED | 2 | | Battery sense (optional) | any of 32–39, via divider |

All in [`firmware/include/robot_config.h`](firmware/include/robot_config.h). Those pins avoid the ESP32 flash pins (6–11), the input-only pins (34–39) and the boot-strapping pins. Battery sense must be on **ADC1**, because ADC2 stops working while the radio is on.

## Bring-up on real hardware

1. **Markers:** `cd satellite && ../.venv/bin/python satellite.py --markers markers/`, then print at 100% scale. Corners 0→1→2→3 go **counter-clockwise seen from above** (the satellite refuses a mirrored layout). Robot *N* wears marker `10+N`.
2. **Wiring check:** `cd firmware && pio run -e motortest -t upload`. Watch each wheel; set `INVERT_L/R` if one spins backwards, and `PWM_MIN` from the ramp.
3. **Calibrate:** with `motortest` running in the arena, run `python satellite.py --measure 1`. Paste the printed `CAL_VMAX` and `CAL_WHEEL_BASE` into `robot_config.h`.
4. **Flash:** `pio run -e robot1 -t upload` (then `robot2`, `robot3`) and `pio run -e gateway -t upload` for the ESP32 on the laptop.
5. **Run:** put the phone above the arena (Continuity Camera on a Mac shows up as camera 0/1), then run `python satellite.py`. Space bar = e-stop all.

Things to measure and set in `satellite/config.json`: arena size, camera height and marker height (for parallax correction), and the `docks` list. Each dock has a position, radius and the HSV colour range of the loads it receives (up to 3 docks; keep each one ≥ 11 cm clear of the walls, see Known limits).

## Roadmap

Status as of 2026-10-05. ✅ done · 🔨 in progress · ⬜ next

| # | Upgrade | Why it matters in a real warehouse | Status |
|---|---|---|---|
| 1 | Run on the physical robots | Proof. Everything else is a model until then | ⬜ needs hardware time |
| 2 | Carry instead of push (gripper) | Walls and corners stop mattering; how real warehouse robots work. Done 2026-10-06 in software and simulation, incl. two-robot carries; needs a gripper fitted to test for real | ✅ software · ⬜ hardware |
| 3 | Motion prediction between camera frames (+ wheel encoders/IMU) | Tested in the simulator 2026-10-05: **no gain for pushing robots.** On the 20-seed set, commanded-wheel prediction: 137/140; simulated encoders: 128–138/140 vs 140/140 baseline. Higher speed loses loads; latency isn't the bottleneck. Revisit after #2 (lift), when carried loads can't be lost | ⏸ parked, evidence |
| 4a | Sort by colour to several docks | Real warehouses sort. Done 2026-10-05: up to 3 docks, colour from the camera, 59/60 seeds in the two-dock scenario | ✅ |
| 4b | Traffic at scale | Done 2026-10-06 as admission control (no job without a free, reachable dock slot) and reachability-first allocation: dense floors 36/40 with 0 collisions. Announced-path reservations not needed so far: collisions measure 0 | ✅ |
| 4c | Plan the next 2–3 pickups | Chaining cuts empty travel | ⬜ |
| 5 | Scale past one camera: on-robot localisation + an orders API | Any floor size; a warehouse system can hand the swarm real work | ⬜ design |
| 6 | Battery-aware jobs + charging dock | Robots that run all day: low robots take short jobs, then charge themselves | ⬜ |
| 7 | Live digital twin: the real floor streamed to the website | Anyone can watch the real warehouse working, live | ⬜ needs #1 |
| 8 | Safety zones: stop near hands/people the camera sees | Robots that can share a floor with humans | ⬜ |
| 9 | Self-diagnosis: a robot notices its own stuck wheel or slipping motor | Flags itself for maintenance instead of silently slowing the floor | ⬜ |
| 10 | Throughput and energy KPIs | What a warehouse manager actually buys. Energy per load done (model): carry uses 27–51% less than push | 🔨 energy done |
| 11 | Priority/express orders | Urgent parcels first, like real fulfilment | ⬜ |
| 12 | Learn from experience: each load's real difficulty, each robot's real speed | Allocation that improves the longer it runs | ⬜ |

## Repo layout

```
core/       proto.h (wire format) · world.h (gateway bookkeeping) · brain.h (robot logic)
firmware/   PlatformIO: robot, gateway, motortest
satellite/  camera → arena coordinates, serial link, calibration, tests
sim/        engine.h (physics, camera and radio around the real brain) + scenarios
web/        WebAssembly build of the engine for the website (bash web/build.sh, needs zig)
test/       core unit tests
run_all.sh  every check
```

## Known limits

- **One camera.** The overhead phone is still the one shared sensor, and its view limits the floor size. The fix is on-robot localisation (roadmap #5); the decisions are already on the robots.
- **Dense floors.** 4 of 40 dense floors still don't finish: crowded two-robot crate jobs, where pairs wait on each other. Dock lanes must be kept clear (nothing stored within 40 cm of a dock), as in any warehouse.
- **Push mode (no gripper)** keeps its old limits: loads against walls get stuck (66% success with loads at the walls), and docks need ~11 cm clear behind them.
- **Sim ≠ floor.** Loads are modelled as discs; real pencils roll and pivot. Expect to tune `push_speed`, `stall_ms` and `PWM_MIN` on the real robots.
- **Parallax.** Robot markers sit above the floor, and the correction assumes a near-overhead camera. Tilt the phone as little as possible.
- **Packet size.** One snapshot carries up to 10 robots, 16 loads and 3 docks (≤ 250-byte ESP-NOW packet).

---

Built by [Kavin Jain](https://kavinjain.in).
