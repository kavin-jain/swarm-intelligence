"""Satellite tests: wire format matches the C++ side byte for byte, and the vision
pipeline recovers known robot poses, tagged parcels and coloured pencils from a synthetic
camera frame.

  ../.venv/bin/python -m unittest -v test_satellite
  ../.venv/bin/python test_satellite.py bench      ms per frame: colour pipeline vs parcel tags
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np

import kpi
import proto
from vision import Tracker, Vision

HERE = os.path.dirname(os.path.abspath(__file__))


class Protocol(unittest.TestCase):
    def test_vision_matches_cpp_golden(self):
        # Same frame and bytes as test_vision_golden_bytes() in test/test_core.cpp.
        b = proto.pack_vision(7, [(1320, 500, 160)], (1500, 1000), [(2, 300, -40, 1.571)], [(9, 812, 433, 1, 40)])
        golden = bytes([1, 7, 0, 1, 0x28, 0x05, 0xF4, 0x01, 0xA0, 0x00, 0xDC, 0x05, 0xE8, 0x03, 1, 1,
                        2, 0x2C, 0x01, 0xD8, 0xFF, 0x23, 0x06, 9, 0x2C, 0x03, 0xB1, 0x01, 1, 20])
        self.assertEqual(b, golden)

    def test_parse_cpp_encoded_snapshot_and_heartbeat(self):
        # Bytes produced by core/proto.h encode() (build/emit_snap.cpp).
        snap = proto.parse(bytes.fromhex("022c01022805f401a0002805fa009600dc05e803020101fa00e2ff2306010404078403800248f400ff06042003a4011232"))
        self.assertEqual(snap["seq"], 300)
        self.assertEqual(snap["docks"], [(1320, 500, 160), (1320, 250, 150)])
        self.assertEqual(snap["robots"][0], {"id": 1, "x": 250, "y": -30, "th": 1.571, "alive": True, "task": 4, "state": 4})
        self.assertEqual(snap["robots"][1]["task"], proto.NONE)
        self.assertFalse(snap["robots"][1]["alive"])
        self.assertEqual(snap["objects"][0], {"id": 4, "x": 800, "y": 420, "demand": 2, "kind": 1, "status": 2, "r": 60})
        hb = proto.parse(bytes.fromhex("0303000203090902fc1c"))
        self.assertEqual(hb, {"type": "heartbeat", "id": 3, "seen": 512, "state": 3, "task": 9, "help": 9, "neighbors": 2, "batt_mv": 7420})

    def test_deframer_resyncs_and_rejects_corruption(self):
        good = proto.frame(b"\x04\x01")
        bad = bytearray(proto.frame(b"\x04\x00")); bad[3] ^= 0xFF
        d = proto.Deframer()
        out = d.feed(b"\x00\xA5\x13" + bytes(bad) + good[:3]) + d.feed(good[3:])
        self.assertEqual(out, [b"\x04\x01"])

    def test_rounding_matches_cpp(self):
        # C++ clamp16 rounds half away from zero; Python's round() is banker's rounding.
        self.assertEqual(proto._i16(2.5), 3)
        self.assertEqual(proto._i16(-2.5), -3)
        self.assertEqual(proto._i16(99999), 32767)


def scene(robots, pencils, corners=True, margin=150, parcels=()):
    """Render the arena from above (1 px = 1 mm, arena y pointing up), then view it through
    a tilted camera. Returns the camera frame. parcels: (tag id, x, y, heading[, BGR box colour])."""
    W, H = 1500, 1000
    canvas = np.full((H + 2 * margin, W + 2 * margin, 3), (205, 200, 195), np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    def px(x, y):
        return x + margin, (H - y) + margin

    def paste_marker(mid, x, y, size, heading):
        m = cv2.aruco.generateImageMarker(d, mid, size)
        m = cv2.copyMakeBorder(m, size // 6, size // 6, size // 6, size // 6, cv2.BORDER_CONSTANT, value=255)
        s = m.shape[0]
        rot = cv2.getRotationMatrix2D((s / 2, s / 2), math.degrees(heading - math.pi / 2), 1.0)
        # Rotate on a bigger tile so corners aren't clipped; outside of it stays transparent.
        big = int(s * 1.5)
        rot[0, 2] += (big - s) / 2; rot[1, 2] += (big - s) / 2
        tile = cv2.warpAffine(m, rot, (big, big), borderValue=0)
        alpha = cv2.warpAffine(np.full((s, s), 255, np.uint8), rot, (big, big), borderValue=0)
        cx, cy = px(x, y)
        x0, y0 = int(cx - big / 2), int(cy - big / 2)
        region = canvas[y0:y0 + big, x0:x0 + big]
        a = alpha[:region.shape[0], :region.shape[1], None] > 0
        region[:] = np.where(a, cv2.cvtColor(tile[:region.shape[0], :region.shape[1]], cv2.COLOR_GRAY2BGR), region)

    if corners:
        for mid, (x, y) in {0: (0, 0), 1: (W, 0), 2: (W, H), 3: (0, H)}.items():
            paste_marker(mid, x, y, 90, math.pi / 2)
    for x, y, length, width, ang, *colour in pencils:
        box = cv2.boxPoints(((px(x, y)), (length, width), -math.degrees(ang)))
        cv2.fillConvexPoly(canvas, np.int32(box), colour[0] if colour else (0, 210, 240))   # default yellow (BGR)
    for tag, x, y, ang, *colour in parcels:   # a 90 mm cardboard box with a 50 mm tag on its lid
        box = cv2.boxPoints(((px(x, y)), (90, 90), -math.degrees(ang)))
        cv2.fillConvexPoly(canvas, np.int32(box), colour[0] if colour else (90, 140, 190))
        paste_marker(tag, x, y, 50, ang)
    for rid, x, y, th in robots:
        cv2.circle(canvas, px(x, y), 60, (60, 60, 60), -1)                 # chassis
        paste_marker(10 + rid, x, y, 70, th)
    # Tilted camera: a mild keystone, like a phone on a stand not perfectly overhead.
    ch, cw = canvas.shape[:2]
    src = np.float32([[0, 0], [cw, 0], [cw, ch], [0, ch]])
    dst = np.float32([[160, 40], [1760, 70], [1830, 1050], [90, 1020]])
    return cv2.warpPerspective(canvas, cv2.getPerspectiveTransform(src, dst), (1920, 1080), borderValue=(40, 40, 40))


class VisionPipeline(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(HERE, "config.json")) as f:
            self.cfg = json.load(f)
        self.cfg["camera_height_mm"] = 0          # synthetic markers lie on the floor plane
        self.cfg["robot_marker_height_mm"] = 0

    def test_recovers_robot_poses_and_pencils(self):
        robots = [(1, 300, 250, 0.0), (2, 700, 650, 2.2), (3, 1100, 300, -1.0)]
        pencils = [(500, 500, 175, 8, 0.3), (900, 800, 175, 8, -1.2), (1250, 650, 175, 8, 1.57)]
        frame = scene(robots, pencils)
        robots_seen, loads = Vision(self.cfg).process(frame, 0.0)
        self.assertEqual(sorted(r[0] for r in robots_seen), [1, 2, 3])
        for rid, x, y, th in robots_seen:
            _, tx, ty, tth = robots[rid - 1]
            self.assertLess(math.hypot(x - tx, y - ty), 10, f"robot {rid} position")
            self.assertLess(abs(math.atan2(math.sin(th - tth), math.cos(th - tth))), math.radians(3), f"robot {rid} heading")
        self.assertEqual(len(loads), 3, loads)
        for _, x, y, kind, rad in loads:
            self.assertEqual(kind, 0)
            self.assertLess(min(math.hypot(x - px, y - py) for px, py, *_ in pencils), 10)
            self.assertLess(abs(rad - 175 / 2), 12)   # footprint: half the pencil's length

    def test_colour_picks_the_dock(self):
        cfg = dict(self.cfg, docks=self.cfg["docks"] + [{"x": 1320, "y": 250, "r": 150, "name": "blue",
                                                          "hsv_lo": [95, 120, 90], "hsv_hi": [125, 255, 255]}])
        blue = (230, 120, 20)   # BGR
        pencils = [(500, 500, 175, 8, 0.3), (900, 800, 175, 8, -1.2, blue), (1250, 650, 175, 8, 1.57, blue)]
        _, loads = Vision(cfg).process(scene([], pencils), 0.0)
        self.assertEqual(len(loads), 3, loads)
        for _, x, y, kind, _ in loads:
            truth = min(pencils, key=lambda p: math.hypot(x - p[0], y - p[1]))
            self.assertEqual(kind, 1 if len(truth) > 5 else 0, (x, y))

    def test_robot_body_never_detected_as_a_load(self):
        cfg = dict(self.cfg, docks=[dict(self.cfg["docks"][0], hsv_lo=[0, 0, 40], hsv_hi=[180, 60, 120])])  # grey = chassis colour
        _, loads = Vision(cfg).process(scene([(1, 750, 500, 0.5)], []), 0.0)
        self.assertEqual(loads, [])

    def test_tagged_parcels(self):
        # Tag number = parcel id and (counted across the docks) its dock. A yellow box with a tag
        # is still one parcel, not two.
        cfg = dict(self.cfg, docks=self.cfg["docks"] + [dict(self.cfg["docks"][0], y=250, name="blue")])
        parcels = [(21, 450, 300, 0.4), (22, 800, 700, -0.9), (23, 1100, 550, 2.0, (0, 210, 240)), (30, 600, 820, 1.1)]
        robots = [(1, 300, 250, 0.0), (2, 1000, 300, 2.2)]
        for colour in (True, False):
            robots_seen, loads = Vision(dict(cfg, colour_loads=colour)).process(scene(robots, [], parcels=parcels), 0.0)
            self.assertEqual(sorted(r[0] for r in robots_seen), [1, 2])
            self.assertEqual([l[0] for l in loads], [21, 22, 23, 30], loads)
            for (tag, x, y, kind, rad), (_, tx, ty, *_) in zip(loads, sorted(parcels)):
                self.assertLess(math.hypot(x - tx, y - ty), 5, f"parcel {tag} position")
                self.assertEqual(kind, (tag - 21) % 2)
                self.assertEqual(rad, 45)

    def test_mirrored_corner_layout_is_refused(self):
        cfg = dict(self.cfg)
        c = cfg["corner_markers"]
        cfg["corner_markers"] = {"0": c["0"], "1": c["3"], "2": c["2"], "3": c["1"]}   # 1 and 3 swapped
        with self.assertRaisesRegex(ValueError, "mirrored"):
            Vision(cfg).process(scene([], []), 0.0)

    def test_tracker_keeps_ids(self):
        t = Tracker()
        a = t.update([(100, 100, 0, 40), (500, 500, 0, 40)], 0.0)
        b = t.update([(510, 505, 0, 40), (104, 98, 0, 41)], 0.1)    # moved a little, listed in the other order
        self.assertEqual({i for i, *_ in a}, {i for i, *_ in b})
        self.assertEqual(dict((i, (round(x), round(y))) for i, x, y, *_ in b)[a[0][0]], (104, 98))
        c = t.update([(104, 98, 0, 40)], 2.0)                    # second one gone for > keep_s
        self.assertEqual(len(c), 1)
        d = t.update([(106, 99, 1, 40)], 2.1)                    # different colour on the same spot = a different load
        self.assertNotEqual(d[0][0], c[0][0])


class Kpis(unittest.TestCase):
    def write(self, rows, docks=((1320, 500, 160),)):
        f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
        f.write("# arena 1500 1000\n" + "".join(f"# dock {x} {y} {r}\n" for x, y, r in docks) + "t,what,id,x,y,th,kind,status,state\n")
        f.write("".join(",".join(map(str, r)) + "\n" for r in rows))
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_hand_made_log(self):
        rows = []
        for k in range(46):                     # 0..4.5 s at 10 fps
            t = k / 10
            x = 600 + max(0, t - 1) * 360       # parcel 5 waits 1 s, rides 720 mm into the dock (x 1320) and sits there 1.5 s
            rows += [(t, "parcel", 5, min(x, 1320), 500, 0, 0, -1, -1), (t, "robot", 1, min(x, 1320) - 105, 500, 0, -1, -1, -1)]
            rows += [(t, "robot", 2, 300, 200 + (0 if k < 10 or k > 20 else 100), 0, -1, -1, -1), (t, "robot", 3, 300, 400, 0, -1, -1, -1)]
            rows += [(t, "parcel", 7, 30 + 10 * k, 980, 0, 0, -1, -1)]          # dragged along the top wall (arena y = 1000)
        r = kpi.kpis(self.write(rows))
        self.assertEqual(r["parcels seen"], 2)
        self.assertEqual(r["delivered"], 1)
        self.assertAlmostEqual(r["wait for pickup mean s"], 0.7, delta=0.05)    # parcel 5 after 1.1 s, parcel 7 (10 mm a frame) after 0.3 s
        self.assertEqual(r["collisions"], 1)                                       # robots 2 and 3 touched once
        self.assertGreater(r["parcel scraping a wall, s"], 0.5)

    def test_satellite_log_is_what_kpi_reads(self):
        import satellite
        with open(os.path.join(HERE, "config.json")) as f:
            cfg = json.load(f)
        snap = {"objects": [{"id": 21, "status": 1}]}
        text = satellite.log_header(cfg) + "".join(
            satellite.log_rows(k / 15, [(1, 200.0 + 10 * k, 300.0, 0.0)], [(21, 1320.0, 500.0, 0, 45.0)], snap, {1: {"state": 9}}) for k in range(30))
        path = tempfile.mktemp(suffix=".csv")
        with open(path, "w") as f:
            f.write(text)
        self.addCleanup(os.unlink, path)
        r = kpi.kpis(path)
        self.assertEqual((r["parcels seen"], r["delivered"]), (1, 1))
        self.assertEqual(r["robot travel m"][1], 0.3)   # 29 steps of 10 mm, reported to 0.1 m

    @unittest.skipUnless(os.path.exists(os.path.join(HERE, "..", "build", "sim")), "build/sim not built")
    def test_agrees_with_the_simulator(self):
        # The simulator logs what a camera would see; kpi.py, from positions alone, must recover
        # the simulator's own ground-truth counts.
        path = tempfile.mktemp(suffix=".csv")
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        out = subprocess.run([os.path.join(HERE, "..", "build", "sim"), "--inbound", "2", "--log", path], capture_output=True, text=True, check=True).stdout
        m = re.search(r"log check: (\d+) landed, (\d+) delivered, wait from landing mean ([\d.]+) s, landing to dock mean ([\d.]+) s, collisions (\d+)", out)
        landed, done, wait, cycle, coll = int(m[1]), int(m[2]), float(m[3]), float(m[4]), int(m[5])
        r = kpi.kpis(path)
        self.assertEqual((r["parcels seen"], r["delivered"], r["collisions"]), (landed, done, coll))
        self.assertLess(abs(r["wait for pickup mean s"] - wait), 0.6)   # the camera sees the parcel move a beat after the grip
        self.assertLess(abs(r["arrival to dock mean s"] - cycle), 0.3)


def bench(n=30):
    """Per-frame cost, accuracy and robustness to lighting on the same synthetic floor: 3 robots
    and 8 parcels, found by colour (yellow boxes) or by their tags (cardboard boxes, tag on top).
    Lighting: as rendered, dim (x0.45), and warm (sodium-like: blue cut, red boosted)."""
    with open(os.path.join(HERE, "config.json")) as f:
        cfg = dict(json.load(f), camera_height_mm=0, robot_marker_height_mm=0)
    cfg["parcel_tags"] = dict(cfg["parcel_tags"], height_mm=0)
    robots = [(1, 300, 250, 0.0), (2, 700, 650, 2.2), (3, 1100, 300, -1.0)]
    spots = [(450, 400), (600, 800), (850, 250), (950, 600), (1200, 820), (1250, 450), (380, 700), (700, 200)]
    tagged = scene(robots, [], parcels=[(21 + i, x, y, 0.3 * i) for i, (x, y) in enumerate(spots)])
    coloured = scene(robots, [(x, y, 90, 90, 0.3 * i) for i, (x, y) in enumerate(spots)])
    light = {"as rendered": lambda f: f, "dim": lambda f: (f * 0.45).astype(np.uint8),
             "warm": lambda f: np.clip(f * np.float32([0.55, 0.9, 1.25]), 0, 255).astype(np.uint8)}
    for name, frame, c in (("colour", coloured, dict(cfg, parcel_tags={})), ("tags", tagged, dict(cfg, colour_loads=False))):
        for lname, fx in light.items():
            v, f = Vision(c), fx(frame)
            v.process(f, 0.0)
            t0 = time.perf_counter()
            for k in range(n):
                robots_seen, loads = v.process(f, 0.1 * (k + 1))
            ms = (time.perf_counter() - t0) / n * 1000
            err = max(min(math.hypot(x - sx, y - sy) for sx, sy in spots) for _, x, y, *_ in loads) if loads else float("nan")
            print(f"{name:7s} {lname:12s} {ms:5.1f} ms/frame | robots {len(robots_seen)}/3 | parcels {len(loads)}/8 | worst parcel error {err:.1f} mm")


if __name__ == "__main__":
    if sys.argv[1:] == ["bench"]:
        bench()
    else:
        unittest.main()
