"""Satellite tests: wire format matches the C++ side byte for byte, and the vision
pipeline recovers known robot poses and pencil positions from a synthetic camera frame.

  ../.venv/bin/python -m unittest -v test_satellite
"""
import json
import math
import os
import unittest

import cv2
import numpy as np

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


def scene(robots, pencils, corners=True, margin=150):
    """Render the arena from above (1 px = 1 mm, arena y pointing up), then view it through
    a tilted camera. Returns the camera frame."""
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


if __name__ == "__main__":
    unittest.main()
