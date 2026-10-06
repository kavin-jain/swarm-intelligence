"""The 'satellite': turns an overhead camera frame into arena coordinates.

Corner ArUco markers give a homography from the image to the arena floor (mm). Robot
markers give each robot's position and heading. Parcels carry an ArUco tag too: it's found in
the same detection pass as the robots (no extra work per frame), its number is the parcel's ID
and picks its dock, and it needs no colour tuning. Untagged loads can still be found by colour
on a top-down warp of the floor, with robots and tagged parcels masked out, and tracked so they
keep their IDs. Each dock in the config owns one colour range: a load's colour is its kind.
"""
import math

import cv2
import numpy as np


class Tracker:
    """Nearest-neighbour tracking so a pencil keeps the same ID from frame to frame.
    Points are (x, y, kind, radius_mm); a track only ever matches a detection of its own colour."""

    def __init__(self, max_jump_mm=80.0, keep_s=1.0, reserved=()):
        self.tracks = {}            # id -> [x, y, last_seen, kind, radius]
        self.next_id = 1
        self.reserved = set(reserved)   # parcel tag numbers: never handed out to colour tracks
        self.max_jump, self.keep = max_jump_mm, keep_s

    def update(self, points, t):
        free = dict(self.tracks)
        out = []
        # Greedy: closest (track, detection) pairs first.
        pairs = sorted(((math.hypot(px - tx, py - ty), tid, i) for i, (px, py, pk, _) in enumerate(points)
                        for tid, (tx, ty, _, tk, _) in free.items() if pk == tk), key=lambda p: p[0])
        used_t, used_p = set(), set()
        for d, tid, i in pairs:
            if d > self.max_jump or tid in used_t or i in used_p:
                continue
            used_t.add(tid); used_p.add(i)
            self.tracks[tid] = [points[i][0], points[i][1], t, points[i][2], points[i][3]]
        for i, (px, py, pk, pr) in enumerate(points):
            if i not in used_p:
                tid = self._new_id()
                self.tracks[tid] = [px, py, t, pk, pr]
                used_t.add(tid)
        for tid in list(self.tracks):
            if t - self.tracks[tid][2] > self.keep:
                del self.tracks[tid]
        return [(tid, x, y, k, r) for tid, (x, y, seen, k, r) in sorted(self.tracks.items()) if seen == t]

    def _new_id(self):
        while self.next_id in self.tracks or self.next_id in self.reserved or self.next_id == 0 or self.next_id == 0xFF:
            self.next_id = self.next_id % 250 + 1
        tid = self.next_id
        self.next_id = self.next_id % 250 + 1
        return tid


class Vision:
    def __init__(self, cfg):
        self.cfg = cfg
        dict_id = getattr(cv2.aruco, cfg["aruco_dictionary"])
        # Classic detector on purpose: ArUco3 (useAruco3Detection) was measured on the synthetic
        # floor and only got faster by dropping 50 mm parcel tags; at settings that kept all 15
        # markers it was no faster (3.1 vs 3.0 ms per 1080p frame).
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dict_id), cv2.aruco.DetectorParameters())
        self.H = None                # image -> arena mm (floor plane)
        lo, hi = cfg.get("parcel_tags", {}).get("ids", [0, -1])
        self.tag_ids = range(lo, hi + 1)
        self.tracker = Tracker(reserved=self.tag_ids)
        self.mmpp = cfg["objects"]["mm_per_px"]

    # ---- geometry ----
    def _to_arena(self, pts):
        p = cv2.perspectiveTransform(np.asarray(pts, np.float32).reshape(-1, 1, 2), self.H)
        return p.reshape(-1, 2)

    def _parallax(self, p, height):
        """A marker `height` mm above the floor appears pushed away from the camera's nadir.
        Pull it back: true = nadir + (seen - nadir) * (1 - h / camera_height)."""
        ch = self.cfg.get("camera_height_mm", 0)
        if not ch or not height:
            return p
        return self.nadir + (p - self.nadir) * (1 - height / ch)

    def update_homography(self, ids, corners, frame_shape):
        want = self.cfg["corner_markers"]
        img_pts, arena_pts = [], []
        for mid, xy in want.items():
            if int(mid) in ids:
                img_pts.append(corners[ids.index(int(mid))].mean(axis=0))
                arena_pts.append(xy)
        if len(img_pts) == 4:   # camera is fixed: keep the last good calibration otherwise
            H = cv2.getPerspectiveTransform(np.float32(img_pts), np.float32(arena_pts))
            # Handedness: robots turn left for positive angles, so the arena frame must be
            # counter-clockwise seen from above. Image coords (y down) are clockwise from
            # above, so a correct mapping flips handedness: Jacobian determinant < 0.
            h, w = frame_shape[:2]
            c = np.float32([[w / 2, h / 2], [w / 2 + 1, h / 2], [w / 2, h / 2 + 1]]).reshape(-1, 1, 2)
            a = cv2.perspectiveTransform(c, H).reshape(-1, 2)
            u, v = a[1] - a[0], a[2] - a[0]
            det = u[0] * v[1] - u[1] * v[0]
            if det > 0:
                raise ValueError("corner markers are mirrored: seen from above they must go 0 -> 1 -> 2 -> 3 "
                                 "counter-clockwise. Swap markers 1 and 3 (or their coordinates in the config).")
            self.H = H
            self.nadir = self._to_arena([[w / 2, h / 2]])[0]   # good enough for a near-overhead camera
        return self.H is not None

    # ---- per frame ----
    def process(self, frame, t):
        cfg = self.cfg
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        ids = [] if ids is None else [int(i) for i in ids.ravel()]
        corners = [c.reshape(4, 2) for c in corners]
        if not self.update_homography(ids, corners, frame.shape):
            return None
        robots = []
        base, maxr = cfg["robot_marker_base"], cfg["max_robots"]
        for mid, c in zip(ids, corners):
            if not base < mid <= base + maxr:
                continue
            rid = mid - base
            top, bottom = (c[0] + c[1]) / 2, (c[2] + c[3]) / 2   # marker's top edge = robot's front
            a = self._to_arena([c.mean(axis=0), top, bottom])
            h = cfg.get("robot_marker_height_mm", 0)
            centre, top, bottom = (self._parallax(p, h) for p in a)
            th = math.atan2(top[1] - bottom[1], top[0] - bottom[0])
            th += math.radians(cfg.get("heading_offset_deg", {}).get(str(rid), 0))
            robots.append((rid, float(centre[0]), float(centre[1]), math.atan2(math.sin(th), math.cos(th))))
        tagged = self.find_tagged(ids, corners)
        colour = self.find_loads(frame, robots, corners, tagged) if cfg.get("colour_loads", True) else []
        objects = sorted(self.tracker.update(colour, t) + tagged)
        return robots, objects

    def find_tagged(self, ids, corners):
        """Parcels with an ArUco tag on top: (tag id, x, y, kind, radius). Kind (= dock) is the
        tag number counted across the docks: tag lo -> dock 0, lo+1 -> dock 1, ..."""
        pt = self.cfg.get("parcel_tags", {})
        out = []
        for mid, c in zip(ids, corners):
            if mid in self.tag_ids:
                x, y = self._parallax(self._to_arena([c.mean(axis=0)])[0], pt.get("height_mm", 0))
                out.append((mid, float(x), float(y), (mid - self.tag_ids.start) % max(1, len(self.cfg["docks"])), float(pt.get("radius_mm", 40))))
        return out

    def find_loads(self, frame, robots, corners, tagged=()):
        cfg, o = self.cfg, self.cfg["objects"]
        w, h = int(cfg["arena_w"] / self.mmpp), int(cfg["arena_h"] / self.mmpp)
        S = np.array([[1 / self.mmpp, 0, 0], [0, 1 / self.mmpp, 0], [0, 0, 1]], np.float64)
        top = cv2.warpPerspective(frame, S @ self.H, (w, h))
        hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
        # Robots and markers are never loads: blank them out.
        blank = np.full(hsv.shape[:2], 255, np.uint8)
        r_px = int(cfg["robot_mask_radius_mm"] / self.mmpp)
        for _, x, y, _ in robots:
            cv2.circle(blank, (int(x / self.mmpp), int(y / self.mmpp)), r_px, 0, -1)
        for c in corners:
            poly = (self._to_arena(c) / self.mmpp).astype(np.int32)
            cv2.fillConvexPoly(blank, poly, 0)
        for _, x, y, _, r in tagged:   # a tagged parcel may be coloured too: count it once
            cv2.circle(blank, (int(x / self.mmpp), int(y / self.mmpp)), int((r + 10) / self.mmpp), 0, -1)
        k = np.ones((3, 3), np.uint8)
        found = []
        for kind, dock in enumerate(cfg["docks"]):
            mask = cv2.inRange(hsv, np.array(dock["hsv_lo"]), np.array(dock["hsv_hi"])) & blank
            mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k, iterations=2)
            for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                area = cv2.contourArea(cnt) * self.mmpp ** 2
                if o["min_area_mm2"] <= area <= o["max_area_mm2"]:
                    m = cv2.moments(cnt)
                    _, rad = cv2.minEnclosingCircle(cnt)   # footprint the robots plan around and grip against
                    found.append((m["m10"] / m["m00"] * self.mmpp, m["m01"] / m["m00"] * self.mmpp, kind, rad * self.mmpp))
        return found
