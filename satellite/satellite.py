#!/usr/bin/env python3
"""Overhead 'satellite' camera for the swarm. It only reports what it sees; robots decide.

  python satellite.py                          run (camera 0, gateway auto-detected)
  python satellite.py --camera http://PHONE:8080/video   phone streaming as an IP camera
  python satellite.py --no-serial              vision only, no gateway attached
  python satellite.py --markers markers/       print-ready ArUco markers (corners, robots, parcel lids)
  python satellite.py --measure 1              calibrate robot 1 while it runs `motortest`

Keys in the window: SPACE = emergency stop toggle (all robots), Q = quit.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np

import proto
from vision import Vision

HERE = os.path.dirname(os.path.abspath(__file__))


def find_port():
    for pat in ("/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def open_camera(src):
    cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
    if not cap.isOpened():
        sys.exit(f"can't open camera {src!r} (on a Mac, Continuity Camera shows up as 0 or 1)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap


def make_markers(cfg, out, mm=80):
    """Write PNGs at 300 dpi: print at 100% so each marker is `mm` wide (plus a white border)."""
    os.makedirs(out, exist_ok=True)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["aruco_dictionary"]))
    px = int(mm / 25.4 * 300)
    jobs = [(int(k), f"corner_{k}", px) for k in cfg["corner_markers"]]
    jobs += [(cfg["robot_marker_base"] + i, f"robot_{i}", px) for i in range(1, 7)]
    lo, hi = cfg.get("parcel_tags", {}).get("ids", [0, -1])
    docks = [d_.get("name", str(k)) for k, d_ in enumerate(cfg["docks"])]
    jobs += [(t, f"parcel_{t}_to_{docks[(t - lo) % len(docks)]}", int(50 / 25.4 * 300)) for t in range(lo, hi + 1)]   # 50 mm, for box lids
    for mid, name, px in jobs:
        img = cv2.aruco.generateImageMarker(d, mid, px)
        img = cv2.copyMakeBorder(img, px // 8, px // 8, px // 8, px // 8, cv2.BORDER_CONSTANT, value=255)
        cv2.putText(img, f"{name} (id {mid}) - this edge = robot front" if name.startswith("robot") else f"{name} (id {mid})",
                    (10, px // 8 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
        cv2.imwrite(os.path.join(out, f"{name}_id{mid}.png"), img)
    print(f"wrote {len(jobs)} markers to {out}/ - print at 100% scale: {mm} mm squares for corners and robots, 50 mm for parcel lids")


def draw(frame, vis, robots, objects, snap, hbs, estop, cfg):
    """Camera view with the swarm's own state painted on (from the gateway's echo)."""
    if vis.H is None:
        cv2.putText(frame, "show all 4 corner markers to calibrate", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        return frame
    Hinv = np.linalg.inv(vis.H)

    def img(x, y):
        p = cv2.perspectiveTransform(np.float32([[[x, y]]]), Hinv)[0, 0]
        return int(p[0]), int(p[1])

    for i, z in enumerate(cfg["docks"]):
        pts = [img(z["x"] + z["r"] * math.cos(a), z["y"] + z["r"] * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 40)]
        cv2.polylines(frame, [np.int32(pts)], True, (0, 200, 0), 3)
        cv2.putText(frame, f"dock {i}: {z.get('name', '')}", img(z["x"] - z["r"], z["y"] + z["r"] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
    objs = {o["id"]: o for o in (snap or {}).get("objects", [])}
    for oid, x, y, kind, _ in objects:
        o = objs.get(oid, {})
        status = proto.STATUS[o.get("status", 0)] if o.get("status", 0) < 3 else "?"
        col = {"open": (0, 220, 255), "delivered": (0, 200, 0), "stuck": (0, 0, 255)}.get(status, (200, 200, 200))
        cv2.circle(frame, img(x, y), 12, col, 3)
        cv2.putText(frame, f"L{oid}>dock {kind} x{o.get('demand', 1)} {status}", img(x + 30, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    for rid, x, y, th in robots:
        hb = hbs.get(rid, {})
        state = proto.STATES[hb["state"]] if hb.get("state", 99) < len(proto.STATES) else "no radio"
        task = hb.get("task", proto.NONE)
        cv2.arrowedLine(frame, img(x, y), img(x + 90 * math.cos(th), y + 90 * math.sin(th)), (255, 120, 0), 3)
        label = f"R{rid} {state}" + (f" -> L{task}" if task != proto.NONE else "") + f" | sees {hb.get('neighbors', '?')}"
        if hb.get("help", proto.NONE) != proto.NONE:
            label += " HELP!"
        cv2.putText(frame, label, img(x - 60, y - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2)
    if estop:
        cv2.putText(frame, "EMERGENCY STOP (space to release)", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
    return frame


def measure(args, cfg, vis, cap):
    """Calibration: watch robot N run `motortest` and estimate vmax and wheel base."""
    rid, track, t0 = args.measure, [], time.time()
    print(f"watching robot {rid} for 40 s - flash `motortest` on it and put it in the arena")
    while time.time() - t0 < 40:
        ok, frame = cap.read()
        if not ok:
            continue
        res = vis.process(frame, time.time())
        if res:
            for r in res[0]:
                if r[0] == rid:
                    track.append((time.time(), r[1], r[2], r[3]))
        cv2.imshow("measure", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    if len(track) < 30:
        sys.exit("robot marker barely seen - check lighting / marker ID")
    v_best = w_best = 0.0
    for i in range(len(track)):
        for j in range(i + 1, len(track)):
            dt = track[j][0] - track[i][0]
            if 0.8 <= dt <= 1.2:
                v = math.hypot(track[j][1] - track[i][1], track[j][2] - track[i][2]) / dt
                dth = sum(math.atan2(math.sin(track[k + 1][3] - track[k][3]), math.cos(track[k + 1][3] - track[k][3])) for k in range(i, j))
                v_best, w_best = max(v_best, v), max(w_best, abs(dth) / dt)
                break
    wb = 2 * v_best / w_best if w_best else 0
    print(f"\n#define CAL_VMAX       {v_best:.0f}.0f   // mm/s, from the 2 s straight run")
    print(f"#define CAL_WHEEL_BASE {wb:.0f}.0f   // mm, from the spin (2 * v / omega)")
    print("paste these into firmware/include/robot_config.h and re-flash the robots")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="0")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--markers", metavar="DIR")
    ap.add_argument("--measure", type=int, metavar="ROBOT_ID")
    args = ap.parse_args()
    cfg = json.load(open(args.config))
    if args.markers:
        return make_markers(cfg, args.markers)

    vis = Vision(cfg)
    cap = open_camera(args.camera)
    if args.measure:
        return measure(args, cfg, vis, cap)

    ser = None
    if not args.no_serial:
        import serial
        port = find_port() if args.port == "auto" else args.port
        if not port:
            sys.exit("no gateway ESP32 found - plug it in, pass --port, or use --no-serial")
        ser = serial.Serial(port, cfg["serial_baud"], timeout=0)
        ser.write(proto.frame(proto.pack_estop(False)))   # clear a stop left latched by a previous session
        print(f"gateway on {port}")

    deframer, seq, last_send, estop = proto.Deframer(), 0, 0.0, False
    snap, hbs = None, {}
    docks = [(z["x"], z["y"], z["r"]) for z in cfg["docks"]]
    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera frame dropped"); time.sleep(0.05); continue
        now = time.time()
        try:
            res = vis.process(frame, now)
        except ValueError as e:
            sys.exit(str(e))
        robots, objects = res if res else ([], [])
        if res and ser and not estop and now - last_send >= 1 / cfg["send_hz"]:
            # Sending nothing while e-stopped means robots also time out on their own.
            seq += 1
            ser.write(proto.frame(proto.pack_vision(seq, docks, (cfg["arena_w"], cfg["arena_h"]), robots, objects)))
            last_send = now
        if ser:
            for payload in deframer.feed(ser.read(4096)):
                msg = proto.parse(payload)
                if msg and msg["type"] == "snapshot":
                    snap = msg
                elif msg and msg["type"] == "heartbeat":
                    hbs[msg["id"]] = msg
        cv2.imshow("swarm satellite", draw(frame, vis, robots, objects, snap, hbs, estop, cfg))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            estop = not estop
            if ser:
                ser.write(proto.frame(proto.pack_estop(estop)))
            print("EMERGENCY STOP" if estop else "released")
    if ser:
        ser.write(proto.frame(proto.pack_estop(True)))   # leave the robots parked


if __name__ == "__main__":
    main()
