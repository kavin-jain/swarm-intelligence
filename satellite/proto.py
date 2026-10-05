"""Wire format, mirrored from core/proto.h. Little-endian, field by field."""
import math
import struct

MSG_VISION, MSG_SNAPSHOT, MSG_HEARTBEAT, MSG_ESTOP = 1, 2, 3, 4
NONE = 0xFF
STATES = ["idle", "goto", "align", "wait", "push", "backoff", "stopped"]
STATUS = ["open", "delivered", "stuck"]


def _i16(v):
    """Round half away from zero and clamp, exactly like clamp16() in proto.h."""
    v = math.floor(v + 0.5) if v >= 0 else math.ceil(v - 0.5)
    return max(-32768, min(32767, int(v)))


def pack_vision(seq, zone, arena, robots, objects):
    """zone=(x, y, r) mm; arena=(w, h) mm; robots=[(id, x, y, theta_rad)]; objects=[(id, x, y)]."""
    b = struct.pack("<BHhhHHHBB", MSG_VISION, seq & 0xFFFF, _i16(zone[0]), _i16(zone[1]), int(zone[2]),
                    int(arena[0]), int(arena[1]), len(robots), len(objects))
    for rid, x, y, th in robots:
        b += struct.pack("<Bhhh", rid, _i16(x), _i16(y), _i16(th * 1000))
    for oid, x, y in objects:
        b += struct.pack("<Bhh", oid, _i16(x), _i16(y))
    return b


def pack_estop(on):
    return bytes([MSG_ESTOP, 1 if on else 0])


def crc8(data):
    c = 0
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def frame(payload):
    return bytes([0xA5, 0x5A, len(payload)]) + payload + bytes([crc8(payload)])


class Deframer:
    """Feed it serial bytes; it yields complete, CRC-checked payloads."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xA5\x5A")
            if i < 0:
                del self.buf[:-1]
                return out
            del self.buf[:i]
            if len(self.buf) < 3 or len(self.buf) < 4 + self.buf[2]:
                return out
            n = self.buf[2]
            payload, crc = bytes(self.buf[3:3 + n]), self.buf[3 + n]
            if n and crc8(payload) == crc:
                out.append(payload)
                del self.buf[:4 + n]
            else:
                del self.buf[:2]   # bad frame: resync on the next header


def parse(payload):
    """Decode a gateway->laptop payload into a dict (snapshot or heartbeat), or None."""
    t = payload[0]
    if t == MSG_HEARTBEAT and len(payload) == 10:
        _, rid, seen, state, task, help_, nb, mv = struct.unpack("<BBHBBBBH", payload)
        return {"type": "heartbeat", "id": rid, "seen": seen, "state": state, "task": task, "help": help_,
                "neighbors": nb, "batt_mv": mv}
    if t == MSG_SNAPSHOT and len(payload) >= 15:
        _, seq, zx, zy, zr, aw, ah, nr, no = struct.unpack_from("<BHhhHHHBB", payload)
        if len(payload) != 15 + nr * 10 + no * 7:
            return None
        robots, objects, off = [], [], 15
        for _ in range(nr):
            rid, x, y, th, alive, task, state = struct.unpack_from("<BhhhBBB", payload, off)
            robots.append({"id": rid, "x": x, "y": y, "th": th / 1000, "alive": bool(alive), "task": task, "state": state})
            off += 10
        for _ in range(no):
            oid, x, y, demand, status = struct.unpack_from("<BhhBB", payload, off)
            objects.append({"id": oid, "x": x, "y": y, "demand": demand, "status": status})
            off += 7
        return {"type": "snapshot", "seq": seq, "zone": (zx, zy, zr), "arena": (aw, ah), "robots": robots, "objects": objects}
    return None
