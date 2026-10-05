"""Wire format, mirrored from core/proto.h. Little-endian, field by field."""
import math
import struct

MSG_VISION, MSG_SNAPSHOT, MSG_HEARTBEAT, MSG_ESTOP = 1, 2, 3, 4
NONE = 0xFF
MAX_ROBOTS, MAX_ZONES, MAX_OBJECTS = 10, 3, 16
STATES = ["idle", "goto", "align", "wait", "push", "backoff", "stopped"]
STATUS = ["open", "delivered", "stuck"]


def _i16(v):
    """Round half away from zero and clamp, exactly like clamp16() in proto.h."""
    v = math.floor(v + 0.5) if v >= 0 else math.ceil(v - 0.5)
    return max(-32768, min(32767, int(v)))


def pack_vision(seq, docks, arena, robots, objects):
    """docks=[(x, y, r)] mm, a load of kind k goes to docks[k % len]; arena=(w, h) mm;
    robots=[(id, x, y, theta_rad)]; objects=[(id, x, y, kind)]."""
    docks, robots, objects = docks[:MAX_ZONES], robots[:MAX_ROBOTS], objects[:MAX_OBJECTS]
    b = struct.pack("<BHB", MSG_VISION, seq & 0xFFFF, len(docks))
    for x, y, r in docks:
        b += struct.pack("<hhH", _i16(x), _i16(y), int(r))
    b += struct.pack("<HHBB", int(arena[0]), int(arena[1]), len(robots), len(objects))
    for rid, x, y, th in robots:
        b += struct.pack("<Bhhh", rid, _i16(x), _i16(y), _i16(th * 1000))
    for oid, x, y, kind in objects:
        b += struct.pack("<BhhB", oid, _i16(x), _i16(y), kind)
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
    if t == MSG_SNAPSHOT and len(payload) >= 4:
        _, seq, nz = struct.unpack_from("<BHB", payload)
        off = 4 + nz * 6
        if nz > MAX_ZONES or len(payload) < off + 6:
            return None
        docks = [struct.unpack_from("<hhH", payload, 4 + i * 6) for i in range(nz)]
        aw, ah, nr, no = struct.unpack_from("<HHBB", payload, off)
        off += 6
        if len(payload) != off + nr * 10 + no * 7:
            return None
        robots, objects = [], []
        for _ in range(nr):
            rid, x, y, th, alive, task, state = struct.unpack_from("<BhhhBBB", payload, off)
            robots.append({"id": rid, "x": x, "y": y, "th": th / 1000, "alive": bool(alive), "task": task, "state": state})
            off += 10
        for _ in range(no):
            oid, x, y, dk, status = struct.unpack_from("<BhhBB", payload, off)   # demand and kind share a byte
            objects.append({"id": oid, "x": x, "y": y, "demand": dk & 0x0F, "kind": dk >> 4, "status": status})
            off += 7
        return {"type": "snapshot", "seq": seq, "docks": docks, "arena": (aw, ah), "robots": robots, "objects": objects}
    return None
