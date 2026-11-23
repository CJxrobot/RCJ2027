# /field_data v2 packet builder for the Maix Cam (copy into your camera code, task R01)
# Layout (18 bytes, same as the "Packets" tab and lib/protocol/protocol.h):
#   CC DE flags ballA ballD frontA frontD backA backD seq chk EE
#   angles 0-359 deg in the ROBOT frame (0 = right, 90 = front, CCW), distances in cm
#   uint16 little endian, 0xFFFF = not seen, chk = sum of bytes 1..15 & 0xFF

NONE = 0xFFFF
FLAG_BALL, FLAG_FRONT, FLAG_BACK = 0x01, 0x02, 0x04


def _u16(v):
    v = int(v) & 0xFFFF
    return [v & 0xFF, v >> 8]


def build_packet_v2(ball, front, back, seq):
    """ball / front / back: (angle_deg, dist_cm) tuple, or None when not seen."""
    flags = 0
    body = []
    for obj, flag in ((ball, FLAG_BALL), (front, FLAG_FRONT), (back, FLAG_BACK)):
        if obj is None:
            body += _u16(NONE) + _u16(NONE)
        else:
            flags |= flag
            angle, dist = obj
            body += _u16(int(round(angle)) % 360) + _u16(min(int(round(dist)), 0xFFFE))
    pkt = [0xCC, 0xDE, flags] + body + [seq & 0xFF]
    pkt.append(sum(pkt[1:16]) & 0xFF)
    pkt.append(0xEE)
    return bytes(pkt)
