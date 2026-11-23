# T04 test sender: run on the Maix Cam instead of Cam.py.
# Sends fake /field_data v2 packets at 100 Hz on the same UART as Cam.py:
#   ball  : angle sweeps +3 deg per packet, distance 50 cm
#   front : 90 deg, 120 cm, seen / not seen every 2 s
#   back  : 270 deg, 80 cm, always seen
from maix import uart, pinmap, err, app
import time as systime

UART_BAUD = 115200
TX_HZ = 100

for pin, func in {"A19": "UART1_TX", "A18": "UART1_RX"}.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")
ser = uart.UART("/dev/ttyS1", UART_BAUD)

NONE = 0xFFFF


def _u16(v):
    v = int(v) & 0xFFFF
    return [v & 0xFF, v >> 8]


def build_packet_v2(ball, front, back, seq):
    flags = 0
    body = []
    for obj, flag in ((ball, 0x01), (front, 0x02), (back, 0x04)):
        if obj is None:
            body += _u16(NONE) + _u16(NONE)
        else:
            flags |= flag
            body += _u16(int(obj[0]) % 360) + _u16(int(obj[1]))
    pkt = [0xCC, 0xDE, flags] + body + [seq & 0xFF]
    pkt.append(sum(pkt[1:16]) & 0xFF)
    pkt.append(0xEE)
    return bytes(pkt)


seq = 0
angle = 0
t0 = systime.monotonic()
next_t = t0
period = 1.0 / TX_HZ
while not app.need_exit():
    front_seen = int((systime.monotonic() - t0) / 2) % 2 == 0
    ser.write(build_packet_v2((angle, 50), (90, 120) if front_seen else None, (270, 80), seq))
    seq = (seq + 1) & 0xFF
    angle = (angle + 3) % 360
    if seq == 0:
        print("sent 256 packets, ball angle", angle)
    next_t += period
    d = next_t - systime.monotonic()
    if d > 0:
        systime.sleep(d)
    else:
        next_t = systime.monotonic()
