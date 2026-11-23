from maix import camera, image, app, time, uart, err, pinmap
import math
import threading
import time as systime   # standard time (maix.time shadows the name "time")

# ================= PROTOCOL: /field_data v2 (task T04) =================
# 18 bytes, little-endian uint16 fields. Same as core_act.h and the "封包" tab:
#
#   [0]  0xCC          header 1
#   [1]  0xDE          header 2
#   [2]  flags         bit0 = ball found, bit1 = front goal found, bit2 = back goal found
#   [3-4]   ball angle        0..359 deg   (0xFFFF if not found)
#   [5-6]   ball distance                  (0xFFFF if not found)
#   [7-8]   front goal angle  (the goal we attack)
#   [9-10]  front goal distance
#   [11-12] back goal angle   (our own goal)
#   [13-14] back goal distance
#   [15] seq           +1 every packet (the Teensy counts lost packets from gaps)
#   [16] checksum      sum of bytes [1..15] & 0xFF
#   [17] 0xEE          tail
#
# Angles: robot frame, 0 = right, 90 = front, 180 = left, 270 = back (counter-clockwise).
# With the picture the right way up: screen top = front, screen right = right.
PKT_LEN = 18
NOT_FOUND = 0xFFFF

FLAG_BALL = 0x01
FLAG_FRONT = 0x02
FLAG_BACK = 0x04

# ================= UART CONFIG =================
UART_BAUD = 115200
TX_HZ = 200          # packets per second (> 100). 18 bytes at 115200 baud allow ~640 Hz

pin_function = {
    "A19": "UART1_TX",
    "A18": "UART1_RX"
}
device = "/dev/ttyS1"

for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")

ser = uart.UART(device, UART_BAUD)

_max_hz = UART_BAUD / (PKT_LEN * 10.0)   # 8N1: 10 bits per byte
if TX_HZ > _max_hz:
    print(f"WARNING: TX_HZ={TX_HZ} exceeds UART capacity (~{int(_max_hz)} Hz at {UART_BAUD} baud)")

# ================= VISION CONFIG =================
DEBUG_VIEW = True
GOAL_UPDATE_EVERY = 10     # goals are searched every N frames
BALL_UPDATE_EVERY = 1      # ball is searched every N frames (1 = every frame)

cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
cam.set_awb(1)

if DEBUG_VIEW:
    from maix import display
    disp = display.Display()

YELLOW_LAB = [30, 77, -16, 28, 29, 72]
BLUE_LAB = [30, 53, -24, 23, -47, -10]

# The goal we attack is the FRONT goal, the other one is the BACK goal (ATTACK_YELLOW in core_act.h)
ATTACK_YELLOW = True
FRONT_LAB = YELLOW_LAB if ATTACK_YELLOW else BLUE_LAB
BACK_LAB = BLUE_LAB if ATTACK_YELLOW else YELLOW_LAB

# Orange ball (tune in the MaixVision threshold tool)
# keep A-min above the yellow goal's A-max (28) so the goal isn't read as the ball
ORANGE_LAB = [44, 75, 6, 47, 10, 53]
# Image centre (robot centre) AFTER the flip below.
# Only the Y (up-down) flip is applied now. The old values (175, 125) were measured with
# both flips, so X is mirrored: 319 - 175 = 144, Y stays 125.
# If the centre cross is not on the robot centre, re-measure both in the debug view.
CENTER_X = 144
CENTER_Y = 125
R_MIN = 37
R_MAX = 100

BALL_R_MIN = R_MIN
BALL_R_MAX = R_MAX

# ---- Ball filter: reject hands and other orange-ish things ----
# A real ball is small, round and solid. Its pixel area shrinks the farther it is from the
# centre, so the largest allowed area goes from BALL_MAX_PIXELS_NEAR (at R_MIN) down to
# BALL_MAX_PIXELS_FAR (at R_MAX). Look at the "ball px" debug print with the ball at both
# ends and set these a bit above what you see. A hand is much bigger and less solid.
BALL_MIN_PIXELS = 4
BALL_MAX_PIXELS_NEAR = 600   # ball right next to the robot (r = R_MIN)
BALL_MAX_PIXELS_FAR = 120    # ball far away (r = R_MAX)
BALL_MAX_ASPECT = 2.0        # w/h and h/w must be below this (a ball box is about square)
BALL_MIN_FILL = 0.45         # pixels / (w*h): a ball fills ~0.6-0.8 of its box, a hand less

# Goal distance formula
A = 0.003379401783
B = 1671.194765
C = -23.10186756

# Ball distance: dist = BALL_A*r^2 + BALL_B*r + BALL_C  (r = pixel radius from centre)
# defaults send the raw pixel radius (the chase parameters were tuned on pixel radius)
BALL_A = 0.0
BALL_B = 1.0
BALL_C = 0.0

HIST_N = 5

frame_count = 0
last_print = time.ticks_ms()


def point_radius(x, y):
    return math.hypot(x - CENTER_X, y - CENTER_Y)


def point_angle_deg(x, y):
    # Robot frame: screen top = front (90), screen right = right (0), counter-clockwise.
    # Image y grows downward, so use (CENTER_Y - y) to make "up" positive.
    return int(round(math.degrees(math.atan2(CENTER_Y - y, x - CENTER_X)) % 360)) % 360


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def pack_uint16(value):
    value = int(value) & 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def build_packet_v2(ball, front, back, seq):
    """ball / front / back: (angle_deg, dist) or None when not found."""
    flags = 0
    body = []
    for obj, flag in ((ball, FLAG_BALL), (front, FLAG_FRONT), (back, FLAG_BACK)):
        if obj is None:
            body += pack_uint16(NOT_FOUND) + pack_uint16(NOT_FOUND)
        else:
            flags |= flag
            angle, dist = obj
            body += pack_uint16(int(angle) % 360) + pack_uint16(min(int(dist), 0xFFFE))
    pkt = [0xCC, 0xDE, flags] + body + [seq & 0xFF]
    pkt.append(sum(pkt[1:16]) & 0xFF)
    pkt.append(0xEE)
    return bytes(pkt)


# ================= UART TX THREAD =================
# The vision loop only stores the newest results; the TX thread builds a fresh packet
# (with its own seq) for every send, so the Teensy sees TX_HZ packets per second.
_data_lock = threading.Lock()
_latest = (None, None, None)   # (ball, front, back)
_tx_running = True
tx_count = 0


def set_data(ball, front, back):
    global _latest
    with _data_lock:
        _latest = (ball, front, back)


def tx_loop(hz):
    global tx_count
    period = 1.0 / hz
    seq = 0
    next_t = systime.monotonic()
    while _tx_running:
        with _data_lock:
            ball, front, back = _latest
        ser.write(build_packet_v2(ball, front, back, seq))
        seq = (seq + 1) & 0xFF
        tx_count += 1

        next_t += period
        delay = next_t - systime.monotonic()
        if delay > 0:
            systime.sleep(delay)
        else:
            next_t = systime.monotonic()   # fell behind: resync instead of bursting


tx_thread = threading.Thread(target=tx_loop, args=(TX_HZ,), daemon=True)
tx_thread.start()


# ================= GOAL DETECTION =================
def merge_goal_parts(blobs):
    parts = []
    if not blobs:
        return None

    for b in blobs:
        r = point_radius(b.cx(), b.cy())
        if not (R_MIN <= r <= R_MAX):
            continue
        if b.pixels() < 8:
            continue
        parts.append({"blob": b, "r": r, "angle": math.atan2(b.cy() - CENTER_Y, b.cx() - CENTER_X)})

    if not parts:
        return None

    main = max(parts, key=lambda p: p["blob"].pixels())

    group = []
    for part in parts:
        if abs(part["r"] - main["r"]) < 18 and angle_diff(part["angle"], main["angle"]) < math.radians(45):
            group.append(part["blob"])
    if not group:
        group = [main["blob"]]

    x1 = min(b.x() for b in group)
    y1 = min(b.y() for b in group)
    x2 = max(b.x() + b.w() for b in group)
    y2 = max(b.y() + b.h() for b in group)
    total_pixels = sum(b.pixels() for b in group)

    # split by an obstacle: use the leftmost / rightmost parts for the centre
    cx = (x1 + x2) // 2
    cy = sum(b.cy() * b.pixels() for b in group) // total_pixels

    return {"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1,
            "cx": int(cx), "cy": int(cy), "pixels": total_pixels, "count": len(group)}


class GoalTracker:
    def __init__(self, lab, name):
        self.lab = lab
        self.name = name
        self.history = []
        self.best = None
        self.result = None   # (angle, dist) or None

    def update(self, img, roi):
        blobs = img.find_blobs([self.lab], roi=roi, merge=False, area_threshold=8, pixels_threshold=8)
        self.best = merge_goal_parts(blobs)
        if not self.best:
            self.result = None
            self.history.clear()
            return
        r = point_radius(self.best["cx"], self.best["cy"])
        self.history.append(r)
        if len(self.history) > HIST_N:
            self.history.pop(0)
        avg_r = sum(self.history) / len(self.history)
        dist = A * avg_r * avg_r + B / math.sqrt(self.best["pixels"]) + C
        dist = max(dist, 0.0)
        self.result = (point_angle_deg(self.best["cx"], self.best["cy"]), int(round(dist)))


# ================= BALL DETECTION =================
def ball_max_pixels(r):
    t = (r - BALL_R_MIN) / float(BALL_R_MAX - BALL_R_MIN)
    t = min(max(t, 0.0), 1.0)
    return BALL_MAX_PIXELS_NEAR + (BALL_MAX_PIXELS_FAR - BALL_MAX_PIXELS_NEAR) * t


reject_count = 0   # blobs rejected by the area / shape filter in the last frame


def find_ball(blobs):
    global reject_count
    reject_count = 0
    best = None
    for b in blobs or []:
        px = b.pixels()
        if px < BALL_MIN_PIXELS:
            continue
        r = point_radius(b.cx(), b.cy())
        if not (BALL_R_MIN <= r <= BALL_R_MAX):
            continue
        w, h = max(b.w(), 1), max(b.h(), 1)
        aspect = max(w / h, h / w)
        fill = px / float(w * h)
        if px > ball_max_pixels(r) or aspect > BALL_MAX_ASPECT or fill < BALL_MIN_FILL:
            reject_count += 1          # too big (hand), too long, or too hollow
            continue
        if best is None or px > best["pixels"]:
            best = {"x": b.x(), "y": b.y(), "w": b.w(), "h": b.h(),
                    "cx": b.cx(), "cy": b.cy(), "r": r, "pixels": px}
    return best


def calculate_ball(ball):
    r = ball["r"]
    dist = max(BALL_A * r * r + BALL_B * r + BALL_C, 0.0)
    return point_angle_deg(ball["cx"], ball["cy"]), int(round(dist))


# ================= MAIN LOOP =================
ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)
front_goal = GoalTracker(FRONT_LAB, "front")
back_goal = GoalTracker(BACK_LAB, "back")
best_ball = None
ball = None
last_tx_count = 0

try:
    while not app.need_exit():
        img = cam.read()
        if img is None:
            continue

        img = img.flip(image.FlipDir.Y)   # up-down flip only (no left-right flip)
        frame_count += 1

        # ---- ball ----
        if frame_count % BALL_UPDATE_EVERY == 0:
            ball_blobs = img.find_blobs([ORANGE_LAB], roi=ROI, merge=False,
                                        area_threshold=BALL_MIN_PIXELS, pixels_threshold=BALL_MIN_PIXELS)
            best_ball = find_ball(ball_blobs)
            ball = calculate_ball(best_ball) if best_ball else None

        # ---- goals ----
        if frame_count % GOAL_UPDATE_EVERY == 0:
            front_goal.update(img, ROI)
            back_goal.update(img, ROI)

        set_data(ball, front_goal.result, back_goal.result)

        if DEBUG_VIEW:
            img.draw_circle(CENTER_X, CENTER_Y, R_MIN, image.Color.from_rgb(0, 255, 0), 1)
            img.draw_circle(CENTER_X, CENTER_Y, R_MAX, image.Color.from_rgb(255, 0, 0), 1)
            img.draw_cross(CENTER_X, CENTER_Y, image.Color.from_rgb(255, 255, 255), 8, 1)

            for g, col in ((front_goal, (255, 255, 0)), (back_goal, (0, 128, 255))):
                if g.best:
                    c = image.Color.from_rgb(*col)
                    img.draw_rect(g.best["x"], g.best["y"], g.best["w"], g.best["h"], c, 2)
                    img.draw_line(CENTER_X, CENTER_Y, g.best["cx"], g.best["cy"], c, 2)

            if best_ball:
                rad = max(best_ball["w"], best_ball["h"]) // 2 + 2
                img.draw_circle(best_ball["cx"], best_ball["cy"], rad, image.Color.from_rgb(255, 128, 0), 2)
                img.draw_line(CENTER_X, CENTER_Y, best_ball["cx"], best_ball["cy"],
                              image.Color.from_rgb(255, 128, 0), 1)

            now = time.ticks_ms()
            if now - last_print >= 200:
                tx_hz = (tx_count - last_tx_count) * 1000.0 / (now - last_print)
                last_tx_count = tx_count
                last_print = now
                print("ball:", ball, "px", best_ball["pixels"] if best_ball else "-",
                      "rejected", reject_count,
                      "| front:", front_goal.result, "| back:", back_goal.result,
                      "| fps:", round(time.fps(), 1), "| tx Hz:", round(tx_hz))
            disp.show(img)
finally:
    _tx_running = False
    tx_thread.join(timeout=0.5)
