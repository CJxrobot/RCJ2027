# ============================================================
#  CAMERA MAIN (omni mirror) -> Teensy Act, /field_data v2
#  Uses the SAME image frame, centre, ring, angle maths and exposure as color_picker
#  (tune.py), so a LAB range that works in the picker works here unchanged.
# ============================================================
from maix import camera, image, app, time, uart, err, pinmap
import math
import threading
import time as systime   # standard time (maix.time shadows the name "time")

# ================= SETTINGS (keep the same as the color picker) =================
CENTER_X = 170
CENTER_Y = 125
R_MIN = 35
R_MAX = 104

YELLOW_LAB = [30, 77, -16, 28, 29, 72]
BLUE_LAB = [30, 53, -24, 23, -47, -10]
ORANGE_LAB = [44, 75, 6, 47, 10, 53]

ATTACK_YELLOW = True          # front goal = yellow (the one we attack), back goal = blue

# ---- exposure (use the SAME values in main.py, color_picker.py, calibrate.py) ----
# 0 = lock whatever auto exposure picked at start-up. For the same colours every run,
# copy the "EXPOSURE LOCKED -> ..." numbers the picker prints into these two.
AUTO_EXPOSURE = False
EXPOSURE_US = 0
GAIN = 0

# ---- display only: does NOT change any angle or position ----
DEBUG_VIEW = True
DISPLAY_FLIP_Y = True         # flip the preview up-down so it looks the right way round

# ================= PROTOCOL: /field_data v2 (same as core_act.h) =================
#   CC DE flags ballA ballD frontA frontD backA backD seq chk EE   (18 bytes, uint16 LE)
#   flags: bit0 ball, bit1 front goal, bit2 back goal. 0xFFFF = not found.
#   chk = sum of bytes [1..15] & 0xFF
#   Angles: robot frame, 0 = right, 90 = front, 180 = left, 270 = back
#   Distances: ball = pixel radius from the image centre, goals = cm (formula A, B, C)
#   (same maths as the picker: the back goal reads 270, as act_defense was tested with)
UART_BAUD = 115200
TX_HZ = 200                   # 18 bytes at 115200 baud allow ~640 Hz
PKT_LEN = 18
NOT_FOUND = 0xFFFF
FLAG_BALL, FLAG_FRONT, FLAG_BACK = 0x01, 0x02, 0x04

GOAL_UPDATE_EVERY = 10        # goals searched every N frames, ball every frame

# ---- ball ----
BALL_MIN_PIXELS = 4           # same as the picker
BALL_JOIN_PX = 6              # pieces this close to the biggest piece are the same ball
                              # (a highlight on the ball can split it into pieces)
# Hand filter, applied to the joined ball. The largest allowed area goes from
# BALL_MAX_PIXELS_NEAR (at R_MIN) to BALL_MAX_PIXELS_FAR (at R_MAX). Look at "px" in the
# terminal with the ball near and far, set these ~1.5x that. 0 = filter off.
BALL_MAX_PIXELS_NEAR = 0
BALL_MAX_PIXELS_FAR = 0
BALL_MAX_ASPECT = 2.5         # w/h of the joined ball box, 0 = off

# Goal distance formula
A = 0.003379401783
B = 1671.194765
C = -23.10186756
HIST_N = 5

# Ball distance = raw pixel radius (the chase parameters were tuned on pixel radius)
BALL_A, BALL_B, BALL_C = 0.0, 1.0, 0.0

# ================= UART =================
for pin, func in {"A19": "UART1_TX", "A18": "UART1_RX"}.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")
ser = uart.UART("/dev/ttyS1", UART_BAUD)


def pack_uint16(v):
    v = int(v) & 0xFFFF
    return [v & 0xFF, v >> 8]


def build_packet_v2(ball, front, back, seq):
    flags = 0
    body = []
    for obj, flag in ((ball, FLAG_BALL), (front, FLAG_FRONT), (back, FLAG_BACK)):
        if obj is None:
            body += pack_uint16(NOT_FOUND) + pack_uint16(NOT_FOUND)
        else:
            flags |= flag
            body += pack_uint16(int(obj[0]) % 360) + pack_uint16(min(int(obj[1]), 0xFFFE))
    pkt = [0xCC, 0xDE, flags] + body + [seq & 0xFF]
    pkt.append(sum(pkt[1:16]) & 0xFF)
    pkt.append(0xEE)
    return bytes(pkt)


_lock = threading.Lock()
_latest = (None, None, None)
_tx_running = True
tx_count = 0


def set_data(ball, front, back):
    global _latest
    with _lock:
        _latest = (ball, front, back)


def tx_loop(hz):
    global tx_count
    period, seq, next_t = 1.0 / hz, 0, systime.monotonic()
    while _tx_running:
        with _lock:
            ball, front, back = _latest
        ser.write(build_packet_v2(ball, front, back, seq))
        seq = (seq + 1) & 0xFF
        tx_count += 1
        next_t += period
        d = next_t - systime.monotonic()
        if d > 0:
            systime.sleep(d)
        else:
            next_t = systime.monotonic()


threading.Thread(target=tx_loop, args=(TX_HZ,), daemon=True).start()

# ================= CAMERA =================
cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
cam.set_awb(1)


def setup_exposure(cam):
    if AUTO_EXPOSURE:
        return
    for _ in range(30):          # let auto exposure settle first
        cam.read()
    exp = EXPOSURE_US if EXPOSURE_US > 0 else cam.exposure()
    g = GAIN if GAIN > 0 else cam.gain()
    cam.exposure(int(exp))       # setting exposure/gain switches to manual mode
    cam.gain(int(g))
    print("EXPOSURE LOCKED -> EXPOSURE_US = %d   GAIN = %d" % (exp, g))


setup_exposure(cam)
if DEBUG_VIEW:
    from maix import display
    disp = display.Display()

ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)


# ================= GEOMETRY (same as the picker) =================
def radius(x, y):
    return math.hypot(x - CENTER_X, y - CENTER_Y)


def angle_of(x, y):
    return int(round(math.degrees(math.atan2(y - CENTER_Y, x - CENTER_X)))) % 360


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def in_ring(b):
    return R_MIN < radius(b.cx(), b.cy()) < R_MAX


# ================= GOALS (same grouping as the picker) =================
def pick_goal(blobs):
    parts = [(b, radius(b.cx(), b.cy()), math.atan2(b.cy() - CENTER_Y, b.cx() - CENTER_X))
             for b in blobs if b.pixels() >= 8 and in_ring(b)]
    if not parts:
        return None
    main = max(parts, key=lambda p: p[0].pixels())
    group = [p[0] for p in parts
             if abs(p[1] - main[1]) < 18 and angle_diff(p[2], main[2]) < math.radians(45)]
    x1 = min(b.x() for b in group)
    y1 = min(b.y() for b in group)
    x2 = max(b.x() + b.w() for b in group)
    y2 = max(b.y() + b.h() for b in group)
    px = sum(b.pixels() for b in group)
    cx = (x1 + x2) // 2
    cy = sum(b.cy() * b.pixels() for b in group) // px
    return {"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1, "cx": cx, "cy": cy, "px": px}


class Goal:
    def __init__(self, lab):
        self.lab, self.hist, self.box, self.result = lab, [], None, None

    def update(self, img):
        self.box = pick_goal(img.find_blobs([self.lab], roi=ROI, merge=False,
                                            area_threshold=8, pixels_threshold=8))
        if not self.box:
            self.result = None
            self.hist.clear()
            return
        self.hist.append(radius(self.box["cx"], self.box["cy"]))
        if len(self.hist) > HIST_N:
            self.hist.pop(0)
        r = sum(self.hist) / len(self.hist)
        dist = max(A * r * r + B / math.sqrt(self.box["px"]) + C, 0.0)
        self.result = (angle_of(self.box["cx"], self.box["cy"]), int(round(dist)))


# ================= BALL =================
ball_rejected = False   # last frame: a ball-coloured thing was found but filtered out (hand?)


def ball_max_px(r):
    if BALL_MAX_PIXELS_NEAR <= 0 or BALL_MAX_PIXELS_FAR <= 0:
        return 1e9
    t = min(max((r - R_MIN) / float(R_MAX - R_MIN), 0.0), 1.0)
    return BALL_MAX_PIXELS_NEAR + (BALL_MAX_PIXELS_FAR - BALL_MAX_PIXELS_NEAR) * t


def pick_ball(blobs):
    """Biggest ball-coloured piece in the ring (as the picker), joined with the pieces
    touching it, then the hand filter. Returns a box dict or None."""
    global ball_rejected
    ball_rejected = False
    parts = [b for b in blobs if b.pixels() >= BALL_MIN_PIXELS and in_ring(b)]
    if not parts:
        return None
    main = max(parts, key=lambda b: b.pixels())
    mx1, my1 = main.x() - BALL_JOIN_PX, main.y() - BALL_JOIN_PX
    mx2, my2 = main.x() + main.w() + BALL_JOIN_PX, main.y() + main.h() + BALL_JOIN_PX
    group = [b for b in parts
             if b.x() < mx2 and b.x() + b.w() > mx1 and b.y() < my2 and b.y() + b.h() > my1]
    x1 = min(b.x() for b in group)
    y1 = min(b.y() for b in group)
    x2 = max(b.x() + b.w() for b in group)
    y2 = max(b.y() + b.h() for b in group)
    px = sum(b.pixels() for b in group)
    cx = sum(b.cx() * b.pixels() for b in group) / px
    cy = sum(b.cy() * b.pixels() for b in group) / px
    w, h = max(x2 - x1, 1), max(y2 - y1, 1)
    box = {"x": x1, "y": y1, "w": w, "h": h, "cx": cx, "cy": cy, "px": px, "r": radius(cx, cy)}

    if px > ball_max_px(box["r"]) or (BALL_MAX_ASPECT > 0 and max(w / h, h / w) > BALL_MAX_ASPECT):
        ball_rejected = True
        box["rejected"] = True
    return box


# ================= MAIN LOOP =================
front_goal = Goal(YELLOW_LAB if ATTACK_YELLOW else BLUE_LAB)
back_goal = Goal(BLUE_LAB if ATTACK_YELLOW else YELLOW_LAB)
frame = 0
last_print = time.ticks_ms()
last_tx = 0

WHITE = image.Color.from_rgb(255, 255, 255)
GREEN = image.Color.from_rgb(0, 255, 0)
RED = image.Color.from_rgb(255, 0, 0)
ORANGE = image.Color.from_rgb(255, 128, 0)
YELLOW = image.Color.from_rgb(255, 255, 0)
BLUE = image.Color.from_rgb(0, 128, 255)

try:
    while not app.need_exit():
        img = cam.read()
        if img is None:
            continue
        img = img.flip(image.FlipDir.X)          # same frame as the picker
        img = img.flip(image.FlipDir.Y)
        frame += 1

        box = pick_ball(img.find_blobs([ORANGE_LAB], roi=ROI, merge=False,
                                       area_threshold=BALL_MIN_PIXELS, pixels_threshold=BALL_MIN_PIXELS))
        ball = None
        if box and not box.get("rejected"):
            r = box["r"]
            ball = (angle_of(box["cx"], box["cy"]), int(round(max(BALL_A * r * r + BALL_B * r + BALL_C, 0))))

        if frame % GOAL_UPDATE_EVERY == 0:
            front_goal.update(img)
            back_goal.update(img)

        set_data(ball, front_goal.result, back_goal.result)

        if DEBUG_VIEW:
            img.draw_circle(CENTER_X, CENTER_Y, R_MIN, GREEN, 1)
            img.draw_circle(CENTER_X, CENTER_Y, R_MAX, RED, 1)
            img.draw_cross(CENTER_X, CENTER_Y, WHITE, 8, 1)
            for g, col in ((front_goal, YELLOW), (back_goal, BLUE)):
                if g.box:
                    img.draw_rect(g.box["x"], g.box["y"], g.box["w"], g.box["h"], col, 2)
                    img.draw_line(CENTER_X, CENTER_Y, g.box["cx"], g.box["cy"], col, 2)
            if box:
                col = RED if box.get("rejected") else ORANGE     # red box = filtered out (hand?)
                img.draw_rect(box["x"] - 2, box["y"] - 2, box["w"] + 4, box["h"] + 4, col, 2)
                if not box.get("rejected"):
                    img.draw_line(CENTER_X, CENTER_Y, int(box["cx"]), int(box["cy"]), WHITE, 1)
            disp.show(img.flip(image.FlipDir.Y) if DISPLAY_FLIP_Y else img)

        now = time.ticks_ms()
        if now - last_print >= 200:
            hz = (tx_count - last_tx) * 1000.0 / (now - last_print)
            last_tx, last_print = tx_count, now
            print("ball:", ball,
                  "px", box["px"] if box else "-", "r", round(box["r"], 1) if box else "-",
                  "REJECTED" if ball_rejected else "",
                  "| front:", front_goal.result, "| back:", back_goal.result,
                  "| fps:", round(time.fps(), 1), "| tx Hz:", round(hz))
finally:
    _tx_running = False
