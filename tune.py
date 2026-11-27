# ============================================================
#  COLOR PICKER (omni mirror) — goal + ball — run from MaixVision
# ============================================================
#
#  1. Choose what to pick:
TARGET = "ball"        # "yellow", "blue" or "ball"
#
#  2. Put it on the WHITE SQUARE(S) in the preview:
#       goal -> 5 squares in a curve behind the robot
#       ball -> 1 square on the right
#  3. Read the terminal:   USE THIS: [L1, L2, A1, A2, B1, B2]
#     Copy into goal_ball_detection.py as YELLOW_LAB / BLUE_LAB / ORANGE_LAB.
#
#  BALL: 1 square. Put the ball on it.
#        Change BALL_SPOT below to move it (0 = R_MIN, 1 = R_MAX).
#
#  OPTIONAL: join results from several runs (e.g. different lighting).
#     Paste each USE THIS into MERGE, the script joins and TESTS them.
MERGE = [
    #[34, 41, 26, 35, 26, 35]
]
#
#  VERIFY: paste numbers here to test them (no measuring):
#     TEST_LAB = [53, 79, -11, 14, 41, 71]
TEST_LAB = [44, 75, 6, 47, 10, 53]
#
#  Preview:
#     purple box       = everything the numbers match inside the ring
#     thick yellow box = the goal main.py would pick
#     thick orange box = the ball main.py would pick
#  Press Run again to start fresh.
# ============================================================

# square settings per target  (angle in degrees, same angle as main.py prints)
SETTINGS = {
    "yellow": dict(angle=90, r=75, span=30, n=5, size=6),
    "blue":   dict(angle=90, r=75, span=30, n=5, size=6),
    "ball":   dict(angle=0,  r=60, span=0,  n=1, size=4),   # position set by BALL_SPOT
}
# ball square: position across the ring (0 = R_MIN, 1 = R_MAX), square size
BALL_SPOT = (0.5, 5)
MARGIN = 5            # extra room added to each side of the result

# ============================================================

from maix import camera, image, display, app, time
import math

# same as goal_ball_detection.py (measured there with the Y flip only)
CENTER_X = 175
CENTER_Y = 116
R_MIN = 35
R_MAX = 100

IS_BALL = TARGET == "ball"
S = SETTINGS[TARGET]
def square_at(angle, r, size):
    a = math.radians(angle)
    cx = int(CENTER_X + r * math.cos(a))
    cy = int(CENTER_Y + r * math.sin(a))
    return (cx - size // 2, cy - size // 2, size, size)



# join the MERGE results into one range and test it
MERGE = [m for m in MERGE if m and len(m) == 6]
if MERGE:
    TEST_LAB = [min(m[0] for m in MERGE), max(m[1] for m in MERGE),
                min(m[2] for m in MERGE), max(m[3] for m in MERGE),
                min(m[4] for m in MERGE), max(m[5] for m in MERGE)]
    print("=" * 50)
    print("MERGED %d results -> %s" % (len(MERGE), TEST_LAB))
    print("put this in main.py if the test below looks good")
    print("=" * 50)

# build the squares along the arc
SQUARES = []
if IS_BALL:
    f, size = BALL_SPOT
    SQUARES.append(square_at(S["angle"], R_MIN + (R_MAX - R_MIN) * f, size))
    print("ball square at r =", round(R_MIN + (R_MAX - R_MIN) * f))
for i in range(0 if IS_BALL else S["n"]):
    deg = S["angle"] if S["n"] == 1 else S["angle"] - S["span"] / 2 + S["span"] * i / (S["n"] - 1)
    SQUARES.append(square_at(deg, S["r"], S["size"]))

cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
cam.set_awb(1)


# ---- camera image (use the SAME values in goal_ball_detection.py and tune.py) ----
# Dark blue goal hard to see: raise LUMA first (55-70), then SATURATION (55-70).
# Still too dark: EXPOSURE_SCALE 1.2-1.5 (longer exposure, more motion blur).
# After ANY change here: re-pick ALL THREE colours with tune.py (the LAB numbers shift).
LUMA = 60              # brightness 0-100 (camera default 50), -1 = leave as is
CONTRAST = -1          # 0-100 (camera default 50), -1 = leave as is
SATURATION = 60        # 0-100 (camera default 50): goals / ball colours stand out more
AUTO_EXPOSURE = False  # False = fixed exposure, colours stay stable during the match
EXPOSURE_US = 0        # 0 = lock what auto exposure picked at start-up (x EXPOSURE_SCALE)
EXPOSURE_SCALE = 1.0   # > 1 = brighter (only when EXPOSURE_US = 0)
GAIN = 0               # 0 = lock what auto exposure picked at start-up


def setup_camera(cam):
    # MaixPy spells contrast "constrast"
    for name, val in (("luma", LUMA), ("constrast", CONTRAST), ("saturation", SATURATION)):
        if val >= 0:
            try:
                getattr(cam, name)(int(val))
            except Exception as e:
                print("camera %s not set: %s" % (name, e))
    if not AUTO_EXPOSURE:
        for _ in range(30):          # let auto exposure settle first
            cam.read()
        exp = EXPOSURE_US if EXPOSURE_US > 0 else cam.exposure() * EXPOSURE_SCALE
        g = GAIN if GAIN > 0 else cam.gain()
        cam.exposure(int(exp))       # setting exposure / gain switches to manual mode
        cam.gain(int(g))
    try:
        print("CAMERA: luma %d contrast %d saturation %d | exposure %s %d us gain %d" % (
            cam.luma(), cam.constrast(), cam.saturation(),
            "AUTO" if AUTO_EXPOSURE else "LOCKED", cam.exposure(), cam.gain()))
    except Exception as e:
        print("CAMERA settings:", e)

setup_camera(cam)
disp = display.Display()

WHITE = image.Color.from_rgb(255, 255, 255)
GREEN = image.Color.from_rgb(0, 255, 0)
RED = image.Color.from_rgb(255, 0, 0)
PURPLE = image.Color.from_rgb(255, 0, 255)
GREY = image.Color.from_rgb(120, 120, 120)
PICK = image.Color.from_rgb(255, 128, 0) if IS_BALL else image.Color.from_rgb(255, 255, 0)

ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)


# ---------- measuring ----------
def count_pixels(img, th):
    n = 0
    for sq in SQUARES:
        for b in img.find_blobs([th], roi=sq, merge=False,
                                area_threshold=1, pixels_threshold=1):
            n += b.pixels()
    return n


def histogram(img, lo, hi, step, make_th):
    hist = []
    v = lo
    while v <= hi:
        top = min(v + step - 1, hi)
        hist.append((v, top, count_pixels(img, make_th(v, top))))
        v += step
    return hist


def channel_range(img, lo, hi, make_th):
    coarse = [h for h in histogram(img, lo, hi, 16, make_th) if h[2] > 0]
    if not coarse:
        return None
    fine = histogram(img, coarse[0][0], coarse[-1][1], 2, make_th)
    total = sum(h[2] for h in fine)
    if total == 0:
        return None
    cut = total * 0.05          # drop the outer 5 % on each side
    acc, low = 0, fine[0][0]
    for v, top, c in fine:
        acc += c
        if acc > cut:
            low = v
            break
    acc, high = 0, fine[-1][1]
    for v, top, c in reversed(fine):
        acc += c
        if acc > cut:
            high = top
            break
    return low, high


def measure(img):
    L = channel_range(img, 0, 100, lambda a, b: [a, b, -128, 127, -128, 127])
    A = channel_range(img, -128, 127, lambda a, b: [0, 100, a, b, -128, 127])
    B = channel_range(img, -128, 127, lambda a, b: [0, 100, -128, 127, a, b])
    if L is None or A is None or B is None:
        return None
    return [L[0], L[1], A[0], A[1], B[0], B[1]]


# ---------- picking (same rules as main.py) ----------
def radius(x, y):
    return math.hypot(x - CENTER_X, y - CENTER_Y)


def angle_of(x, y):
    return int(math.degrees(math.atan2(y - CENTER_Y, x - CENTER_X)) % 360)


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def pick(blobs):
    # returns (x, y, w, h, cx, cy, pixels) or None
    if not blobs:
        return None
    if IS_BALL:
        b = max(blobs, key=lambda b: b.pixels())
        return b.x(), b.y(), b.w(), b.h(), b.cx(), b.cy(), b.pixels()
    parts = [(b, radius(b.cx(), b.cy()), math.atan2(b.cy() - CENTER_Y, b.cx() - CENTER_X))
             for b in blobs if b.pixels() >= 8]
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
    return x1, y1, x2 - x1, y2 - y1, cx, cy, px


# ---------- main loop ----------
best = None
last_print = time.ticks_ms()
print("TARGET:", TARGET, "| mode:", "TEST " + str(TEST_LAB) if TEST_LAB else "MEASURE")

while not app.need_exit():
    img = cam.read()
    if img is None:
        continue
    img = img.flip(image.FlipDir.Y)   # up-down flip only, same as goal_ball_detection.py

    now_th = None if TEST_LAB else measure(img)
    if now_th:
        if best is None:
            best = now_th[:]
        else:
            best = [min(best[0], now_th[0]), max(best[1], now_th[1]),
                    min(best[2], now_th[2]), max(best[3], now_th[3]),
                    min(best[4], now_th[4]), max(best[5], now_th[5])]

    use = TEST_LAB
    if best and not TEST_LAB:
        use = [max(best[0] - MARGIN, 0), min(best[1] + MARGIN, 100),
               max(best[2] - MARGIN, -128), min(best[3] + MARGIN, 127),
               max(best[4] - MARGIN, -128), min(best[5] + MARGIN, 127)]

    t = None
    n_match = 0
    if use:
        blobs = img.find_blobs([use], roi=ROI, merge=False,
                               area_threshold=4, pixels_threshold=4)
        blobs = [b for b in blobs if R_MIN < radius(b.cx(), b.cy()) < R_MAX]
        n_match = len(blobs)
        for b in blobs:
            img.draw_rect(b.x(), b.y(), b.w(), b.h(), PURPLE, 1)
        t = pick(blobs)
        if t:
            img.draw_rect(t[0] - 2, t[1] - 2, t[2] + 4, t[3] + 4, PICK, 2)
            img.draw_line(CENTER_X, CENTER_Y, t[4], t[5], WHITE, 1)

    img.draw_circle(CENTER_X, CENTER_Y, R_MIN, GREEN, 1)
    img.draw_circle(CENTER_X, CENTER_Y, R_MAX, RED, 1)
    if not TEST_LAB:
        for x, y, w, h in SQUARES:
            img.draw_rect(x - 1, y - 1, w + 2, h + 2, WHITE, 1)
    disp.show(img)

    now = time.ticks_ms()
    if now - last_print >= 500:
        last_print = now
        if t:
            info = "FOUND angle=%d r=%.1f px=%d" % (angle_of(t[4], t[5]), radius(t[4], t[5]), t[6])
        else:
            info = "NOT FOUND"
        if TEST_LAB:
            print("TESTING", TEST_LAB, "->", info, "| matches:", n_match)
        else:
            print("in squares now:", now_th)
            print("USE THIS:", use, "->", info, "| matches:", n_match)