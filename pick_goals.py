# ============================================================
#  PICK GOAL COLOURS ON THE FIELD (omni mirror) -- run from MaixVision
# ============================================================
#  Step by step, the screen tells what to do (no buttons, a countdown for each step):
#    0. brightness: the camera sets exposure / gain for this room (~5 s, keep still)
#    1. BLUE:   turn the robot so the BLUE goal covers the white squares   (countdown, then measured)
#    2. YELLOW: turn the robot so the YELLOW goal covers the white squares (countdown, then measured)
#    3. BALL (optional): put the ball on the small square; no ball -> old value kept
#  Then it is saved to CONFIG_FILE and the screen shows what the new values find
#  (cyan = blue, purple = yellow, orange = ball). goal_ball_detection_auto.py loads that file
#  at start-up. goal_ball_detection.py and tune.py are not changed.
#  A step where the squares do not show that colour (e.g. the goal was not there) keeps the
#  old value and says so in the terminal.
# ============================================================

CONFIG_FILE = "/root/camera_auto.json"
WAIT_S = 8            # seconds to turn the robot / put the ball before each measurement
MEASURE_S = 2         # seconds measured (robot still)
TARGET_L = 45         # wanted brightness of the ring (0-100); dark room: more gain
SATURATION = 60       # 0-100 (camera default 50)
EXPOSURE_MAX_US = 30000
GAIN_MAX_X = 8        # gain at most this x what auto exposure picked
MARGIN = 5            # added to each side of a measured range

# Same image geometry as goal_ball_detection.py / tune.py (Y flip only)
CENTER_X = 175
CENTER_Y = 116
R_MIN = 35
R_MAX = 100
GOAL_SQ = dict(angle=90, r=75, span=30, n=5, size=6)    # same squares as tune.py
BALL_SQ = dict(angle=0, r=(R_MIN + R_MAX) / 2, size=5)

OLD = {   # kept when a step finds nothing (same as goal_ball_detection.py)
    "blue":   [30, 53, -24, 23, -47, -10],
    "yellow": [30, 77, -16, 28, 29, 72],
    "orange": [44, 75, 6, 47, 10, 53],
}

from maix import camera, image, display, app, time
import math, json

cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
disp = display.Display()
ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)
WHITE = image.Color.from_rgb(255, 255, 255)
COL = {"blue": image.Color.from_rgb(0, 255, 255), "yellow": image.Color.from_rgb(255, 0, 255),
       "orange": image.Color.from_rgb(255, 128, 0)}


def square_at(angle, r, size):
    a = math.radians(angle)
    return (int(CENTER_X + r * math.cos(a)) - size // 2, int(CENTER_Y + r * math.sin(a)) - size // 2, size, size)


GOAL_SQUARES = [square_at(GOAL_SQ["angle"] - GOAL_SQ["span"] / 2 + GOAL_SQ["span"] * i / (GOAL_SQ["n"] - 1),
                          GOAL_SQ["r"], GOAL_SQ["size"]) for i in range(GOAL_SQ["n"])]
BALL_SQUARES = [square_at(BALL_SQ["angle"], BALL_SQ["r"], BALL_SQ["size"])]


def read():
    img = cam.read()
    return img.flip(image.FlipDir.Y) if img is not None else None


def show(img, squares, text):
    for x, y, w, h in squares:
        img.draw_rect(x - 1, y - 1, w + 2, h + 2, WHITE, 1)
    try:
        img.draw_string(4, 4, text, WHITE, 1.5)
    except Exception:
        pass
    disp.show(img)


def pixels(img, th, rois):
    n = 0
    for r in rois:
        for b in img.find_blobs([th], roi=r, merge=False, area_threshold=1, pixels_threshold=1):
            n += b.pixels()
    return n


# range of one channel in the squares, 5 % cut on each side (same method as tune.py)
def channel_range(img, ch, rois):
    lo_lim, hi_lim = (0, 100) if ch == 0 else (-128, 127)
    hist = []
    v = lo_lim
    while v <= hi_lim:
        th = [0, 100, -128, 127, -128, 127]
        th[2 * ch], th[2 * ch + 1] = v, min(v + 1, hi_lim)
        hist.append((v, min(v + 1, hi_lim), pixels(img, th, rois)))
        v += 2
    total = sum(h[2] for h in hist)
    if total == 0:
        return None
    cut, acc, low, high = total * 0.05, 0, hist[0][0], hist[-1][1]
    for a, b, c in hist:
        acc += c
        if acc > cut:
            low = a
            break
    acc = 0
    for a, b, c in reversed(hist):
        acc += c
        if acc > cut:
            high = b
            break
    return low, high


def measure(img, rois):
    r = [channel_range(img, ch, rois) for ch in range(3)]
    return None if None in r else [r[0][0], r[0][1], r[1][0], r[1][1], r[2][0], r[2][1]]


def ring_median(img, ch):          # median of L (ch 0) / A / B of the ring = the field
    step, lo, hi = (5, 5, 100) if ch == 0 else (4, -60, 60)
    hist = []
    for v in range(lo, hi, step):
        th = [5, 100, -128, 127, -128, 127]
        th[2 * ch], th[2 * ch + 1] = v, v + step - 1
        hist.append((v, pixels(img, th, [ROI])))
    total = sum(c for _, c in hist)
    acc = 0
    for v, c in hist:
        acc += c
        if total and acc >= total / 2:
            return v + step // 2
    return 0


def countdown(seconds, squares, text):
    t0 = time.ticks_ms()
    while time.ticks_ms() - t0 < seconds * 1000 and not app.need_exit():
        img = read()
        if img is not None:
            left = seconds - (time.ticks_ms() - t0) // 1000
            show(img, squares, "%s  %d" % (text, left))


# ---------- 0. brightness ----------
cam.set_awb(1)                     # white balance stays auto (as goal_ball_detection.py)
try:
    cam.saturation(SATURATION)
except Exception as e:
    print("saturation not set:", e)
try:
    cam.exp_mode(camera.AeMode.Auto)
except Exception:
    pass
countdown(3, [], "BRIGHTNESS: keep still")
exp, gain = float(cam.exposure()), float(cam.gain())
gain0 = gain
cam.exposure(int(exp))
cam.gain(int(gain))
for it in range(10):
    for _ in range(4):
        read()
    m = ring_median(read(), 0)
    print("exposure %6d us  gain %5d  ->  brightness %d (target %d)" % (exp, gain, m, TARGET_L))
    if m <= 2:
        print("picture black -> auto exposure kept (lens covered?)")
        try:
            cam.exp_mode(camera.AeMode.Auto)
        except Exception:
            pass
        break
    if abs(m - TARGET_L) <= 4:
        break
    want = exp * min(2.0, max(0.5, TARGET_L / float(m)))
    if want <= EXPOSURE_MAX_US:
        exp = want
    else:
        gain = min(gain * want / EXPOSURE_MAX_US, gain0 * GAIN_MAX_X)
        exp = EXPOSURE_MAX_US
    cam.exposure(int(exp))
    cam.gain(int(gain))
exp, gain = cam.exposure(), cam.gain()
print("BRIGHTNESS LOCKED: exposure %d us, gain %d" % (exp, gain))

# ---------- 1-3. colours ----------
img = read()
field_a, field_b = ring_median(img, 1), ring_median(img, 2)
print("field colour A %d B %d" % (field_a, field_b))


# the squares really show that colour? (else the goal / ball was not there)
def looks_like(name, th):
    a_mid, b_mid = (th[2] + th[3]) / 2, (th[4] + th[5]) / 2
    if name == "blue":   return b_mid < field_b - 8
    if name == "yellow": return b_mid > field_b + 8
    return a_mid > field_a + 15                     # orange: redder than the field


lab = {}
for name, squares, text in (("blue", GOAL_SQUARES, "BLUE goal on the squares"),
                            ("yellow", GOAL_SQUARES, "YELLOW goal on the squares"),
                            ("orange", BALL_SQUARES, "BALL on the square (or skip)")):
    print("== %s: %d s to get ready" % (name.upper(), WAIT_S))
    countdown(WAIT_S, squares, text)
    best = None
    t0 = time.ticks_ms()
    while time.ticks_ms() - t0 < MEASURE_S * 1000 and not app.need_exit():
        img = read()
        if img is None:
            continue
        th = measure(img, squares)
        if th:
            best = th if best is None else [min(best[0], th[0]), max(best[1], th[1]),
                                            min(best[2], th[2]), max(best[3], th[3]),
                                            min(best[4], th[4]), max(best[5], th[5])]
        show(img, squares, "%s: measuring" % name.upper())
    if best is None or not looks_like(name, best):
        lab[name] = OLD[name]
        print("%s: the squares do not show it (%s) -> old value kept %s" % (name, best, OLD[name]))
        continue
    lab[name] = [max(best[0] - MARGIN, 0), min(best[1] + MARGIN, 100),
                 max(best[2] - MARGIN, -128), min(best[3] + MARGIN, 127),
                 max(best[4] - MARGIN, -128), min(best[5] + MARGIN, 127)]
    print("%s_LAB = %s" % (name.upper(), lab[name]))

# yellow goal must not be read as the ball (A ranges apart)
if lab["orange"] is not OLD["orange"] and lab["orange"][2] <= lab["yellow"][3]:
    mid = (lab["yellow"][3] + lab["orange"][2]) // 2
    lab["yellow"][3], lab["orange"][2] = mid, mid + 1
    print("yellow / orange overlap on A: split at", mid)

cfg = {"exposure_us": int(exp), "gain": int(gain), "wb_gain": None, "saturation": SATURATION,
       "luma": 50, "yellow": lab["yellow"], "blue": lab["blue"], "orange": lab["orange"]}
try:
    with open(CONFIG_FILE, "w") as fp:
        json.dump(cfg, fp)
    print("SAVED", CONFIG_FILE, "-> run goal_ball_detection_auto.py")
except Exception as e:
    print("NOT SAVED (%s): copy the _LAB lines above into goal_ball_detection.py" % e)

# show what the new values find (stop with the MaixVision stop button)
while not app.need_exit():
    img = read()
    if img is None:
        continue
    for name in lab:
        for b in img.find_blobs([lab[name]], roi=ROI, merge=True, area_threshold=6, pixels_threshold=6):
            if R_MIN < math.hypot(b.cx() - CENTER_X, b.cy() - CENTER_Y) < R_MAX:
                img.draw_rect(b.x(), b.y(), b.w(), b.h(), COL[name], 2)
    show(img, [], "DONE: cyan blue / purple yellow / orange ball")
