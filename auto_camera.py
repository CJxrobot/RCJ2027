# ============================================================
#  AUTO CAMERA SETUP (omni mirror) -- run from MaixVision before a match / in a new room
# ============================================================
#  For dark (or very bright) fields where the goals are not found any more.
#
#  1. Put the robot on the field centre spot, both goals visible in the mirror ring
#     (the ball may lie anywhere in the ring, or be left out).
#  2. Run this script. It takes ~15 s and touches nothing else:
#       a. auto white balance + auto exposure settle, then the white balance is LOCKED
#       b. exposure / gain set so the ring has the brightness TARGET_L (dark field: longer
#          exposure, then more gain)
#       c. yellow goal, blue goal and the orange ball are found by themselves (largest
#          yellow / blue / orange area in the ring) and their LAB ranges measured
#  3. Result: saved to CONFIG_FILE and printed. goal_ball_detection.py loads that file at
#     start-up (camera settings + YELLOW_LAB / BLUE_LAB / ORANGE_LAB). Delete the file to go
#     back to the values written in goal_ball_detection.py.
#  Screen: purple = yellow found, cyan = blue found, orange = ball found.
#  Nothing found for a colour -> its old value is kept (message in the terminal).
# ============================================================

CONFIG_FILE = "/root/camera_auto.json"

TARGET_L = 45          # wanted median brightness (L 0-100) of the ring
SATURATION = 65        # 0-100 (camera default 50): colours stand out more in a dark room
LUMA = 50              # 0-100 ISP brightness (exposure does the work, keep 50)
EXPOSURE_MAX_US = 30000  # longest exposure (30 fps frame; longer = blur)
GAIN_MAX_X = 8         # gain at most this x the auto exposure's gain
FRAMES = 12            # frames measured for the colours
MARGIN = 5             # added to each side of a measured range
MIN_GOAL_PIX = 120     # a goal has at least this many pixels
MIN_BALL_PIX = 6

# Same image geometry as goal_ball_detection.py (Y flip only)
CENTER_X = 175
CENTER_Y = 116
R_MIN = 35
R_MAX = 100

# Each colour's area is found relative to the FIELD colour (median A / B of the ring), so a
# dark green field that looks a bit yellow is not taken for the yellow goal:
#   yellow: B at least COLOUR_GAP over the field, A within +-40
#   blue:   B at least COLOUR_GAP under the field
#   orange: A at least BALL_GAP over the field and over the yellow goal's A (found first), B over the field
COLOUR_GAP = 12
BALL_GAP = 20
OLD = {   # kept if a colour is not found (same as goal_ball_detection.py)
    "yellow": [30, 77, -16, 28, 29, 72],
    "blue":   [30, 53, -24, 23, -47, -10],
    "orange": [44, 75, 6, 47, 10, 53],
}

from maix import camera, image, display, app, time
import math, json

ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)
cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
disp = display.Display()


def read():
    img = cam.read()
    return img.flip(image.FlipDir.Y) if img is not None else None


def settle(n):
    for _ in range(n):
        img = read()
        if img is not None:
            disp.show(img)


def in_ring(b):
    return R_MIN < math.hypot(b.cx() - CENTER_X, b.cy() - CENTER_Y) < R_MAX


def pixels(img, th, roi=ROI):
    return sum(b.pixels() for b in img.find_blobs([th], roi=roi, merge=False,
                                                   area_threshold=1, pixels_threshold=1))


# median L of the ring (histogram from find_blobs pixel counts, 5-wide bins)
def median_l(img):
    # from L 5: the black outside the mirror and the corners of the ROI square are not counted
    hist = [(lo, pixels(img, [lo, lo + 4, -128, 127, -128, 127])) for lo in range(5, 100, 5)]
    total = sum(c for _, c in hist)
    if total == 0:
        return 0
    acc = 0
    for lo, c in hist:
        acc += c
        if acc >= total / 2:
            return lo + 2
    return 50


# median of channel ch (1 = A, 2 = B) in the ring: the field colour
def median_ch(img, ch):
    hist = []
    for lo in range(-60, 60, 4):
        th = [5, 100, -128, 127, -128, 127]
        th[2 * ch], th[2 * ch + 1] = lo, lo + 3
        hist.append((lo, pixels(img, th)))
    total = sum(c for _, c in hist)
    acc = 0
    for lo, c in hist:
        acc += c
        if total and acc >= total / 2:
            return lo + 2
    return 0


CAND = {}
def set_candidates(img):
    fa, fb = median_ch(img, 1), median_ch(img, 2)
    CAND["yellow"] = [5, 100, -40, 40, min(fb + COLOUR_GAP, 127), 127]
    CAND["blue"] = [3, 100, -60, 60, -128, max(fb - COLOUR_GAP, -128)]
    CAND["orange"] = [5, 100, min(fa + BALL_GAP, 127), 127, fb, 127]
    print("field colour A %d B %d -> candidates %s" % (fa, fb, CAND))


# range of one channel inside roi, among the pixels of the candidate threshold (5..95 %)
def channel_range(img, cand, ch, roi):
    lo_lim, hi_lim = (0, 100) if ch == 0 else (-128, 127)
    lo_lim, hi_lim = max(lo_lim, cand[2 * ch]), min(hi_lim, cand[2 * ch + 1])
    hist = []
    v = lo_lim
    while v <= hi_lim:
        th = list(cand)
        th[2 * ch], th[2 * ch + 1] = v, min(v + 1, hi_lim)
        hist.append((v, min(v + 1, hi_lim), pixels(img, th, roi)))
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


# the colour's largest area in the ring -> its LAB range, or None
def measure(img, name, min_pix):
    blobs = [b for b in img.find_blobs([CAND[name]], roi=ROI, merge=False,
                                       area_threshold=min_pix, pixels_threshold=min_pix)
             if in_ring(b)]
    if not blobs:
        return None, None
    b = max(blobs, key=lambda b: b.pixels())
    roi = (b.x(), b.y(), max(b.w(), 2), max(b.h(), 2))
    r = [channel_range(img, CAND[name], ch, roi) for ch in range(3)]
    if None in r:
        return None, b
    return [r[0][0], r[0][1], r[1][0], r[1][1], r[2][0], r[2][1]], b


# ---------------- a. white balance + exposure settle, lock the white balance ----------------
print("AUTO CAMERA: settling (auto white balance + auto exposure) ...")
try:
    cam.saturation(SATURATION)
    cam.luma(LUMA)
except Exception as e:
    print("saturation / luma not set:", e)
cam.set_awb(1)
try:
    cam.exp_mode(camera.AeMode.Auto)
except Exception:
    pass
settle(60)
wb = None
try:
    wb = list(cam.set_wb_gain())          # current auto gains ...
    cam.set_wb_gain(wb)                    # ... set = white balance locked
    print("white balance locked:", [round(g, 4) for g in wb])
except Exception as e:
    print("white balance not locked (stays auto):", e)

# ---------------- b. exposure / gain to TARGET_L ----------------
exp0, gain0 = cam.exposure(), cam.gain()
exp, gain = float(exp0), float(gain0)
cam.exposure(int(exp))                     # manual mode from here
cam.gain(int(gain))
for it in range(12):
    settle(4)
    img = read()
    m = median_l(img)
    print("exposure %6d us  gain %5d  ->  ring median L %d (target %d)" % (exp, gain, m, TARGET_L))
    if abs(m - TARGET_L) <= 3:
        break
    ratio = min(2.0, max(0.5, TARGET_L / max(m, 1)))
    want = exp * ratio
    if want <= EXPOSURE_MAX_US:
        exp = want
    else:                                  # exposure at its limit: the rest with gain
        rest = want / EXPOSURE_MAX_US
        exp = EXPOSURE_MAX_US
        gain = min(gain * rest, gain0 * GAIN_MAX_X)
    if ratio < 1 and gain > gain0:         # too bright with extra gain: take gain back first
        gain = max(gain0, gain * ratio)
    cam.exposure(int(exp))
    cam.gain(int(gain))
exp, gain = cam.exposure(), cam.gain()     # what the camera really took
print("EXPOSURE LOCKED: %d us, gain %d" % (exp, gain))

# ---------------- c. colours ----------------
settle(4)
set_candidates(read())
found = {"yellow": None, "blue": None, "orange": None}
COL = {"yellow": image.Color.from_rgb(255, 0, 255), "blue": image.Color.from_rgb(0, 255, 255),
       "orange": image.Color.from_rgb(255, 128, 0)}
for f in range(FRAMES):
    img = read()
    if img is None:
        continue
    for name in ("yellow", "blue", "orange"):            # yellow first: orange must be redder
        if name == "orange" and found["yellow"]:
            CAND["orange"][2] = max(CAND["orange"][2], found["yellow"][3] + 3)
        th, b = measure(img, name, MIN_BALL_PIX if name == "orange" else MIN_GOAL_PIX)
        if th:
            old = found[name]
            found[name] = th if old is None else [min(old[0], th[0]), max(old[1], th[1]),
                                                  min(old[2], th[2]), max(old[3], th[3]),
                                                  min(old[4], th[4]), max(old[5], th[5])]
        if b:
            img.draw_rect(b.x(), b.y(), b.w(), b.h(), COL[name], 2)
    img.draw_circle(CENTER_X, CENTER_Y, R_MIN, image.Color.from_rgb(0, 255, 0), 1)
    img.draw_circle(CENTER_X, CENTER_Y, R_MAX, image.Color.from_rgb(255, 0, 0), 1)
    disp.show(img)

lab = {}
for name, th in found.items():
    if th is None:
        lab[name] = OLD[name]
        print("%-6s NOT FOUND -> old value kept %s" % (name, OLD[name]))
        continue
    lab[name] = [max(th[0] - MARGIN, 0), min(th[1] + MARGIN, 100),
                 max(th[2] - MARGIN, -128), min(th[3] + MARGIN, 127),
                 max(th[4] - MARGIN, -128), min(th[5] + MARGIN, 127)]
# keep yellow and orange apart on A (the goal must not be read as the ball)
if found["yellow"] and found["orange"] and lab["orange"][2] <= lab["yellow"][3]:
    mid = (found["yellow"][3] + found["orange"][2]) // 2
    lab["yellow"][3], lab["orange"][2] = mid, mid + 1
    print("yellow / orange overlap on A: split at", mid)

# check: how many pixels each final range finds in the ring now
img = read()
for name in lab:
    print("%-6s %s  -> %d px in the ring now" % (name.upper() + "_LAB", lab[name], pixels(img, lab[name])))

cfg = {"exposure_us": int(exp), "gain": int(gain), "wb_gain": wb, "saturation": SATURATION,
       "luma": LUMA, "yellow": lab["yellow"], "blue": lab["blue"], "orange": lab["orange"]}
try:
    with open(CONFIG_FILE, "w") as fp:
        json.dump(cfg, fp)
    print("SAVED", CONFIG_FILE, "-> goal_ball_detection.py uses it from the next start")
except Exception as e:
    print("NOT SAVED (%s): copy the lines above into goal_ball_detection.py" % e)

# keep showing what the saved ranges find (stop with the MaixVision stop button)
while not app.need_exit():
    img = read()
    if img is None:
        continue
    for name in lab:
        for b in img.find_blobs([lab[name]], roi=ROI, merge=True,
                                area_threshold=MIN_BALL_PIX, pixels_threshold=MIN_BALL_PIX):
            if in_ring(b):
                img.draw_rect(b.x(), b.y(), b.w(), b.h(), COL[name], 1)
    disp.show(img)
