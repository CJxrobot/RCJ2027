from maix import camera, image, app, time, uart, err, pinmap
import math
import threading
import time as systime   # standard time (maix.time shadows the name "time")

# ================= PROTOCOL (v2) =================
# 13 bytes, little-endian uint16 fields:
#
#   [0]  0xCC          header 1
#   [1]  0xDD          header 2
#   [2]  flags         bit0 = ball found
#                      bit1 = goal found
#                      bit2 = goal colour (0 = yellow, 1 = blue)
#   [3]  ballAngleL    ball angle, degrees 0..359   (0xFFFF if not found)
#   [4]  ballAngleH
#   [5]  ballDistL     ball distance                (0xFFFF if not found)
#   [6]  ballDistH
#   [7]  goalAngleL    goal angle, degrees 0..359   (0xFFFF if not found)
#   [8]  goalAngleH
#   [9]  goalDistL     goal distance                (0xFFFF if not found)
#   [10] goalDistH
#   [11] checksum      (0xDD + bytes[2..10]) & 0xFF
#   [12] 0xEE          tail
PKT_LEN = 13
NOT_FOUND = 0xFFFF

FLAG_BALL = 0x01
FLAG_GOAL = 0x02
FLAG_GOAL_BLUE = 0x04

# ================= UART CONFIG =================
UART_BAUD = 115200
TX_HZ = 100          # packet send rate in Hz; set <= 0 to send once per camera frame

pin_function = {
    "A19": "UART1_TX",
    "A18": "UART1_RX"
}
device = "/dev/ttyS1"

for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")

ser = uart.UART(device, UART_BAUD)

# one packet takes PKT_LEN * 10 bits on the wire (8N1)
_max_hz = UART_BAUD / (PKT_LEN * 10.0)
if TX_HZ > _max_hz:
    print(f"WARNING: TX_HZ={TX_HZ} exceeds UART capacity (~{int(_max_hz)} Hz at {UART_BAUD} baud)")

# ================= VISION CONFIG =================
DEBUG_VIEW = True
GOAL_UPDATE_EVERY = 10     # goal is searched every N frames
BALL_UPDATE_EVERY = 1      # ball is searched every N frames (1 = every frame)

cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
cam.set_awb(1)

if DEBUG_VIEW:
    from maix import display
    disp = display.Display()

# 選擇要找的球門
YELLOW_LAB = [30, 77, -16, 28, 29, 72]
BLUE_LAB = [30, 53, -24, 23, -47, -10]

GOAL_LAB = YELLOW_LAB
# GOAL_LAB = BLUE_LAB
GOAL_IS_BLUE = GOAL_LAB is BLUE_LAB

# 橘色球 (starting values — tune in MaixVision threshold tool)
# keep A-min above the yellow goal's A-max (28) so the goal isn't read as the ball
#ORANGE_LAB = [20, 80, 30, 80, 15, 70]
ORANGE_LAB = [44, 75, 6, 47, 10, 53]
CENTER_X = 175
CENTER_Y = 125
R_MIN = 37
R_MAX = 100

BALL_R_MIN = R_MIN
BALL_R_MAX = R_MAX
BALL_MIN_PIXELS = 4

# 球門距離公式參數
A = 0.003379401783
B = 1671.194765
C = -23.10186756

# 球距離公式參數: dist = BALL_A*r^2 + BALL_B*r + BALL_C  (r = pixel radius from centre)
# defaults send the raw pixel radius — calibrate these like the goal formula
BALL_A = 0.0
BALL_B = 1.0
BALL_C = 0.0

HIST_N = 5
radius_history = []

frame_count = 0
best_goal = None
best_ball = None

goal_found = 0
goal_angle = NOT_FOUND
goal_dist = NOT_FOUND

ball_found = 0
ball_angle = NOT_FOUND
ball_dist = NOT_FOUND

last_print = time.ticks_ms()


def point_radius(x, y):
    return math.hypot(x - CENTER_X, y - CENTER_Y)


def point_angle_deg(x, y):
    return int(round(math.degrees(math.atan2(y - CENTER_Y, x - CENTER_X)) % 360)) % 360


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def pack_uint16(value):
    value = int(value) & 0xFFFF
    return value & 0xFF, (value >> 8) & 0xFF


def build_packet(b_found, b_angle, b_dist, g_found, g_angle, g_dist):
    flags = 0
    if b_found:
        flags |= FLAG_BALL
    if g_found:
        flags |= FLAG_GOAL
    if GOAL_IS_BLUE:
        flags |= FLAG_GOAL_BLUE

    payload = [flags]
    for v in (b_angle, b_dist, g_angle, g_dist):
        payload.extend(pack_uint16(v))

    checksum = (0xDD + sum(payload)) & 0xFF
    return bytes([0xCC, 0xDD] + payload + [checksum, 0xEE])


def current_packet():
    return build_packet(ball_found, ball_angle, ball_dist,
                        goal_found, goal_angle, goal_dist)


# ================= UART TX THREAD =================
_packet_lock = threading.Lock()
_latest_packet = current_packet()
_tx_running = True


def set_packet(pkt):
    global _latest_packet
    with _packet_lock:
        _latest_packet = pkt


def tx_loop(hz):
    period = 1.0 / hz
    next_t = systime.monotonic()
    while _tx_running:
        with _packet_lock:
            pkt = _latest_packet
        ser.write(pkt)

        next_t += period
        delay = next_t - systime.monotonic()
        if delay > 0:
            systime.sleep(delay)
        else:
            # fell behind (e.g. system hiccup): resync instead of bursting
            next_t = systime.monotonic()


tx_thread = None
if TX_HZ > 0:
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

        parts.append({
            "blob": b,
            "r": r,
            "angle": math.atan2(b.cy() - CENTER_Y, b.cx() - CENTER_X)
        })

    if not parts:
        return None

    main = max(parts, key=lambda p: p["blob"].pixels())

    group = []
    for part in parts:
        r_difference = abs(part["r"] - main["r"])
        angle_difference = angle_diff(part["angle"], main["angle"])
        if r_difference < 18 and angle_difference < math.radians(45):
            group.append(part["blob"])

    if not group:
        group = [main["blob"]]

    x1 = min(b.x() for b in group)
    y1 = min(b.y() for b in group)
    x2 = max(b.x() + b.w() for b in group)
    y2 = max(b.y() + b.h() for b in group)

    total_pixels = sum(b.pixels() for b in group)

    # 被遮擋分割時，使用最左和最右推算中心
    cx = (x1 + x2) // 2
    cy = sum(b.cy() * b.pixels() for b in group) // total_pixels

    return {
        "x": x1,
        "y": y1,
        "w": x2 - x1,
        "h": y2 - y1,
        "cx": int(cx),
        "cy": int(cy),
        "pixels": total_pixels,
        "count": len(group)
    }


def calculate_goal(goal):
    r = point_radius(goal["cx"], goal["cy"])

    radius_history.append(r)
    if len(radius_history) > HIST_N:
        radius_history.pop(0)

    average_radius = sum(radius_history) / len(radius_history)

    distance = (
        A * average_radius * average_radius +
        B / math.sqrt(goal["pixels"]) +
        C
    )
    distance = max(distance, 0.0)

    return point_angle_deg(goal["cx"], goal["cy"]), int(round(distance))


# ================= BALL DETECTION =================
def find_ball(blobs):
    best = None
    best_pixels = 0

    for b in blobs or []:
        if b.pixels() < BALL_MIN_PIXELS:
            continue
        r = point_radius(b.cx(), b.cy())
        if not (BALL_R_MIN <= r <= BALL_R_MAX):
            continue
        if b.pixels() > best_pixels:
            best_pixels = b.pixels()
            best = {
                "x": b.x(), "y": b.y(), "w": b.w(), "h": b.h(),
                "cx": b.cx(), "cy": b.cy(), "r": r,
                "pixels": b.pixels()
            }
    return best


def calculate_ball(ball):
    r = ball["r"]
    distance = BALL_A * r * r + BALL_B * r + BALL_C
    distance = max(distance, 0.0)
    return point_angle_deg(ball["cx"], ball["cy"]), int(round(distance))


# ================= MAIN LOOP =================
ROI = (CENTER_X - R_MAX, CENTER_Y - R_MAX, R_MAX * 2, R_MAX * 2)

try:
    while not app.need_exit():
        img = cam.read()
        if img is None:
            continue

        img = img.flip(image.FlipDir.X)
        img = img.flip(image.FlipDir.Y)

        frame_count += 1
        changed = False

        # ---- ball ----
        if frame_count % BALL_UPDATE_EVERY == 0:
            ball_blobs = img.find_blobs(
                [ORANGE_LAB],
                roi=ROI,
                merge=False,
                area_threshold=BALL_MIN_PIXELS,
                pixels_threshold=BALL_MIN_PIXELS
            )
            best_ball = find_ball(ball_blobs)

            if best_ball:
                ball_found = 1
                ball_angle, ball_dist = calculate_ball(best_ball)
                ball_dist = min(ball_dist, 65534)
            else:
                ball_found = 0
                ball_angle = NOT_FOUND
                ball_dist = NOT_FOUND
            changed = True

        # ---- goal ----
        if frame_count % GOAL_UPDATE_EVERY == 0:
            goal_blobs = img.find_blobs(
                [GOAL_LAB],
                roi=ROI,
                merge=False,
                area_threshold=8,
                pixels_threshold=8
            )
            best_goal = merge_goal_parts(goal_blobs)

            if best_goal:
                goal_found = 1
                goal_angle, goal_dist = calculate_goal(best_goal)
                goal_dist = min(goal_dist, 65534)
            else:
                goal_found = 0
                goal_angle = NOT_FOUND
                goal_dist = NOT_FOUND
                radius_history.clear()
            changed = True

        # only rebuild the packet when the data was refreshed
        if changed:
            set_packet(current_packet())

        if TX_HZ <= 0:
            with _packet_lock:
                ser.write(_latest_packet)

        if DEBUG_VIEW:
            img.draw_circle(CENTER_X, CENTER_Y, R_MIN, image.Color.from_rgb(0, 255, 0), 1)
            img.draw_circle(CENTER_X, CENTER_Y, R_MAX, image.Color.from_rgb(255, 0, 0), 1)
            img.draw_cross(CENTER_X, CENTER_Y, image.Color.from_rgb(255, 255, 255), 8, 1)

            if best_goal:
                img.draw_rect(best_goal["x"], best_goal["y"], best_goal["w"], best_goal["h"],
                              image.Color.from_rgb(255, 255, 0), 2)
                img.draw_cross(best_goal["cx"], best_goal["cy"],
                               image.Color.from_rgb(0, 255, 0), 8, 2)
                img.draw_line(CENTER_X, CENTER_Y, best_goal["cx"], best_goal["cy"],
                              image.Color.from_rgb(255, 255, 255), 2)

            if best_ball:
                rad = max(best_ball["w"], best_ball["h"]) // 2 + 2
                img.draw_circle(best_ball["cx"], best_ball["cy"], rad,
                                image.Color.from_rgb(255, 128, 0), 2)
                img.draw_line(CENTER_X, CENTER_Y, best_ball["cx"], best_ball["cy"],
                              image.Color.from_rgb(255, 128, 0), 1)

            now = time.ticks_ms()
            if now - last_print >= 200:
                last_print = now
                print("ball:", ball_found, ball_angle, ball_dist,
                      "| goal:", goal_found, goal_angle, goal_dist,
                      "| fps:", round(time.fps(), 1))
            disp.show(img)
finally:
    _tx_running = False
    if tx_thread is not None:
        tx_thread.join(timeout=0.5)