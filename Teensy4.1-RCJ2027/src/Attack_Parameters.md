# `act_attack3.cpp` Parameters Reference

Complete reference of all `#define` parameters and related sensor/array constants in `Teensy4.1-RCJ2027/src/act_attack3.cpp`.

---

## 1. Hardware, Pushing & Orbiting (`CHASE` / `ORBIT` / `PUSH`)

| Parameter | Default | Meaning & Usage |
| :--- | :--- | :--- |
| `MOTOR_REVERSE` | `1` | **Motor direction flag**: `1` for an attack chassis (inverted wiring relative to defense), `0` for defense. |
| `DATA_BUTTON` | `26` | **Data/pause button pin (SW2)**: Active-low button (`INPUT_PULLUP`) used to pause motion and display live diagnostic telemetry on the OLED (`dataScreen`). |
| `PUSH_DEG3` | `5.0f` | **Front push window (deg)**: Ball angle within `90 deg +- 5 deg` (`85–95 deg`) triggers a straight forward push instead of chasing or orbiting. |
| `PUSH_SPEED3` | `60.0f` | **Steady push speed (%)**: Forward speed when pushing the ball after the initial slow-start period. |
| `PUSH_SLOW3` | `40.0f` | **Initial push speed (%)**: Gentle initial forward speed when first entering the push window, reducing ball knock-away. |
| `PUSH_SLOW_MS` | `300` | **Initial push duration (ms)**: Time spent at `PUSH_SLOW3` before ramping up to `PUSH_SPEED3`. |
| `ORBIT_SLOW3` | `25.0f` | **Minimum orbit speed (%)**: Orbit speed when the ball is close to straight ahead (`0 deg` offset from `90 deg`). |
| `ORBIT_SLOW_DEG` | `60.0f` | **Orbit speed ramp angle (deg)**: Offset from front (`|off|`) at which orbit speed reaches the full `ORBIT_SPEED3`. |
| `CHASE_SPEED3` | `50.0f` | **Straight chase speed (%)**: Speed moving directly toward the ball when compensated IR strength is below `STR_ORBIT`. |
| `ORBIT_SPEED3` | `40.0f` | **Base orbit speed (%)**: Upper orbit speed when the ball is at or beyond `ORBIT_SLOW_DEG` off the front. |
| `STR_ORBIT` | `5` | **Orbit IR strength threshold**: At or above this compensated IR strength, the robot orbits around the ball rather than driving straight into it. |
| `STR_FULL` | `7.0f` | **Full-orbit IR strength**: Strength at which the exponential orbit factor `g` reaches `1.0` (full angular offset). |
| `STR_LAMBDA` | `2.9f` | **IR strength exponential scale**: Controls how `g = min(1, exp((str - STR_FULL) / STR_LAMBDA))` grows with IR strength (`0.50` at strength `5`, `~0.71` at `6`). |
| `ORBIT_K` | `1.5f` | **Orbit angle multiplier**: Multiplies `|off|` (angle off `90 deg`) before scaling by `g` to compute extra tangential offset around the ball. |
| `ORBIT_MAX_DEG` | `90.0f` | **Maximum orbit angle offset (deg)**: Caps the added angular offset around the ball. |
| `ORBIT_MIN_DEG` | `35.0f` | **Minimum orbit offset in front half (deg)**: Ensures a strong ball in front is never driven straight into. |
| `ORBIT_MIN_BACK_DEG` | `60.0f` | **Minimum orbit offset in back half (deg)**: Larger minimum offset when the ball is more than `90 deg` off the front (`|off| > 90 deg`). |

---

## 2. IR Wheel & Rear Shadow Compensation

| Parameter | Default | Meaning & Usage |
| :--- | :--- | :--- |
| `WHEEL_ZONE_DEG` | `20.0f` | **Wheel shadow angular radius (deg)**: Adds wheel bonus within `+-20 deg` of wheel angles (`45`, `135`, `225`, `315 deg`), tapering linearly to `0`. |
| `WHEEL_BONUS[4]` | `{3.0f, 3.0f, 2.0f, 2.0f}` | **Per-wheel IR strength bonus**: Compensates for IR ring occlusion near the four wheels (`45`, `135`, `225`, `315 deg`) so the robot does not misread a close ball as far. |
| `BACK_ZONE_DEG` | `30.0f` | **Rear shadow angular radius (deg)**: Adds rear bonus within `+-30 deg` of `270 deg` (behind the robot), tapering linearly to `0`. |
| `BACK_BONUS` | `2.0f` | **Rear IR strength bonus**: Compensates for IR signal attenuation around `270 deg` (behind the robot). |

---

## 3. White Line Guard (`lineGuard`)

| Parameter | Default | Meaning & Usage |
| :--- | :--- | :--- |
| `LINE_ESCAPE3` | `40.0f` | **Line escape push speed (%)**: Extra velocity pushed away from `lineDir` while locked onto a white line. |
| `LINE_SIDE_DEG` | `90.0f` | **Line side tracking limit (deg)**: Updates `lineDirField` only while the detected line angle stays within `+-90 deg` of the initial touch (`lineInitField`); if the last seen sensor is farther than `90 deg` away when the line disappears, `lineCrossed` is set so it keeps backing out until re-crossing properly. |
| `LINE_IGNORE3` | `0x0080UL` | **Broken ring channel bitmask**: Masks out ring channel `7` (`157.5 deg`) in software (`0x0080UL` = bit 7; `0` disables masking) because hardware/Sense line interrupts (`PIN_0` and `PIN_1`) are detached and cannot mask broken channels on the Act side. |

---

## 4. Camera Goal Damping, Own-Goal Risk & Opponent Penalty Area (`goalSlow`)

| Parameter | Default | Meaning & Usage |
| :--- | :--- | :--- |
| `GOAL_NEAR_CM` | `25.0f` | **Goal proximity floor (cm)**: Distance at which the goal damping factor reaches its minimum (`FRONT_MIN_K` or `OWN_MIN_K`). |
| `FRONT_MIN_K` | `0.5f` | **Minimum opponent-goal speed factor**: Lower bound on overall speed multiplier when moving toward the front (opponent) goal. |
| `FRONT_LAMBDA` | `30.0f` | **Opponent-goal damping length (cm)**: Exponential decay scale for slowing down as the robot approaches the opponent goal. |
| `THEIR_STOP_CM` | `2.0f` | **Opponent penalty-area stop margin (cm)**: Component of velocity moving toward the opponent goal drops to `0` once robot center is `2.0 cm` outside the opponent penalty area boundary (preventing full entry into their area). |
| `THEIR_LAMBDA` | `12.0f` | **Opponent penalty-area approach decay (cm)**: Exponential smoothing length for braking into the opponent penalty area boundary. |
| `THEIR_OUT_CM` | `-3.0f` | **Opponent penalty-area inside threshold (cm)**: Negative `theirArea3` (`< -3.0 cm`) indicates the robot center is inside their penalty area, overriding `v` to push straight out away from their goal. |
| `THEIR_OUT_SPEED` | `40.0f` | **Opponent penalty-area exit speed (%)**: Speed used to drive straight out when `theirInside3` is triggered. |
| `PA_FRONT_CM3` | `30.0f` | **Penalty area front distance (cm)**: Front boundary distance measured from the camera's goal reference point. |
| `PA_HALF_CM3` | `39.0f` | **Penalty area half-width (cm)**: Half-width of the opponent penalty area rectangle. |
| `PA_R_CM3` | `15.0f` | **Penalty area corner radius (cm)**: Rounded corner radius of the penalty area geometry. |
| `OWN_MIN_K` | `0.3f` | **Minimum own-goal risk speed factor**: Stronger speed reduction when moving toward the back (own) goal while the ball is between the robot and our goal. |
| `OWN_LAMBDA` | `30.0f` | **Own-goal risk damping length (cm)**: Exponential decay scale for own-goal risk braking. |
| `OWN_RISK_DEG` | `45.0f` | **Own-goal risk cone (deg)**: If the ball angle is within `+-45 deg` of the back goal direction and motion goes toward our goal, own-goal speed damping activates. |

---

## 5. Ultrasonic Wall Field (`usField`)

| Parameter | Default | Meaning & Usage |
| :--- | :--- | :--- |
| `US_FIELD3` | `1` | **Master ultrasonic damping switch**: `1` enables ultrasonic wall braking; `0` disables all ultrasonic velocity cuts while still displaying and logging readings. |
| `US_USE_FRONT3` | `1` | **Front ultrasonic enable (H1, `90 deg`)**: `1` uses front sensor; `0` marks it failed (`US_FAIL3` applied). |
| `US_USE_RIGHT3` | `1` | **Right ultrasonic enable (H2, `0 deg`)**: `1` uses right sensor; `0` marks it failed. |
| `US_USE_BACK3` | `1` | **Back ultrasonic enable (H3, `270 deg`)**: `1` uses back sensor; `0` marks it failed. |
| `US_USE_LEFT3` | `1` | **Left ultrasonic enable (H4, `180 deg`)**: `1` uses left sensor; `0` marks it failed. |
| `US_STALE3_MS` | `200` | **Ultrasonic packet timeout (ms)**: If no Sense ultrasonic packet (`senseUs`) arrives within `200 ms`, sensors are treated as failed (`0` reading). |
| `US_FAIL3` | `0.7f` | **Failed-sensor speed multiplier**: When a sensor is disabled, reads `0`, or times out, the component of `v` toward that side is multiplied by `0.7` (cut factor `1 - 0.7 = 0.3`). |
| `US_WALL_TO_LINE3` | `13.0f` | **Wall-to-white-line offset (cm)**: Distance from the physical wall to the white line center, subtracted from raw ultrasonic distance to estimate distance to the line. |
| `US_LINE_STOP3` | `3.0f` | **Ultrasonic line stop margin (cm)**: Distance before the white line where velocity component toward that wall reaches `0` cut (full stop into that direction). |
| `US_LAMBDA3` | `10.0f` | **Ultrasonic braking decay length (cm)**: Exponential decay constant controlling how smoothly speed toward a wall or line tapers off before `US_LINE_STOP3`. |
| `US_LAG3` | `0.30f` | **Ultrasonic reading lag (s)**: Estimated age or latency of ultrasonic measurements used to compensate for robot motion during the lag. |
| `US_LAG_MAX3` | `20.0f` | **Maximum lag compensation (cm)**: Caps the motion-lag distance offset (`moving * US_CMS_PCT3 * US_LAG3`) at `20.0 cm`. |
| `US_CMS_PCT3` | `3.0f` | **Speed-to-cm/s conversion (cm/s per %)**: Maps `100%` command output to `300 cm/s` (`3.0 cm/s` per `1%`) when computing lag distance. |
| `US_BALL_DEG3` | `30.0f` | **Front ball bypass angle (deg)**: Bypasses front ultrasonic braking when the robot holds the ball or the ball is within `90 deg +- 30 deg`, allowing the robot to reach and push the ball. |
