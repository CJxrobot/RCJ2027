#include <core_act.h>

static Vec2 polar(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

static float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void strategy_begin() {
  robot.P_factor = HEADING_P;
  robot.heading_threshold = HEADING_DEADBAND;
  robot.robot_heading = 90.0f;
}

__attribute__((weak)) UsData US_Sensor() { return UsData(); }

// ===== Line =====
LineCtl back_to_field(bool got_line_packet) {
  static bool active = false;
  static float first = -1;   // direction when the line was first seen
  static float away = 0;
  static float dir = -1;
  static uint32_t lastFlag = 0;

  LineCtl out;
  uint32_t now = millis();

  if (line_flag()) {
    lastFlag = now;
    if (!active) { active = true; first = -1; dir = -1; }
    if (got_line_packet) {
      if (first < 0) first = line_angle(lineData.state);
      float d = line_angle(lineData.state, first);
      if (d >= 0) { dir = d; away = d + 180.0f; }
    }
  } else if (active && now - lastFlag >= LINE_HOLD_MS) {
    active = false;
  }

  out.active = active;
  out.line_dir = dir;
  if (active && dir >= 0) out.v = polar(LINE_AVOID_SPEED, away);
  return out;
}

// ===== Ball =====
MergedBall merge_ball_data(const FieldData& field, const IrBallData& ir) {
  MergedBall m;
  bool cam = camera_fresh(CAM_TIMEOUT_MS) && field.ball.seen;
  bool irOk = ir_ball_fresh(IR_TIMEOUT_MS);

  if (cam && (!irOk || fabsf(wrap180(field.ball.angle - ir.angle)) < MERGE_MAX_ERR_DEG)) {
    m.src = SRC_CAM;
    m.angle = field.ball.angle;
    m.closeness = clamp01((ORBIT_FAR_CM - field.ball.dist) / (ORBIT_FAR_CM - ORBIT_NEAR_CM));
  } else if (irOk) {
    m.src = SRC_IR;
    m.angle = ir.angle;
    m.closeness = clamp01((ir.strength - IR_FAR_STR) / (IR_NEAR_STR - IR_FAR_STR));
  }
  return m;
}

Vec2 chase_ball(const MergedBall& ball, const FieldData& field) {
  if (ball.src == SRC_NONE) {
    // No ball anywhere: go home toward our own goal, stop when close
    const FieldObj& g = field.back_goal;
    if (camera_fresh(CAM_TIMEOUT_MS) && g.seen && g.dist > HOME_DIST_CM)
      return polar(HOME_SPEED, g.angle);
    return Vec2();
  }

  float diff = wrap180(ball.angle - 90.0f);        // 0 = ball straight ahead
  if (fabsf(diff) <= CHASE_FRONT_DEG) return polar(CHASE_SPEED, ball.angle);

  // Ball to the side or behind: aim past it so the robot circles behind the ball.
  // Closer ball -> bigger offset.
  float offset = min(CHASE_OFFSET_MAX, CHASE_OFFSET_GAIN * fabsf(diff)) * ball.closeness;
  float dir = ball.angle + (diff > 0 ? offset : -offset);
  return polar(CHASE_SPEED, dir);
}

// ===== Heading =====
float rotate(const FieldData& field) {
  const FieldObj& g = field.front_goal;
  if (!camera_fresh(CAM_TIMEOUT_MS) || !g.seen) return 90.0f;
  // Turning CCW by d moves an object's angle by -d, so turn by (goal - 90)
  return current_heading() + wrap180(g.angle - 90.0f);
}

// ===== Vector field =====
static void push(Vec2& f, float k, float dist_cm, float from_deg) {
  float m = k * expf(-(dist_cm - VF_D0_CM) / VF_LAMBDA_CM);
  Vec2 p = polar(m, from_deg + 180.0f);   // away from the object
  f.x += p.x;
  f.y += p.y;
}

Vec2 Vector_Field(const FieldData& field, const UsData& us) {
  Vec2 f;
  if (camera_fresh(CAM_TIMEOUT_MS)) {
    const FieldObj* goals[2] = {&field.front_goal, &field.back_goal};
    for (const FieldObj* g : goals) {
      if (!g->seen || g->dist == PKT_NONE) continue;
      push(f, VF_K, g->dist, g->angle);
      if (g->dist < PENALTY_R_CM) {        // inside the curved penalty area
        Vec2 p = polar(VF_PENALTY_K, g->angle + 180.0f);
        f.x += p.x;
        f.y += p.y;
      }
    }
  }
  // Walls (R03): front 90, left 180, back 270, right 0
  if (us.front > 0) push(f, VF_K, us.front, 90);
  if (us.left  > 0) push(f, VF_K, us.left, 180);
  if (us.back  > 0) push(f, VF_K, us.back, 270);
  if (us.right > 0) push(f, VF_K, us.right, 0);

  float m = sqrtf(f.x * f.x + f.y * f.y);
  if (m > VF_MAX) { f.x *= VF_MAX / m; f.y *= VF_MAX / m; }
  return f;
}

const char* ball_src_name(BallSrc s) {
  return s == SRC_CAM ? "CAM" : (s == SRC_IR ? "IR" : "NONE");
}
