#include "hud/scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hud {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadToDeg = 180.0F / kPi;

struct Point {
  float x;
  float y;
  float nearness;
};

float clamp(float value, float low, float high) {
  return std::min(high, std::max(low, value));
}

float finite_or_zero(const std::optional<float>& value) {
  return value && std::isfinite(*value) ? *value : 0.0F;
}

Point rotate(Point point, float degrees, float center_x, float center_y) {
  const float radians = degrees * kPi / 180.0F;
  const float cosine = std::cos(radians);
  const float sine = std::sin(radians);
  const float x = point.x - center_x;
  const float y = point.y - center_y;
  return {center_x + x * cosine - y * sine, center_y + x * sine + y * cosine,
          point.nearness};
}

Point project(float forward_m, float lateral_m, float vertical_m, float center_x,
              float center_y, float camera_height_m, float focal, float near_m) {
  const float depth = std::max(1.2F, forward_m + near_m);
  const float inverse_z = focal / depth;
  return {center_x + lateral_m * inverse_z,
          center_y + (camera_height_m - vertical_m) * inverse_z,
          inverse_z / (focal / near_m)};
}

void thick_line(DrawTarget& target, Point a, Point b, Color color, float width) {
  target.line(a.x, a.y, b.x, b.y, color, width);
}

void draw_world(DrawTarget& target, float width, float height, float roll_deg, float pitch_deg) {
  const float center_x = width / 2.0F;
  const float center_y = height / 2.0F;
  const float world_pitch = clamp(pitch_deg, -30.0F, 30.0F) * 6.0F;
  constexpr float depths[] = {2.0F, 8.0F, 16.0F, 28.0F, 44.0F, 64.0F};
  constexpr float lanes[] = {-3.0F, -1.5F, 0.0F, 1.5F, 3.0F};

  auto transform = [&](Point point) {
    point.y += world_pitch;
    return rotate(point, -roll_deg, center_x, center_y);
  };

  for (float lane : lanes) {
    Point previous = transform(project(depths[0], lane * 3.0F, 0.0F, center_x, center_y,
                                       3.6F, 240.0F, 6.0F));
    for (std::size_t i = 1; i < sizeof(depths) / sizeof(depths[0]); ++i) {
      Point next = transform(project(depths[i], lane * 3.0F, 0.0F, center_x, center_y,
                                     3.6F, 240.0F, 6.0F));
      thick_line(target, previous, next, kHudMuted, 1.0F);
      previous = next;
    }
  }
  for (float depth : depths) {
    Point left = transform(project(depth, lanes[0] * 3.0F, 0.0F, center_x, center_y,
                                   3.6F, 240.0F, 6.0F));
    Point right = transform(project(depth, lanes[4] * 3.0F, 0.0F, center_x, center_y,
                                    3.6F, 240.0F, 6.0F));
    thick_line(target, left, right, kHudMuted, 1.0F);
  }

  Point horizon_left{center_x - width, center_y + world_pitch, 1.0F};
  Point horizon_right{center_x + width, center_y + world_pitch, 1.0F};
  thick_line(target, rotate(horizon_left, -roll_deg, center_x, center_y),
             rotate(horizon_right, -roll_deg, center_x, center_y), kHudCyan, 1.0F);

  constexpr int ladder_steps[] = {-30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30};
  const float gap = width * 0.17F;
  for (int step : ladder_steps) {
    const bool major = step % 10 == 0;
    const float tick = std::min(width, height) * (major ? 0.07F : 0.035F);
    const float y = center_y - static_cast<float>(step) * 6.0F + world_pitch;
    Point ll{center_x - gap - tick, y, 1.0F};
    Point lr{center_x - gap, y, 1.0F};
    Point rl{center_x + gap, y, 1.0F};
    Point rr{center_x + gap + tick, y, 1.0F};
    ll = rotate(ll, -roll_deg, center_x, center_y);
    lr = rotate(lr, -roll_deg, center_x, center_y);
    rl = rotate(rl, -roll_deg, center_x, center_y);
    rr = rotate(rr, -roll_deg, center_x, center_y);
    thick_line(target, ll, lr, kHudPale, 1.0F);
    thick_line(target, rl, rr, kHudPale, 1.0F);
  }
}

void draw_trajectory(DrawTarget& target, const TrajectoryResolution& trajectory,
                     float width, float height, float roll_deg, float pitch_deg) {
  if (trajectory.point_count < 2U) return;
  const float center_x = width / 2.0F;
  const float center_y = height / 2.0F;
  const float camera_height = 2.3F + clamp(pitch_deg, -30.0F, 30.0F) * 0.28F;
  std::array<Point, kMaxTrajectoryPoints> center{};
  std::array<Point, kMaxTrajectoryPoints> left{};
  std::array<Point, kMaxTrajectoryPoints> right{};
  std::size_t count = trajectory.point_count;

  for (std::size_t i = 0; i < trajectory.point_count; ++i) {
    if (i >= 2U && trajectory.points[i].forward_m <= trajectory.points[i - 1U].forward_m) {
      count = i;
      break;
    }
    const ForwardPathPoint& point = trajectory.points[i];
    center[i] = rotate(project(point.forward_m, point.lateral_m, point.vertical_m, center_x,
                               center_y, camera_height, 240.0F, 6.0F),
                       roll_deg, center_x, center_y);
    left[i] = rotate(project(point.forward_m, point.lateral_m - 3.0F, point.vertical_m,
                             center_x, center_y, camera_height, 240.0F, 6.0F),
                     roll_deg, center_x, center_y);
    right[i] = rotate(project(point.forward_m, point.lateral_m + 3.0F, point.vertical_m,
                              center_x, center_y, camera_height, 240.0F, 6.0F),
                      roll_deg, center_x, center_y);
  }
  count = std::max<std::size_t>(2U, count);

  for (std::size_t i = 1; i < count; ++i) {
    thick_line(target, left[i - 1U], left[i], kHudPale, 1.0F);
    thick_line(target, right[i - 1U], right[i], kHudPale, 1.0F);
    const float delta_vertical = trajectory.points[i].vertical_m - trajectory.points[i - 1U].vertical_m;
    const Color path_color = delta_vertical > 0.02F ? kHudCyan
                           : delta_vertical < -0.02F ? kHudAmber : kHudMuted;
    thick_line(target, center[i - 1U], center[i], path_color,
               trajectory.is_stalled ? 1.0F : (1.0F + center[i].nearness));
    if (i % 4U == 0U) thick_line(target, left[i], right[i], kHudPale, 1.0F);
  }
  target.circle(center[count - 1U].x, center[count - 1U].y, 3.0F, kHudCyan, true);
}

void draw_aircraft_reference(DrawTarget& target, float width, float height) {
  const float x = width / 2.0F;
  const float y = height / 2.0F;
  const float half = std::min(width, height) * 0.14F;
  target.line(x - half, y, x - half * 0.24F, y, kHudAmber, 2.0F);
  target.line(x + half * 0.24F, y, x + half, y, kHudAmber, 2.0F);
  target.line(x - half * 0.24F, y, x - half * 0.10F, y + half * 0.17F, kHudAmber, 2.0F);
  target.line(x - half * 0.10F, y + half * 0.17F, x + half * 0.10F,
              y + half * 0.17F, kHudAmber, 2.0F);
  target.line(x + half * 0.10F, y + half * 0.17F, x + half * 0.24F, y, kHudAmber, 2.0F);
}

void draw_vario(DrawTarget& target, float vertical_rate, float width, float height) {
  const float center_y = height / 2.0F;
  const float x = width * 0.92F;
  const float half_height = height * 0.30F;
  target.line(x, center_y - half_height, x, center_y + half_height, kHudPale, 1.0F);
  for (int value = -10; value <= 10; value += 5) {
    const float y = center_y - static_cast<float>(value) * half_height / 10.0F;
    const float tick = value == 0 ? 9.0F : 5.0F;
    target.line(x - tick, y, x + tick, y, kHudPale, 1.0F);
  }
  const float bug_y = center_y - clamp(vertical_rate, -10.0F, 10.0F) * half_height / 10.0F;
  target.triangle(x - 2.0F, bug_y, x - 14.0F, bug_y - 6.0F, x - 14.0F,
                  bug_y + 6.0F, kHudAmber, true);
}

}  // namespace

void compose_unified_hud(const TelemetrySample& sample, DrawTarget& target,
                         const SceneConfig& config) {
  target.clear(kBlack);
  const float roll_deg = sample.attitude
      ? finite_or_zero(sample.attitude->roll_rad) * kRadToDeg : 0.0F;
  const float pitch_deg = sample.attitude
      ? finite_or_zero(sample.attitude->pitch_rad) * kRadToDeg : 0.0F;
  const TrajectoryResolution trajectory = resolve_trajectory(sample, config.trajectory);

  draw_world(target, config.width, config.height, roll_deg, pitch_deg);
  draw_trajectory(target, trajectory, config.width, config.height, roll_deg, pitch_deg);
  draw_aircraft_reference(target, config.width, config.height);
  draw_vario(target, trajectory.vertical_rate_m_s, config.width, config.height);

  if (trajectory.is_stalled) {
    target.text(config.width / 2.0F - 30.0F, config.height * 0.18F, "STALL", kHudRed, 2.0F);
  }
  const Coordination coordination = resolve_coordination(trajectory.turn_rate_rad_s,
                                                          trajectory.coordinated_turn_rate_rad_s);
  if (coordination != Coordination::kWingsLevel && coordination != Coordination::kCoordinated) {
    const char* label = coordination_name(coordination);
    target.text(config.width / 2.0F - 32.0F, config.height - 14.0F, label, kHudAmber, 1.0F);
    const Direction direction = resolve_coordination_direction(trajectory.turn_rate_rad_s,
                                                               trajectory.coordinated_turn_rate_rad_s);
    if (direction == Direction::kLeft) {
      target.triangle(72.0F, config.height - 13.0F, 84.0F, config.height - 18.0F,
                      84.0F, config.height - 8.0F, kHudAmber, true);
    } else if (direction == Direction::kRight) {
      target.triangle(config.width - 72.0F, config.height - 13.0F,
                      config.width - 84.0F, config.height - 18.0F,
                      config.width - 84.0F, config.height - 8.0F, kHudAmber, true);
    }
  }

  if (!sample.link_alive) {
    target.text(8.0F, 8.0F, "NO TELEMETRY", kHudRed, 1.0F);
  } else if (!sample.attitude) {
    target.text(8.0F, 8.0F, "ATTITUDE STALE", kHudRed, 1.0F);
  }

  char heading[12];
  const HeadingResolution resolved_heading = resolve_heading(sample);
  if (resolved_heading.heading_deg) {
    std::snprintf(heading, sizeof(heading), "%03d", static_cast<int>(std::lround(*resolved_heading.heading_deg)) % 360);
    target.text(config.width / 2.0F - 10.0F, 8.0F, heading, kHudPale, 1.0F);
  }
}

}  // namespace hud
