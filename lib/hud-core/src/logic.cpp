#include "hud/logic.hpp"

#include <algorithm>
#include <cmath>

namespace hud {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadToDeg = 180.0F / kPi;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kGravity = 9.81F;

float clamp(float value, float low, float high) {
  return std::min(high, std::max(low, value));
}

bool valid(const std::optional<float>& value) {
  return value.has_value() && std::isfinite(*value);
}

float shortest_angle_radians(float from, float to) {
  float delta = to - from;
  if (delta > kPi) delta -= 2.0F * kPi;
  if (delta < -kPi) delta += 2.0F * kPi;
  return delta;
}

float speed_from_velocity(std::int16_t north_cm_s, std::int16_t east_cm_s) {
  return std::hypot(static_cast<float>(north_cm_s), static_cast<float>(east_cm_s)) / 100.0F;
}

}  // namespace

float normalize_degrees(float degrees) {
  const float normalized = std::fmod(degrees, 360.0F);
  return normalized < 0.0F ? normalized + 360.0F : normalized;
}

float heading_rate_from_body_rates(float roll_rad, float pitch_rad, float pitch_rate_rad_s,
                                   float yaw_rate_rad_s) {
  const float guarded_pitch = clamp(pitch_rad, -1.396F, 1.396F);
  const float guarded_cos_pitch = std::max(0.17F, std::cos(guarded_pitch));
  return (std::sin(roll_rad) * pitch_rate_rad_s + std::cos(roll_rad) * yaw_rate_rad_s) /
         guarded_cos_pitch;
}

float climb_angle_rate_from_body_rates(float roll_rad, float pitch_rate_rad_s,
                                       float yaw_rate_rad_s) {
  return std::cos(roll_rad) * pitch_rate_rad_s - std::sin(roll_rad) * yaw_rate_rad_s;
}

HeadingResolution resolve_heading(const TelemetrySample& sample) {
  if (sample.vfr_hud && valid(sample.vfr_hud->heading_deg)) {
    return {normalize_degrees(*sample.vfr_hud->heading_deg), HeadingSource::kVfrHud, false};
  }
  if (sample.attitude && valid(sample.attitude->yaw_rad)) {
    return {normalize_degrees(*sample.attitude->yaw_rad * kRadToDeg),
            HeadingSource::kAttitudeYaw, true};
  }
  if (sample.global_position && sample.global_position->heading_cdeg &&
      *sample.global_position->heading_cdeg != 65535U) {
    return {normalize_degrees(static_cast<float>(*sample.global_position->heading_cdeg) / 100.0F),
            HeadingSource::kGlobalPosition, true};
  }
  return {};
}

ScalarTelemetry resolve_scalars(const TelemetrySample& sample) {
  ScalarTelemetry result;
  if (sample.vfr_hud) {
    if (valid(sample.vfr_hud->airspeed_m_s)) result.airspeed_m_s = sample.vfr_hud->airspeed_m_s;
    if (valid(sample.vfr_hud->groundspeed_m_s)) result.groundspeed_m_s = sample.vfr_hud->groundspeed_m_s;
    if (valid(sample.vfr_hud->climb_m_s)) result.climb_m_s = sample.vfr_hud->climb_m_s;
  }
  if (!result.groundspeed_m_s && sample.global_position &&
      sample.global_position->velocity_north_cm_s && sample.global_position->velocity_east_cm_s) {
    result.groundspeed_m_s = speed_from_velocity(*sample.global_position->velocity_north_cm_s,
                                                 *sample.global_position->velocity_east_cm_s);
  }
  if (!result.groundspeed_m_s && sample.gps_raw && sample.gps_raw->speed_cm_s) {
    result.groundspeed_m_s = static_cast<float>(*sample.gps_raw->speed_cm_s) / 100.0F;
  }
  if (!result.climb_m_s && sample.global_position && sample.global_position->velocity_down_cm_s) {
    result.climb_m_s = -static_cast<float>(*sample.global_position->velocity_down_cm_s) / 100.0F;
  }
  return result;
}

std::optional<float> resolve_track_degrees(const TelemetrySample& sample) {
  if (sample.global_position && sample.global_position->velocity_north_cm_s &&
      sample.global_position->velocity_east_cm_s) {
    const auto north = *sample.global_position->velocity_north_cm_s;
    const auto east = *sample.global_position->velocity_east_cm_s;
    if (speed_from_velocity(north, east) >= 0.01F) {
      return normalize_degrees(std::atan2(static_cast<float>(east), static_cast<float>(north)) *
                               kRadToDeg);
    }
  }
  if (sample.gps_raw && sample.gps_raw->course_cdeg && sample.gps_raw->speed_cm_s &&
      *sample.gps_raw->speed_cm_s >= 1U) {
    return normalize_degrees(static_cast<float>(*sample.gps_raw->course_cdeg) / 100.0F);
  }
  return std::nullopt;
}

TrajectoryResolution resolve_trajectory(const TelemetrySample& sample,
                                        const TrajectoryConfig& config) {
  TrajectoryResolution result;
  const ScalarTelemetry scalar = resolve_scalars(sample);
  const HeadingResolution heading = resolve_heading(sample);
  const std::optional<float> track = resolve_track_degrees(sample);

  const float speed = scalar.groundspeed_m_s.value_or(0.0F);
  const float airspeed = scalar.airspeed_m_s.value_or(speed);
  const float pitch = sample.attitude && valid(sample.attitude->pitch_rad)
                          ? *sample.attitude->pitch_rad : 0.0F;
  const float roll = sample.attitude && valid(sample.attitude->roll_rad)
                         ? *sample.attitude->roll_rad : 0.0F;
  const float heading_rad = heading.heading_deg.value_or(0.0F) * kDegToRad;
  const float track_rad = track.value_or(heading.heading_deg.value_or(0.0F)) * kDegToRad;
  const float drift_rad = shortest_angle_radians(heading_rad, track_rad);

  result.speed_m_s = speed;
  result.airspeed_m_s = airspeed;
  result.stall_speed_m_s = config.stall_speed_m_s;
  result.heading_deg = heading.heading_deg;
  result.track_deg = track;
  result.heading_track_delta_deg = drift_rad * kRadToDeg;
  result.is_stalled = airspeed < config.stall_speed_m_s;

  const float effective_forward_speed = std::max(0.0F, airspeed - config.stall_speed_m_s);
  const float heading_blend = clamp((airspeed - config.stall_speed_m_s) /
                                        std::max(1.0F, config.stall_speed_m_s),
                                    0.0F, 1.0F);
  (void)heading_blend;  // Retained for parity with the world-space source projection.
  result.wind_drift_m_s = clamp(speed * std::sin(drift_rad) * config.wind_offset_gain, -8.0F, 8.0F);

  const bool has_pitch_rate = sample.attitude && valid(sample.attitude->pitch_rate_rad_s);
  const bool has_yaw_rate = sample.attitude && valid(sample.attitude->yaw_rate_rad_s);
  const bool has_body_rates = has_pitch_rate || has_yaw_rate;
  const float pitch_rate = has_pitch_rate ? *sample.attitude->pitch_rate_rad_s : 0.0F;
  const float yaw_rate = has_yaw_rate ? *sample.attitude->yaw_rate_rad_s : 0.0F;

  result.coordinated_turn_rate_rad_s = clamp(
      (kGravity * std::tan(roll)) / std::max(airspeed, config.stall_speed_m_s), -1.8F, 1.8F);
  result.turn_rate_rad_s = has_body_rates
      ? clamp(heading_rate_from_body_rates(roll, pitch, pitch_rate, yaw_rate), -1.8F, 1.8F)
      : result.coordinated_turn_rate_rad_s;
  result.climb_angle_rate_rad_s = has_body_rates
      ? climb_angle_rate_from_body_rates(roll, pitch_rate, yaw_rate) : 0.0F;

  const std::optional<float> vertical_speed = scalar.climb_m_s;
  const float gamma_initial = vertical_speed && airspeed > 0.01F
      ? std::asin(clamp(*vertical_speed / airspeed, -1.0F, 1.0F)) : pitch;
  result.vertical_rate_m_s = airspeed * std::sin(gamma_initial);
  result.flight_path_angle_deg = gamma_initial * kRadToDeg;

  const float safe_step = std::max(0.05F, config.step_s);
  const std::size_t steps = std::min<std::size_t>(
      kMaxTrajectoryPoints - 1U,
      static_cast<std::size_t>(std::max(1.0F, std::floor(config.horizon_s / safe_step))));
  result.point_count = steps + 1U;
  result.points[0] = {};

  float relative_heading = 0.0F;
  float gamma = gamma_initial;
  for (std::size_t index = 1; index <= steps; ++index) {
    relative_heading += result.turn_rate_rad_s * safe_step;
    gamma = clamp(gamma + result.climb_angle_rate_rad_s * safe_step, -1.047F, 1.047F);
    const float pace = effective_forward_speed * safe_step;
    const float horizontal = std::cos(gamma) * pace;
    const float climb = std::sin(gamma) * pace;
    const ForwardPathPoint& previous = result.points[index - 1U];
    result.points[index] = {
        previous.forward_m + std::cos(relative_heading) * horizontal,
        previous.lateral_m + std::sin(relative_heading) * horizontal + result.wind_drift_m_s * safe_step,
        previous.vertical_m + climb,
        static_cast<float>(index) * safe_step,
    };
  }
  return result;
}

Coordination resolve_coordination(float actual, float coordinated) {
  if (std::fabs(coordinated) < 0.02F) {
    return std::fabs(actual) < 0.02F ? Coordination::kWingsLevel : Coordination::kFlatTurn;
  }
  const float tolerance = std::max(0.03F, std::fabs(coordinated) * 0.15F);
  if (std::fabs(actual - coordinated) <= tolerance) return Coordination::kCoordinated;
  return std::fabs(actual) < std::fabs(coordinated) ? Coordination::kSlipping
                                                    : Coordination::kSkidding;
}

Direction resolve_coordination_direction(float actual, float coordinated) {
  const Coordination state = resolve_coordination(actual, coordinated);
  if (state != Coordination::kSlipping && state != Coordination::kSkidding) return Direction::kNone;
  const float mismatch = coordinated - actual;
  if (std::fabs(mismatch) < 0.0001F) return Direction::kNone;
  return mismatch > 0.0F ? Direction::kRight : Direction::kLeft;
}

const char* coordination_name(Coordination coordination) {
  switch (coordination) {
    case Coordination::kWingsLevel: return "WINGS LEVEL";
    case Coordination::kCoordinated: return "COORDINATED";
    case Coordination::kFlatTurn: return "FLAT TURN";
    case Coordination::kSlipping: return "SLIPPING";
    case Coordination::kSkidding: return "SKIDDING";
  }
  return "";
}

}  // namespace hud
