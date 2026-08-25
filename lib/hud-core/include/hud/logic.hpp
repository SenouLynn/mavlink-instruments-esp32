#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "hud/telemetry.hpp"

namespace hud {

enum class HeadingSource { kVfrHud, kAttitudeYaw, kGlobalPosition, kUnavailable };

struct HeadingResolution {
  std::optional<float> heading_deg;
  HeadingSource source = HeadingSource::kUnavailable;
  bool is_fallback = false;
};

struct ScalarTelemetry {
  std::optional<float> airspeed_m_s;
  std::optional<float> groundspeed_m_s;
  std::optional<float> climb_m_s;
};

struct ForwardPathPoint {
  float forward_m = 0.0F;
  float lateral_m = 0.0F;
  float vertical_m = 0.0F;
  float time_s = 0.0F;
};

struct TrajectoryConfig {
  float stall_speed_m_s = 14.0F;
  float horizon_s = 5.0F;
  float step_s = 0.25F;
  float heading_track_blend = 0.45F;
  float wind_offset_gain = 0.32F;
};

constexpr std::size_t kMaxTrajectoryPoints = 41;

struct TrajectoryResolution {
  std::array<ForwardPathPoint, kMaxTrajectoryPoints> points{};
  std::size_t point_count = 0;
  float speed_m_s = 0.0F;
  float airspeed_m_s = 0.0F;
  float stall_speed_m_s = 0.0F;
  float turn_rate_rad_s = 0.0F;
  float coordinated_turn_rate_rad_s = 0.0F;
  float climb_angle_rate_rad_s = 0.0F;
  float vertical_rate_m_s = 0.0F;
  float flight_path_angle_deg = 0.0F;
  std::optional<float> heading_deg;
  std::optional<float> track_deg;
  float heading_track_delta_deg = 0.0F;
  float wind_drift_m_s = 0.0F;
  bool is_stalled = true;
};

enum class Coordination { kWingsLevel, kCoordinated, kFlatTurn, kSlipping, kSkidding };
enum class Direction { kNone, kLeft, kRight };

float normalize_degrees(float degrees);
float heading_rate_from_body_rates(float roll_rad, float pitch_rad, float pitch_rate_rad_s,
                                   float yaw_rate_rad_s);
float climb_angle_rate_from_body_rates(float roll_rad, float pitch_rate_rad_s,
                                       float yaw_rate_rad_s);
HeadingResolution resolve_heading(const TelemetrySample& sample);
ScalarTelemetry resolve_scalars(const TelemetrySample& sample);
std::optional<float> resolve_track_degrees(const TelemetrySample& sample);
TrajectoryResolution resolve_trajectory(const TelemetrySample& sample,
                                        const TrajectoryConfig& config = {});
Coordination resolve_coordination(float actual_rad_s, float coordinated_rad_s);
Direction resolve_coordination_direction(float actual_rad_s, float coordinated_rad_s);
const char* coordination_name(Coordination coordination);

}  // namespace hud

