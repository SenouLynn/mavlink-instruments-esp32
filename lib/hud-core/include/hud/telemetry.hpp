#pragma once

#include <cstdint>
#include <optional>

namespace hud {

struct AttitudeSample {
  std::optional<float> roll_rad;
  std::optional<float> pitch_rad;
  std::optional<float> yaw_rad;
  std::optional<float> pitch_rate_rad_s;
  std::optional<float> yaw_rate_rad_s;
  std::uint32_t received_ms = 0;
};

struct VfrHudSample {
  std::optional<float> heading_deg;
  std::optional<float> airspeed_m_s;
  std::optional<float> groundspeed_m_s;
  std::optional<float> climb_m_s;
  std::uint32_t received_ms = 0;
};

struct GlobalPositionSample {
  std::optional<std::uint16_t> heading_cdeg;
  std::optional<std::int16_t> velocity_north_cm_s;
  std::optional<std::int16_t> velocity_east_cm_s;
  std::optional<std::int16_t> velocity_down_cm_s;
  std::uint32_t received_ms = 0;
};

struct GpsRawSample {
  std::optional<std::uint16_t> course_cdeg;
  std::optional<std::uint16_t> speed_cm_s;
  std::uint32_t received_ms = 0;
};

struct TelemetrySample {
  std::uint32_t timestamp_ms = 0;
  std::optional<AttitudeSample> attitude;
  std::optional<VfrHudSample> vfr_hud;
  std::optional<GlobalPositionSample> global_position;
  std::optional<GpsRawSample> gps_raw;
  bool link_alive = false;
};

}  // namespace hud

