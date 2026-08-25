#include "hud/logic.hpp"
#include "hud/scene.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool near(float actual, float expected, float epsilon = 0.001F) {
  return std::fabs(actual - expected) <= epsilon;
}

class CountingTarget final : public hud::DrawTarget {
 public:
  int clears = 0;
  int lines = 0;
  int triangles = 0;
  int circles = 0;
  int texts = 0;
  void clear(hud::Color) override { ++clears; }
  void line(float, float, float, float, hud::Color, float) override { ++lines; }
  void triangle(float, float, float, float, float, float, hud::Color, bool) override { ++triangles; }
  void circle(float, float, float, hud::Color, bool) override { ++circles; }
  void text(float, float, const char*, hud::Color, float) override { ++texts; }
};

}  // namespace

int main() {
  assert(near(hud::normalize_degrees(-10.0F), 350.0F));

  hud::TelemetrySample heading_sample;
  heading_sample.vfr_hud = hud::VfrHudSample{450.0F, std::nullopt, std::nullopt,
                                             std::nullopt, 0};
  auto heading = hud::resolve_heading(heading_sample);
  assert(heading.source == hud::HeadingSource::kVfrHud);
  assert(near(*heading.heading_deg, 90.0F));

  heading_sample.vfr_hud.reset();
  heading_sample.attitude = hud::AttitudeSample{std::nullopt, std::nullopt, 1.5707963F,
                                                std::nullopt, std::nullopt, 0};
  heading = hud::resolve_heading(heading_sample);
  assert(heading.source == hud::HeadingSource::kAttitudeYaw);
  assert(heading.is_fallback);
  assert(near(*heading.heading_deg, 90.0F, 0.01F));

  hud::TelemetrySample sample;
  sample.link_alive = true;
  sample.attitude = hud::AttitudeSample{0.5235988F, 0.0872665F, 0.0F, 0.0F, 0.0F};
  sample.vfr_hud = hud::VfrHudSample{0.0F, 24.0F, 24.0F, 0.0F};
  sample.global_position = hud::GlobalPositionSample{0U, 2400, 0, 0};
  const auto trajectory = hud::resolve_trajectory(sample);
  assert(!trajectory.is_stalled);
  assert(trajectory.point_count == 21U);
  assert(near(trajectory.coordinated_turn_rate_rad_s, 9.81F * std::tan(0.5235988F) / 24.0F,
              0.001F));
  // Explicit zero body rates are trusted: a bank without rotation is a slip.
  assert(near(trajectory.turn_rate_rad_s, 0.0F));
  assert(hud::resolve_coordination(trajectory.turn_rate_rad_s,
                                   trajectory.coordinated_turn_rate_rad_s) ==
         hud::Coordination::kSlipping);
  assert(hud::resolve_coordination_direction(0.1F, 0.3F) == hud::Direction::kRight);
  assert(hud::resolve_coordination_direction(0.5F, 0.3F) == hud::Direction::kLeft);

  hud::TelemetrySample stalled;
  stalled.vfr_hud = hud::VfrHudSample{0.0F, 11.0F, 30.0F, 0.0F};
  const auto stalled_path = hud::resolve_trajectory(stalled);
  assert(stalled_path.is_stalled);
  assert(near(stalled_path.points[stalled_path.point_count - 1U].forward_m, 0.0F));

  CountingTarget target;
  hud::compose_unified_hud(sample, target);
  assert(target.clears == 1);
  assert(target.lines > 50);
  assert(target.circles > 0);
  assert(target.texts > 0);

  std::cout << "hud-core tests passed\n";
}
