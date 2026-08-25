#pragma once

#include <cstdint>

#include "hud/logic.hpp"

namespace hud {

using Color = std::uint16_t;
constexpr Color kBlack = 0x0000;
constexpr Color kHudCyan = 0x76BF;
constexpr Color kHudPale = 0xD79F;
constexpr Color kHudAmber = 0xFD0A;
constexpr Color kHudRed = 0xF986;
constexpr Color kHudMuted = 0x7451;

class DrawTarget {
 public:
  virtual ~DrawTarget() = default;
  virtual void clear(Color color) = 0;
  virtual void line(float x1, float y1, float x2, float y2, Color color,
                    float thickness = 1.0F) = 0;
  virtual void triangle(float x1, float y1, float x2, float y2, float x3, float y3,
                        Color color, bool filled = false) = 0;
  virtual void circle(float x, float y, float radius, Color color, bool filled = false) = 0;
  virtual void text(float x, float y, const char* value, Color color, float size = 1.0F) = 0;
};

struct SceneConfig {
  float width = 320.0F;
  float height = 240.0F;
  TrajectoryConfig trajectory{};
};

void compose_unified_hud(const TelemetrySample& sample, DrawTarget& target,
                         const SceneConfig& config = {});

}  // namespace hud
