#include "hud/scene.hpp"

#include <cstdio>

namespace {

void print_color(hud::Color color) {
  const unsigned red = ((color >> 11U) & 0x1FU) * 255U / 31U;
  const unsigned green = ((color >> 5U) & 0x3FU) * 255U / 63U;
  const unsigned blue = (color & 0x1FU) * 255U / 31U;
  std::printf("#%02x%02x%02x", red, green, blue);
}

class SvgTarget final : public hud::DrawTarget {
 public:
  SvgTarget(float width, float height) : width_(width), height_(height) {}

  void clear(hud::Color color) override {
    std::printf("<rect width=\"%.0f\" height=\"%.0f\" fill=\"", width_, height_);
    print_color(color);
    std::puts("\"/>");
  }

  void line(float x1, float y1, float x2, float y2, hud::Color color, float thickness) override {
    std::printf("<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"", x1, y1, x2, y2);
    print_color(color);
    std::printf("\" stroke-width=\"%.1f\"/>\n", thickness);
  }

  void triangle(float x1, float y1, float x2, float y2, float x3, float y3,
                hud::Color color, bool filled) override {
    std::printf("<polygon points=\"%.1f,%.1f %.1f,%.1f %.1f,%.1f\" %s=\"", x1, y1, x2, y2, x3, y3,
                filled ? "fill" : "stroke");
    print_color(color);
    std::printf("\"%s/>\n", filled ? "" : " fill=\"none\"");
  }

  void circle(float x, float y, float radius, hud::Color color, bool filled) override {
    std::printf("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.1f\" %s=\"", x, y, radius,
                filled ? "fill" : "stroke");
    print_color(color);
    std::printf("\"%s/>\n", filled ? "" : " fill=\"none\"");
  }

  void text(float x, float y, const char* value, hud::Color color, float size) override {
    std::printf("<text x=\"%.1f\" y=\"%.1f\" fill=\"", x, y);
    print_color(color);
    std::printf("\" font-size=\"%.1f\" font-family=\"monospace\">%s</text>\n", size * 8.0F, value);
  }

 private:
  float width_;
  float height_;
};

}  // namespace

int main() {
  constexpr float width = 240.0F;
  constexpr float height = 320.0F;
  hud::TelemetrySample sample;
  sample.timestamp_ms = 1000;
  sample.link_alive = true;
  sample.attitude = hud::AttitudeSample{0.35F, 0.08F, 1.57F, 0.04F, 0.18F, 1000};
  sample.vfr_hud = hud::VfrHudSample{90.0F, 24.0F, 22.0F, 1.8F, 1000};
  sample.global_position = hud::GlobalPositionSample{9000U, 2100, 400, -180, 1000};

  std::puts("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"240\" height=\"320\" viewBox=\"0 0 240 320\">");
  SvgTarget target(width, height);
  hud::SceneConfig config;
  config.width = width;
  config.height = height;
  hud::compose_unified_hud(sample, target, config);
  std::puts("</svg>");
}
