#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <MAVLink_ardupilotmega.h>
#include <SPI.h>

#include <cmath>
#include <memory>
#include <new>

#include "hardware_config.hpp"
#include "hud/scene.hpp"

namespace {

SPIClass display_spi(FSPI);
Adafruit_ST7789 display(&display_spi, HUD_TFT_CS_PIN, HUD_TFT_DC_PIN, HUD_TFT_RST_PIN);
std::unique_ptr<GFXcanvas16> framebuffer;

class GfxTarget final : public hud::DrawTarget {
 public:
  explicit GfxTarget(Adafruit_GFX& graphics) : graphics_(graphics) {}

  void clear(hud::Color color) override { graphics_.fillScreen(color); }

  void line(float x1, float y1, float x2, float y2, hud::Color color, float thickness) override {
    graphics_.drawLine(lroundf(x1), lroundf(y1), lroundf(x2), lroundf(y2), color);
    if (thickness >= 1.8F) {
      graphics_.drawLine(lroundf(x1), lroundf(y1) + 1, lroundf(x2), lroundf(y2) + 1, color);
    }
  }

  void triangle(float x1, float y1, float x2, float y2, float x3, float y3,
                hud::Color color, bool filled) override {
    if (filled) {
      graphics_.fillTriangle(lroundf(x1), lroundf(y1), lroundf(x2), lroundf(y2),
                             lroundf(x3), lroundf(y3), color);
    } else {
      graphics_.drawTriangle(lroundf(x1), lroundf(y1), lroundf(x2), lroundf(y2),
                             lroundf(x3), lroundf(y3), color);
    }
  }

  void circle(float x, float y, float radius, hud::Color color, bool filled) override {
    if (filled) graphics_.fillCircle(lroundf(x), lroundf(y), lroundf(radius), color);
    else graphics_.drawCircle(lroundf(x), lroundf(y), lroundf(radius), color);
  }

  void text(float x, float y, const char* value, hud::Color color, float size) override {
    graphics_.setTextWrap(false);
    graphics_.setTextColor(color);
    graphics_.setTextSize(static_cast<uint8_t>(std::max(1L, lroundf(size))));
    graphics_.setCursor(lroundf(x), lroundf(y));
    graphics_.print(value);
  }

 private:
  Adafruit_GFX& graphics_;
};

hud::TelemetrySample telemetry;
std::uint32_t last_message_ms = 0;
std::uint32_t message_count = 0;
std::uint32_t parse_error_count = 0;
mavlink_status_t parser_status{};
bool demo_enabled = true;

bool fresh(std::uint32_t now, std::uint32_t received) {
  return received != 0U && now - received <= HUD_TELEMETRY_STALE_MS;
}

void accept_message(const mavlink_message_t& message, std::uint32_t now) {
  last_message_ms = now;
  ++message_count;
  switch (message.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t value{};
      mavlink_msg_attitude_decode(&message, &value);
      telemetry.attitude = hud::AttitudeSample{value.roll, value.pitch, value.yaw,
                                               value.pitchspeed, value.yawspeed, now};
      break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t value{};
      mavlink_msg_vfr_hud_decode(&message, &value);
      telemetry.vfr_hud = hud::VfrHudSample{static_cast<float>(value.heading), value.airspeed,
                                            value.groundspeed, value.climb, now};
      break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t value{};
      mavlink_msg_global_position_int_decode(&message, &value);
      telemetry.global_position = hud::GlobalPositionSample{
          value.hdg, value.vx, value.vy, value.vz, now};
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t value{};
      mavlink_msg_gps_raw_int_decode(&message, &value);
      hud::GpsRawSample gps;
      const std::uint16_t course_cdeg = value.cog;
      const std::uint16_t speed_cm_s = value.vel;
      if (course_cdeg != UINT16_MAX) gps.course_cdeg = course_cdeg;
      if (speed_cm_s != UINT16_MAX) gps.speed_cm_s = speed_cm_s;
      gps.received_ms = now;
      telemetry.gps_raw = gps;
      break;
    }
    default:
      break;
  }
}

void read_mavlink() {
  mavlink_message_t message{};
  while (Serial1.available() > 0) {
    const std::uint8_t byte = static_cast<std::uint8_t>(Serial1.read());
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &parser_status)) {
      accept_message(message, millis());
      demo_enabled = false;
    }
  }
  parse_error_count = parser_status.parse_error;
}

hud::TelemetrySample live_snapshot(std::uint32_t now) {
  hud::TelemetrySample sample = telemetry;
  sample.timestamp_ms = now;
  sample.link_alive = fresh(now, last_message_ms);
  if (sample.attitude && !fresh(now, sample.attitude->received_ms)) sample.attitude.reset();
  if (sample.vfr_hud && !fresh(now, sample.vfr_hud->received_ms)) sample.vfr_hud.reset();
  if (sample.global_position && !fresh(now, sample.global_position->received_ms)) sample.global_position.reset();
  if (sample.gps_raw && !fresh(now, sample.gps_raw->received_ms)) sample.gps_raw.reset();
  return sample;
}

hud::TelemetrySample demo_snapshot(std::uint32_t now) {
  const float seconds = static_cast<float>(now) / 1000.0F;
  const float roll = std::sin(seconds * 0.55F) * 0.38F;
  const float pitch = std::sin(seconds * 0.31F) * 0.12F;
  const float turn_rate = 9.81F * std::tan(roll) / 24.0F;
  hud::TelemetrySample sample;
  sample.timestamp_ms = now;
  sample.link_alive = true;
  sample.attitude = hud::AttitudeSample{roll, pitch, seconds * 0.08F, 0.0F,
                                        turn_rate * std::cos(roll), now};
  sample.vfr_hud = hud::VfrHudSample{std::fmod(90.0F + seconds * 4.0F, 360.0F),
                                     24.0F, 22.0F, std::sin(seconds * 0.4F) * 2.0F, now};
  sample.global_position = hud::GlobalPositionSample{
      static_cast<std::uint16_t>(std::fmod(9000.0F + seconds * 400.0F, 36000.0F)),
      2200, 250, static_cast<std::int16_t>(-std::sin(seconds * 0.4F) * 200.0F), now};
  return sample;
}

void render(const hud::TelemetrySample& sample) {
  hud::SceneConfig config;
  config.width = display.width();
  config.height = display.height();
  if (framebuffer && framebuffer->getBuffer()) {
    GfxTarget target(*framebuffer);
    hud::compose_unified_hud(sample, target, config);
    display.drawRGBBitmap(0, 0, framebuffer->getBuffer(), display.width(), display.height());
  } else {
    GfxTarget target(display);
    hud::compose_unified_hud(sample, target, config);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("Unified HUD booting (send 'd' to toggle demo mode)");

#if HUD_TFT_BL_PIN >= 0
  pinMode(HUD_TFT_BL_PIN, OUTPUT);
  digitalWrite(HUD_TFT_BL_PIN, HUD_TFT_BL_ACTIVE_HIGH ? HIGH : LOW);
#endif
  display_spi.begin(HUD_TFT_SCLK_PIN, -1, HUD_TFT_MOSI_PIN, HUD_TFT_CS_PIN);
  display.init(HUD_TFT_NATIVE_WIDTH, HUD_TFT_NATIVE_HEIGHT);
  display.setRotation(HUD_TFT_ROTATION);
  display.setSPISpeed(HUD_TFT_SPI_HZ);
  display.fillScreen(hud::kBlack);

#if HUD_USE_FRAMEBUFFER
  framebuffer.reset(new (std::nothrow) GFXcanvas16(display.width(), display.height()));
#endif
  Serial.printf("Display %dx%d, framebuffer=%s\n", display.width(), display.height(),
                framebuffer && framebuffer->getBuffer() ? "yes" : "no (direct fallback)");

  Serial1.begin(HUD_MAVLINK_BAUD, SERIAL_8N1, HUD_MAVLINK_RX_PIN, -1);
  Serial.printf("MAVLink RX GPIO %d at %lu baud (receive-only)\n", HUD_MAVLINK_RX_PIN,
                static_cast<unsigned long>(HUD_MAVLINK_BAUD));
}

void loop() {
  static std::uint32_t last_frame_ms = 0;
  static std::uint32_t last_report_ms = 0;
  read_mavlink();
  if (Serial.available() > 0 && Serial.read() == 'd') {
    demo_enabled = !demo_enabled;
    Serial.printf("Demo mode %s\n", demo_enabled ? "on" : "off");
  }

  const std::uint32_t now = millis();
  const std::uint32_t frame_period = 1000U / HUD_FRAME_RATE_HZ;
  if (now - last_frame_ms >= frame_period) {
    last_frame_ms = now;
    render(demo_enabled ? demo_snapshot(now) : live_snapshot(now));
  }
  if (now - last_report_ms >= 2000U) {
    last_report_ms = now;
    Serial.printf("mode=%s mavlink_messages=%lu parse_errors=%lu age_ms=%lu heap=%u\n",
                  demo_enabled ? "demo" : "live", static_cast<unsigned long>(message_count),
                  static_cast<unsigned long>(parse_error_count),
                  last_message_ms == 0U ? 0UL : static_cast<unsigned long>(now - last_message_ms),
                  ESP.getFreeHeap());
  }
}
