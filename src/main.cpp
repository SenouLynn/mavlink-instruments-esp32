#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <MAVLink_ardupilotmega.h>
#include <SPI.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <new>

#include "hardware_config.hpp"
#include "hud/scene.hpp"

namespace {

constexpr int kAssignedPins[] = {HUD_TFT_SCLK_PIN, HUD_TFT_MOSI_PIN, HUD_TFT_CS_PIN,
                                 HUD_TFT_DC_PIN, HUD_TFT_RST_PIN, HUD_MAVLINK_RX_PIN};

constexpr bool assigned_pins_are_unique() {
  for (std::size_t left = 0; left < sizeof(kAssignedPins) / sizeof(kAssignedPins[0]); ++left) {
    for (std::size_t right = left + 1; right < sizeof(kAssignedPins) / sizeof(kAssignedPins[0]);
         ++right) {
      if (kAssignedPins[left] == kAssignedPins[right]) return false;
    }
  }
  return true;
}

static_assert(assigned_pins_are_unique(), "Display and MAVLink GPIO assignments must be unique");
static_assert(HUD_FRAME_RATE_HZ > 0U && HUD_FRAME_RATE_HZ <= 1000U,
              "Frame rate must produce a nonzero millisecond period");
static_assert(HUD_MAVLINK_RX_BUFFER_BYTES >= 512U,
              "MAVLink RX buffer must cover a blocking full-frame SPI transfer");
#if !HUD_USE_FRAMEBUFFER
#error "The SuperMini target requires the deterministic PSRAM framebuffer"
#endif

SPIClass display_spi(FSPI);
Adafruit_ST7789 display(&display_spi, HUD_TFT_CS_PIN, HUD_TFT_DC_PIN, HUD_TFT_RST_PIN);
std::unique_ptr<GFXcanvas16> framebuffer;

class PsramCanvas16 final : public GFXcanvas16 {
 public:
  PsramCanvas16(std::uint16_t width, std::uint16_t height)
      : GFXcanvas16(width, height, false) {
    const std::size_t bytes = static_cast<std::size_t>(width) * height * sizeof(std::uint16_t);
    buffer = static_cast<std::uint16_t*>(ps_malloc(bytes));
    buffer_owned = buffer != nullptr;
    if (buffer) std::memset(buffer, 0, bytes);
  }
};

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
std::uint8_t active_system_id = 0;
std::atomic<std::uint32_t> uart_error_count{0};
std::uint32_t maximum_render_us = 0;
mavlink_status_t parser_status{};
bool demo_enabled = true;
bool display_ready = false;

bool fresh(std::uint32_t now, std::uint32_t received) {
  return received != 0U && now - received <= HUD_TELEMETRY_STALE_MS;
}

bool accept_message(const mavlink_message_t& message, std::uint32_t now) {
  if (active_system_id != 0U && message.sysid != active_system_id) return false;

  if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID) return false;
    if (active_system_id == 0U) active_system_id = message.sysid;
    last_message_ms = now;
    ++message_count;
    return true;
  }

  const bool is_hud_data = message.msgid == MAVLINK_MSG_ID_ATTITUDE ||
                           message.msgid == MAVLINK_MSG_ID_VFR_HUD ||
                           message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT ||
                           message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT;
  if (!is_hud_data) return false;
  if (active_system_id == 0U) active_system_id = message.sysid;
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
  return true;
}

void read_mavlink() {
  mavlink_message_t message{};
  while (Serial1.available() > 0) {
    const std::uint8_t byte = static_cast<std::uint8_t>(Serial1.read());
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &parser_status)) {
      if (accept_message(message, millis())) demo_enabled = false;
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
  if (!display_ready) return;
  const std::uint32_t started_us = micros();
  hud::SceneConfig config;
  config.width = display.width();
  config.height = display.height();
  GfxTarget target(*framebuffer);
  hud::compose_unified_hud(sample, target, config);
  display.drawRGBBitmap(0, 0, framebuffer->getBuffer(), display.width(), display.height());
  const std::uint32_t elapsed_us = static_cast<std::uint32_t>(micros() - started_us);
  maximum_render_us = std::max(maximum_render_us, elapsed_us);
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
  if (psramFound()) {
    framebuffer.reset(new (std::nothrow) PsramCanvas16(display.width(), display.height()));
  }
#endif
  display_ready = framebuffer && framebuffer->getBuffer();
  Serial.printf("Display %dx%d, PSRAM=%u/%u, framebuffer=%s\n", display.width(), display.height(),
                ESP.getFreePsram(), ESP.getPsramSize(), display_ready ? "yes" : "FAILED");
  if (!display_ready) {
    display.setTextColor(hud::kHudRed);
    display.setTextSize(2);
    display.setCursor(8, 8);
    display.println("PSRAM/FRAMEBUFFER");
    display.println("STARTUP FAILURE");
  }

  Serial1.setRxBufferSize(HUD_MAVLINK_RX_BUFFER_BYTES);
  Serial1.begin(HUD_MAVLINK_BAUD, SERIAL_8N1, HUD_MAVLINK_RX_PIN, -1);
  Serial1.onReceiveError([](hardwareSerial_error_t) { ++uart_error_count; });
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
    Serial.printf("mode=%s sysid=%u mavlink_messages=%lu parse_errors=%lu uart_errors=%lu "
                  "age_ms=%lu heap=%u largest=%u psram_free=%u render_max_us=%lu\n",
                  demo_enabled ? "demo" : "live", active_system_id,
                  static_cast<unsigned long>(message_count),
                  static_cast<unsigned long>(parse_error_count),
                  static_cast<unsigned long>(uart_error_count.load()),
                  last_message_ms == 0U ? 0UL : static_cast<unsigned long>(now - last_message_ms),
                  ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  ESP.getFreePsram(), static_cast<unsigned long>(maximum_render_us));
  }
}
